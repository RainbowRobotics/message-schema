#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_memlog.py
- 터미널에서 1개 이상 로그 파일 경로를 인자로 받음
- 각 로그를 읽어 PSS 중심 그래프를 만들고, 로그 파일 옆에 같은 이름으로 .png 저장
- 콘솔에 요약/누수 판단(LEAK: YES/NO)도 출력
- 기본은 저장만 하고 창은 띄우지 않음 (--show로 표시 가능)

지원 CSV 포맷(자동 감지):
  A) epoch,clock,RSS_kB,PSS_kB,SWAP_kB[,threads,proc_cnt]   ← (추천: 우리가 만든 로거)
  B) time,rss,pss,swap                                      ← 단위는 KB 또는 MB 가능
  C) TIME,RSS,PSS,SWAP                                      ← time은 문자열/숫자 모두 허용

사용 예:
  MPLBACKEND=Agg python3 plot_memlog.py ~/mem_logs/SLAMNAV_3D/20250811_122007_log.csv
  python3 plot_memlog.py file1.csv file2.csv --show
"""

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import matplotlib.pyplot as plt


# ---------- CSV 로딩/해석 ----------

def _to_float_list(vs: List[str]) -> np.ndarray:
    out = []
    for v in vs:
        try:
            out.append(float(v))
        except Exception:
            out.append(np.nan)
    return np.asarray(out, dtype=float)


def read_mem_csv(path: Path) -> Tuple[np.ndarray, Dict[str, np.ndarray], str]:
    """
    CSV 파일을 읽어 (time_minutes, series_dict, unit_string) 반환.
    series_dict keys: 'RSS','PSS','SWAP' (단위는 MB로 통일)
    time_minutes: 시작 시각 기준 분(min) 단위
    """
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        cols = [c.strip() for c in reader.fieldnames or []]

        # 표준 포맷 A: epoch,clock,RSS_kB,PSS_kB,SWAP_kB ...
        fmtA = all(k in cols for k in ["epoch", "RSS_kB", "PSS_kB", "SWAP_kB"])
        # 포맷 B: time,rss,pss,swap
        fmtB = all(k in [c.lower() for c in cols] for k in ["time", "rss", "pss", "swap"])
        # 포맷 C: TIME,RSS,PSS,SWAP
        fmtC = all(k in cols for k in ["TIME", "RSS", "PSS", "SWAP"])

        rows = list(reader)
        if len(rows) < 2:
            raise ValueError("rows < 2")

        if fmtA:
            epoch = _to_float_list([r["epoch"] for r in rows])
            rss  = _to_float_list([r["RSS_kB"] for r in rows])
            pss  = _to_float_list([r["PSS_kB"] for r in rows])
            swp  = _to_float_list([r["SWAP_kB"] for r in rows])
            # 시간: epoch 기준 분 단위
            t_min = (epoch - epoch[0]) / 60.0
            unit = "KB"
        elif fmtB:
            lower = {c.lower(): c for c in cols}
            time_vals = _to_float_list([r[lower["time"]] for r in rows])
            rss  = _to_float_list([r[lower["rss"]]  for r in rows])
            pss  = _to_float_list([r[lower["pss"]]  for r in rows])
            swp  = _to_float_list([r[lower["swap"]] for r in rows])

            # time이 이미 초일 수 있음 → 분으로 정규화
            t_min = (time_vals - time_vals[0]) / 60.0
            # 단위 추정(값이 100000 이상이면 KB, 아니면 MB로 가정)
            unit = "KB" if np.nanmedian(pss) > 1e5 else "MB"
        elif fmtC:
            time_vals = _to_float_list([r["TIME"] for r in rows])
            rss  = _to_float_list([r["RSS"] for r in rows])
            pss  = _to_float_list([r["PSS"] for r in rows])
            swp  = _to_float_list([r["SWAP"] for r in rows])

            t_min = (time_vals - time_vals[0]) / 60.0
            unit = "KB" if np.nanmedian(pss) > 1e5 else "MB"
        else:
            raise ValueError(f"unsupported columns: {cols}")

        # 단위 KB → MB 변환
        if unit.upper() == "KB":
            rss, pss, swp = rss / 1024.0, pss / 1024.0, swp / 1024.0
            unit_str = "MB"
        else:
            unit_str = "MB"

        series = {"RSS": rss, "PSS": pss, "SWAP": swp}
        return t_min, series, unit_str


# ---------- 통계/누수 판정 ----------

def linfit(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """1차 회귀: slope, intercept, R^2 (x: min, y: MB)"""
    p = np.polyfit(x, y, 1)
    yhat = np.polyval(p, x)
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return float(p[0]), float(p[1]), r2


def rolling_slopes(x_min: np.ndarray, y_mb: np.ndarray, window_min: float) -> np.ndarray:
    """연속 구간 윈도우별 1차 회귀 기울기(MB/min) 배열 반환"""
    if x_min[-1] - x_min[0] < window_min:
        return np.array([])
    slopes = []
    i0 = 0
    for i in range(len(x_min)):
        while x_min[i] - x_min[i0] > window_min:
            i0 += 1
        if x_min[i] - x_min[i0] >= window_min:
            xs = x_min[i0:i+1]
            ys = y_mb[i0:i+1]
            s, _, _ = linfit(xs, ys)
            slopes.append(s)
    return np.asarray(slopes, dtype=float)


def verdict_pss(x_min: np.ndarray, pss_mb: np.ndarray,
                slope_thresh=0.5, r2_req=0.80, roll_window_min=5.0) -> Tuple[bool, Dict[str, float]]:
    """PSS 기준 누수 판정"""
    g_slope, _, g_r2 = linfit(x_min, pss_mb)
    roll = rolling_slopes(x_min, pss_mb, roll_window_min)
    condA = (g_slope >= slope_thresh) and (g_r2 >= r2_req)
    if roll.size:
        med = float(np.median(roll))
        pos_ratio = float(np.mean(roll > 0.0))
    else:
        med, pos_ratio = 0.0, 0.0
    condB = (med >= slope_thresh) and (pos_ratio >= 0.80)
    return (condA or condB), {
        "g_slope": g_slope, "g_r2": g_r2,
        "roll_med": med, "roll_pos_ratio": pos_ratio,
        "condA": condA, "condB": condB
    }


# ---------- 플로팅/저장 ----------

def plot_and_save(path: Path, show: bool, thresh_mbpm: float, r2_req: float, window_min: float):
    try:
        t_min, series, unit = read_mem_csv(path)
    except Exception as e:
        print(f"[ERR] {path.name}: {e}")
        return

    rss, pss, swp = series["RSS"], series["PSS"], series["SWAP"]
    dur = float(t_min[-1] - t_min[0])

    # 누수 판정(PSS)
    is_leak, meta = verdict_pss(
        t_min, pss, slope_thresh=thresh_mbpm, r2_req=r2_req, roll_window_min=window_min
    )

    # 콘솔 요약
    g_slope_rss, _, g_r2_rss = linfit(t_min, rss)
    g_slope_pss, _, g_r2_pss = meta["g_slope"], None, meta["g_r2"]
    g_slope_swp, _, g_r2_swp = linfit(t_min, swp)

    print(f"\n[{path.name}] Duration={dur:.1f} min, Samples={len(t_min)}")
    print(f" RSS: slope={g_slope_rss:.3f} MB/min  R^2={g_r2_rss:.3f}  Δ={rss[-1]-rss[0]:.1f} MB")
    print(f" PSS: slope={g_slope_pss:.3f} MB/min  R^2={g_r2_pss:.3f}  Δ={pss[-1]-pss[0]:.1f} MB")
    print(f" SWP: slope={g_slope_swp:.3f} MB/min  R^2={g_r2_swp:.3f}  Δ={swp[-1]-swp[0]:.1f} MB")
    print(f" Verdict: LEAK={'YES' if is_leak else 'NO'} "
          f"(ruleA={meta['condA']}, ruleB={meta['condB']}, "
          f"roll_med={meta['roll_med']:.3f}, pos_ratio={meta['roll_pos_ratio']:.2f})")

    # -------- 그림(좌: 요약 텍스트, 우: PSS 중심 플롯) --------
    fig, (ax_txt, ax) = plt.subplots(
        1, 2, figsize=(12, 5), gridspec_kw={"width_ratios": [1, 2.3]}
    )

    # 왼쪽 텍스트
    ax_txt.axis("off")
    lines = [
        f"File: {path.name}",
        f"Duration: {dur:.1f} min  Samples: {len(t_min)}",
        "",
        f"PSS: slope={g_slope_pss:.3f} MB/min  R²={g_r2_pss:.3f}  Δ={pss[-1]-pss[0]:.1f} MB",
        f"RSS: slope={g_slope_rss:.3f} MB/min  R²={g_r2_rss:.3f}  Δ={rss[-1]-rss[0]:.1f} MB",
        f"SWP: slope={g_slope_swp:.3f} MB/min  R²={g_r2_swp:.3f}  Δ={swp[-1]-swp[0]:.1f} MB",
        "",
        f"LEAK: {'YES' if is_leak else 'NO'}",
        f" - rule A: slope≥{thresh_mbpm} & R²≥{r2_req} -> {meta['condA']}",
        f" - rule B: median≥{thresh_mbpm} & pos_ratio≥0.80 -> {meta['condB']}",
        f"   (median={meta['roll_med']:.3f}, pos_ratio={meta['roll_pos_ratio']:.2f})"
        if dur >= window_min else "(rolling window not enough)"
    ]
    ax_txt.text(0, 1, "\n".join(lines), va="top", ha="left", fontsize=10, family="monospace")

    # 오른쪽 플롯: PSS 메인, RSS/SWAP 보조
    ax.plot(t_min, pss, label="PSS (MB)", linewidth=2.5)
    # 전역 PSS 추세선
    yhat = np.polyval([g_slope_pss, pss[0]], t_min - t_min[0])
    ax.plot(t_min, yhat, linestyle="--", label=f"PSS trend {g_slope_pss:.2f} MB/min (R²={g_r2_pss:.2f})")
    ax.plot(t_min, rss, label="RSS (MB)", alpha=0.35, linewidth=1.0)
    ax.plot(t_min, swp, label="SWAP (MB)", alpha=0.35, linewidth=1.0)

    ax.set_xlabel("Time (min)")
    ax.set_ylabel("Memory (MB)")
    ax.set_title(f"PSS-centric • LEAK: {'YES' if is_leak else 'NO'}")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend(loc="best")
    fig.suptitle(str(path), y=0.98, fontsize=10)
    fig.tight_layout()

    # 저장 경로: 같은 폴더, 같은 이름의 .png
    out_path = path.with_suffix(".png")
    fig.savefig(out_path, dpi=140)
    print(f"[Saved] {out_path}")

    if show and os.environ.get("DISPLAY"):
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Plot memory logs and save PNG next to CSV")
    ap.add_argument("logs", nargs="+", help="CSV file(s) to plot")
    ap.add_argument("--show", action="store_true", help="show window (also saves PNG)")
    ap.add_argument("--thresh-mbpm", type=float, default=0.5, help="leak slope threshold (MB/min)")
    ap.add_argument("--r2", type=float, default=0.80, help="global R^2 threshold")
    ap.add_argument("--window-min", type=float, default=5.0, help="rolling window size (minutes)")
    args = ap.parse_args()

    for p in args.logs:
        plot_and_save(Path(p).expanduser().resolve(),
                      show=args.show,
                      thresh_mbpm=args.thresh_mbpm,
                      r2_req=args.r2,
                      window_min=args.window_min)


if __name__ == "__main__":
    main()

