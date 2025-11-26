#!/usr/bin/env bash
# 여러 프로그램을 동시에 모니터링하고, 각 프로그램별 고정 폴더 안에 날짜시간 로그 생성

# ===== 설정 =====
TARGETS=("SLAMNAV2")  # 프로세스 이름 또는 PID
INTERVAL=1                                       # 초
SUM_CHILDREN=true                                # 자식 포함 여부
# =================

BASE_DIR="$HOME/mem_logs"
TS=$(date +"%Y%m%d_%H%M%S")  # 로그 파일명에 쓸 타임스탬프

sanitize() { echo "$1" | tr -cd '[:alnum:]_-' ; }

# 각 타겟에 대해 라벨/초기 PID/파일 준비
declare -A LABEL PID OUTFILE
echo "[INFO] logs base dir: $BASE_DIR"
for t in "${TARGETS[@]}"; do
  lab="$(sanitize "$t")"
  # PID 해석
  if [[ "$t" =~ ^[0-9]+$ ]]; then
    pid="$t"
  else
    pid="$(pgrep -f "$t" | head -n1)"
  fi
  LABEL["$lab"]="$t"
  PID["$lab"]="$pid"

  # 프로그램별 고정 디렉토리 생성
  dir="$BASE_DIR/$lab"
  mkdir -p "$dir"

  # 로그 파일 경로
  f="$dir/${TS}_log.csv"
  OUTFILE["$lab"]="$f"

  echo "epoch,clock,RSS_kB,PSS_kB,SWAP_kB,threads,proc_cnt" > "$f"
  printf "[INFO] %-14s -> PID=%s  log=%s\n" "$lab" "${pid:-NOTFOUND}" "$f"
done
echo "[INFO] interval=${INTERVAL}s  sum_children=${SUM_CHILDREN}"

get_children_pids() {
  local root="$1"
  ps --no-headers -o pid --ppid "$root" 2>/dev/null
}

collect_metrics() {
  local root="$1"
  local rss=0 pss=0 swp=0 thr=0 n=0

  if [[ -n "$root" ]] && [[ -r "/proc/$root/status" ]]; then
    local pids
    if $SUM_CHILDREN; then
      pids="$(get_children_pids "$root") $root"
    else
      pids="$root"
    fi

    for p in $pids; do
      [[ -r "/proc/$p/status" ]] || continue
      local r thd pp ww
      r=$(awk '/^VmRSS:/{print $2}' /proc/$p/status); r=${r:-0}
      thd=$(awk '/^Threads:/{print $2}' /proc/$p/status); thd=${thd:-0}
      if [[ -r "/proc/$p/smaps_rollup" ]]; then
        pp=$(awk '/^Pss:/{print $2}'  /proc/$p/smaps_rollup); pp=${pp:-0}
        ww=$(awk '/^Swap:/{print $2}' /proc/$p/smaps_rollup); ww=${ww:-0}
      else
        pp=0; ww=0
      fi
      rss=$((rss + r)); pss=$((pss + pp)); swp=$((swp + ww))
      thr=$((thr + thd)); n=$((n + 1))
    done
  fi

  echo "$rss,$pss,$swp,$thr,$n"
}

while :; do
  now_epoch="$(date +%s)"
  now_clock="$(date +%H:%M:%S)"

  for lab in "${!LABEL[@]}"; do
    # PID 갱신(프로세스 재시작 대비)
    if [[ -z "${PID[$lab]}" ]] || [[ ! -r "/proc/${PID[$lab]}/status" ]]; then
      t="${LABEL[$lab]}"
      if [[ "$t" =~ ^[0-9]+$ ]]; then
        newpid="$t"
      else
        newpid="$(pgrep -f "$t" | head -n1)"
      fi
      PID["$lab"]="$newpid"
    fi

    metrics=$(collect_metrics "${PID[$lab]}")
    echo "$now_epoch,$now_clock,$metrics" >> "${OUTFILE[$lab]}"
  done

  sleep "$INTERVAL"
done

