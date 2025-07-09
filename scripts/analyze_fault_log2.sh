#!/usr/bin/env bash
# analyze_fault_log.sh
# ────────────────────────────────────────────────────────────────
# * 스크립트가 위치한 **프로젝트/scripts** 디렉터리를 기준으로
#   - 실행 파일   : <project>/SLAMNAV2  (없으면 find 로 탐색)
#   - fault_log   : <project>/snlog/fault_log/*.txt
# 
# 다른 PC·경로에서도 그대로 복사해 쓸 수 있도록 **절대 경로 하드코딩을 제거**했습니다.
# 필요하면 환경 변수로 EXEC_PATH, LOGDIR_PATH 를 지정해 덮어쓸 수 있습니다.
# ────────────────────────────────────────────────────────────────

set -euo pipefail
shopt -s nullglob              # 글로빙 매칭 없으면 빈 리스트

# --------------------------------------------------------------
# 1) 프로젝트 루트 계산 : <this_script>/.. (scripts 상위)
# --------------------------------------------------------------
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJ_ROOT="$(realpath "$SCRIPT_DIR/..")"

# --------------------------------------------------------------
# 2) SLAMNAV2 실행 파일 경로 결정
#    ① ENV EXEC_PATH 우선, ② <proj>/SLAMNAV2, ③ find 로 탐색
# --------------------------------------------------------------
EXEC="${EXEC_PATH:-$PROJ_ROOT/SLAMNAV2}"
if [[ ! -x "$EXEC" ]]; then
    EXEC="$(find "$PROJ_ROOT" -maxdepth 3 -type f -name SLAMNAV2 -executable | head -n1 || true)"
fi
[[ -x "$EXEC" ]] || { echo "[ERR] SLAMNAV2 실행 파일을 찾지 못했습니다. EXEC_PATH 환경 변수를 지정하세요."; exit 1; }

# --------------------------------------------------------------
# 3) fault_log 디렉터리 경로 결정 (환경 변수로 덮어쓰기 가능)
# --------------------------------------------------------------
LOGDIR="${LOGDIR_PATH:-$PROJ_ROOT/snlog/fault_log}"
[[ -d "$LOGDIR" ]] || { echo "[ERR] fault_log 디렉터리를 찾지 못했습니다: $LOGDIR"; exit 1; }

# 정규식: /path/to/module(+0xOFF)
regex='^([^[:space:]]+)\(\+0x([0-9a-fA-F]+)\)'

for logfile in "$LOGDIR"/*.txt; do
    [[ -e "$logfile" ]] || { echo "No *.txt in $LOGDIR"; exit 1; }

    echo "============================================================"
    echo " LOGFILE : $logfile"
    echo "============================================================"

    while IFS= read -r line; do
        if [[ $line =~ $regex ]]; then
            module="${BASH_REMATCH[1]}"
            offset="0x${BASH_REMATCH[2]}"

            # 모듈이 경로 없이 찍힌 경우 ─ 메인 실행 파일로 간주
            [[ $module == /* ]] || module="$EXEC"

            map_out=$(addr2line -f -C -e "$module" "$offset" 2>/dev/null)

            func=$(echo "$map_out" | head -n1)
            src=$(echo "$map_out"  | tail -n1)

            printf "%s\n    ↳ %s (%s)\n" "$line" "$func" "$src"
        else
            echo "$line"
        fi
    done < "$logfile"

    echo            # 로그 간 공백
done

