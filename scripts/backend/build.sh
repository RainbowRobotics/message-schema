#!/usr/bin/env bash
# 병렬로 services/* 를 docker buildx 로 빌드해서 run.bin 아티팩트를 추출한다.
# ENV:
#   WORKDIR  : backend 디렉토리 경로 (기본: 이 스크립트가 있는 디렉토리)
#   SERVICE  : 특정 서비스만 빌드 (예: SERVICE=manipulate)
#   JOBS     : 동시에 빌드할 작업 개수 (기본 4)
#   VERBOSE  : 1 이면 docker 로그 출력(기본은 조용히)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKDIR="${REPO_ROOT}/backend"
SERVICES_DIR="$WORKDIR/services"

build_one() {
  local svc="$1" arch="$2"
  case "$arch" in amd64|arm64) ;; *) echo "❌ invalid arch: '$arch'"; return 2;; esac
  local outdir="$SERVICES_DIR/$svc/.out-$arch"

  echo "➡️  ${svc} (${arch}) → ${outdir}"
  rm -rf "$outdir" && mkdir -p "$outdir"

  if [[ "${VERBOSE:-0}" == "1" ]]; then
    DOCKER_BUILDKIT=1 docker buildx build \
      --platform "linux/${arch}" \
      --build-arg "SERVICE=${svc}" \
      -f "$WORKDIR/Dockerfile.build" \
      --target artifacts \
      --output "type=local,dest=${outdir}" \
      "$REPO_ROOT"
  else
    DOCKER_BUILDKIT=1 docker buildx build \
      --platform "linux/${arch}" \
      --build-arg "SERVICE=${svc}" \
      -f "$WORKDIR/Dockerfile.build" \
      --target artifacts \
      --output "type=local,dest=${outdir}" \
      "$REPO_ROOT" >/dev/null
  fi

  mv "${outdir}/run.bin" "$SERVICES_DIR/${svc}/${svc}.${arch}.bin"
  rm -rf "$outdir"
}

command -v docker >/dev/null || { echo "docker 필요"; exit 127; }
docker buildx version >/dev/null 2>&1 || { echo "docker buildx 필요"; exit 127; }
[[ -f "$WORKDIR/Dockerfile.build" ]] || { echo "Dockerfile.build 없음: $WORKDIR/Dockerfile.build"; exit 2; }
[[ -d "$SERVICES_DIR" ]] || { echo "services 디렉토리 없음: $SERVICES_DIR"; exit 2; }

if [[ -z "${SERVICE:-}" ]]; then
  echo "🔄 모든 서비스 빌드"
  SERVICES_LIST=$(find "$SERVICES_DIR" -name pyproject.toml -exec dirname {} \; | xargs -n1 basename | sort)
else
  echo "📦 선택 빌드: ${SERVICE}"
  SERVICES_LIST=$(printf '%s\n' ${SERVICE})
fi

ARCHS="${ARCHS:-amd64 arm64}"
for a in $ARCHS; do case "$a" in amd64|arm64) ;; *) echo "❌ invalid arch '$a'"; exit 2;; esac; done

JOBS="${JOBS:-4}"

sem_fifo="$(mktemp -u)"; mkfifo "$sem_fifo"; exec 3<>"$sem_fifo"; rm -f "$sem_fifo"
for _ in $(seq 1 "$JOBS"); do echo >&3; done

fail=0
pids=()

while IFS= read -r svc; do
  [[ -z "$svc" ]] && continue
  case "$svc" in amd64|arm64) echo "❌ SERVICES에 arch 섞임: '$svc'"; exit 2;; esac
  for arch in $ARCHS; do
    read -u3
    (
      if ! build_one "$svc" "$arch"; then
        echo "❌ 실패: $svc ($arch)" >&2
        exit 1
      fi
      echo >&3
    ) & pids+=($!)
  done
done <<< "$SERVICES_LIST"

for pid in "${pids[@]}"; do
  if ! wait "$pid"; then fail=1; fi
done
exec 3>&-

[[ $fail -eq 0 ]] && echo "🎉 병렬 빌드 완료" || { echo "⛔ 일부 실패"; exit 1; }