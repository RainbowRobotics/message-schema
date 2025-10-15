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

CACHE_ROOT="${REPO_ROOT}/.buildx-cache"
KEEP_STORAGE="${KEEP_STORAGE:-10GB}"
BUILDER="${BUILDER:-rrs-py-buildx}"

ensure_builder() {
  docker context use default >/dev/null 2>&1 || true

  if docker buildx inspect "$BUILDER" >/dev/null 2>&1; then
    docker buildx rm -f "$BUILDER" >/dev/null 2>&1 || true
  fi

  docker ps -a --format '{{.ID}} {{.Names}}' \
    | awk -v b="$BUILDER" '$2 ~ ("buildx_buildkit_" b) {print $1}' \
    | xargs -r docker rm -f >/dev/null 2>&1

  if [ -d "$HOME/.docker/buildx/instances" ]; then
    find "$HOME/.docker/buildx/instances" -maxdepth 1 -type f -name "${BUILDER}*" -print0 \
      | xargs -0 -r rm -f >/dev/null 2>&1
  fi

  echo "🚀 새 빌더 '${BUILDER}' 생성(docker-container)"
  docker buildx create \
    --name "$BUILDER" \
    --driver docker-container \
    --node "${BUILDER}-0" \
    --driver-opt env.BUILDKIT_KEEP_STORAGE=$((8*1024*1024*1024)) \
    --use >/dev/null

  echo "🔧 빌더 부트스트랩 중..."
  docker buildx inspect "$BUILDER" --bootstrap >/dev/null
}

build_one() {
  local svc="$1" arch="$2"
  case "$arch" in amd64|arm64) ;; *) echo "❌ invalid arch: '$arch'"; return 2;; esac
  local outdir="$SERVICES_DIR/$svc/.out-$arch"
  local cache_dir="${CACHE_ROOT}/${svc}-${arch}" 

  echo "➡️  ${svc} (${arch}) → ${outdir}"
  rm -rf "$outdir" && mkdir -p "$outdir" "$cache_dir"

  local common=(
    --builder "$BUILDER"
    --platform "linux/${arch}"
    --build-arg "SERVICE=${svc}"
    -f "$WORKDIR/Dockerfile.build"
    --target artifacts
    --output "type=local,dest=${outdir}"
    --provenance=false         # 메타데이터 저장 축소
    --sbom=false               # SBOM 미생성(필요할 때만 켜기)
  )

  if [[ "${VERBOSE:-0}" == "1" ]]; then
    DOCKER_BUILDKIT=1 docker buildx build "${common[@]}" "$REPO_ROOT"
  else
    DOCKER_BUILDKIT=1 docker buildx build "${common[@]}" "$REPO_ROOT" >/dev/null
  fi

  mv "${outdir}/run.bin" "$SERVICES_DIR/${svc}/${svc}.${arch}.bin"
  rm -rf "$outdir"
}

ensure_builder

command -v docker >/dev/null || { echo "docker 필요"; exit 127; }
docker buildx version >/dev/null 2>&1 || { echo "docker buildx 필요"; exit 127; }
[[ -f "$WORKDIR/Dockerfile.build" ]] || { echo "Dockerfile.build 없음: $WORKDIR/Dockerfile.build"; exit 2; }
[[ -d "$SERVICES_DIR" ]] || { echo "services 디렉토리 없음: $SERVICES_DIR"; exit 2; }

if [[ -z "${SERVICE:-}" ]]; then
  echo "🔄 모든 서비스 빌드"
  SERVICES_LIST=$(find "$SERVICES_DIR" -name pyproject.toml -exec dirname {} \; | xargs -n1 basename | sort)
else
  echo "📦 선택 빌드: ${SERVICE}"
  IFS=',' read -r -a SERVICES_ARR <<< "$SERVICE"
  SERVICES_LIST=$(printf '%s\n' "${SERVICES_ARR[@]}" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' | sed '/^$/d')
fi
# 스크립트 안에서 SERVICES_LIST 만든 다음:
echo "count=$(printf '%s\n' "$SERVICES_LIST" | sed '/^$/d' | wc -l)"
printf '%s\n' "$SERVICES_LIST" | nl -ba | cat -v

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



echo "🧹 buildx 캐시 정리 (builder=${BUILDER}, keep=${KEEP_STORAGE})"

docker buildx prune -af --builder "$BUILDER" --keep-storage "$KEEP_STORAGE" >/dev/null
sleep 2
docker buildx prune -af --builder "$BUILDER" --keep-storage "$KEEP_STORAGE" >/dev/null


docker buildx stop "$BUILDER" >/dev/null 2>&1 || true
docker buildx rm -f "$BUILDER" >/dev/null 2>&1 || true

docker ps -a --format '{{.ID}} {{.Names}}' \
  | awk -v b="$BUILDER" '$2 ~ ("buildx_buildkit_" b) {print $1}' \
  | xargs -r docker rm -f >/dev/null 2>&1 || true

if [ -d "$HOME/.docker/buildx/instances" ]; then
  find "$HOME/.docker/buildx/instances" -maxdepth 1 -type f -name "${BUILDER}*" -print0 \
    | xargs -0 -r rm -f >/dev/null 2>&1 || true
fi

[[ $fail -eq 0 ]] && echo "🎉 병렬 빌드 완료" || { echo "⛔ 일부 실패"; exit 1; }