#!/usr/bin/env bash
# set -euo pipefail: 기존 스크립트의 오류 처리 방식 유지
set -euo pipefail

: "${SERVICE:=}"
: "${ARCHS:=amd64 arm64}"
: "${BUILDER_NAME:=rrs_python_builder}"
: "${CACHE_ROOT:=.buildx-cache}"

: "${MAX_CACHE_SIZE_GB:=1}"
: "${MIN_FREE_DAYS:=7}"



OS="$(uname -s)"

STAT_FORMAT_ATIME='%a %N'
STAT_FORMAT_SIZE='%z'
STAT_FORMAT_OPTION='-f'

if [[ "$OS" == "Linux" ]]; then
    STAT_FORMAT_OPTION='-c'
    STAT_FORMAT_ATIME='%X %n'
    STAT_FORMAT_SIZE='%s'
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKDIR="${REPO_ROOT}/backend"
SERVICES_DIR="$WORKDIR/services"
HOST_DIR="$WORKDIR/host"
DOCKERFILE="$WORKDIR/Dockerfile.build"

prune_local_cache_by_size() {
  local arch=$1
  local cache_dir="${REPO_ROOT}/${CACHE_ROOT}-${arch}"
  local target_dir="${cache_dir}/blobs/sha256"
  local max_bytes=$((MAX_CACHE_SIZE_GB * 1024 * 1024 * 1024))

  if [[ ! -d "$target_dir" ]]; then
    return 0
  fi

  local current_kb
  current_kb=$(du -s -k "$cache_dir" | awk '{print $1}')
  local current_bytes=$((current_kb * 1024))
  local current_size_gb=$(echo "scale=2; $current_bytes / (1024*1024*1024)" | bc)

  echo "➡️  캐시 크기 확인 (${arch}): 현재 ${current_size_gb} GB / 최대 ${MAX_CACHE_SIZE_GB} GB"

  if (( current_bytes < max_bytes )); then
    echo "✅ 캐시 크기가 허용 범위 내에 있습니다"
    return 0
  fi

  echo "⚠️  최대 용량(${MAX_CACHE_SIZE_GB} GB) 초과! 정리 시작..."

  local total_removed_count=0

  local deleted_count
  deleted_count=$(find "$target_dir" -type f -atime +"${MIN_FREE_DAYS}" -delete -print 2>/dev/null | wc -l)
  total_removed_count=${deleted_count}

  current_kb=$(du -s -k "$cache_dir" | awk '{print $1}')
  current_bytes=$((current_kb * 1024))
  current_size_gb=$(echo "scale=2; $current_bytes / (1024*1024*1024)" | bc)

  if (( current_bytes >= max_bytes )); then

    local bytes_to_remove=$((current_bytes - max_bytes))
    local removed_bytes=0
    local removed_count=0

    find "$target_dir" -type f -print0 | xargs -0 stat "$STAT_FORMAT_OPTION" "$STAT_FORMAT_ATIME" | sort -n | while read -r atime filename; do
      if (( removed_bytes >= bytes_to_remove )); then
          break
      fi

      local file_size
      file_size=$(stat "$STAT_FORMAT_OPTION" "$STAT_FORMAT_SIZE" "$filename")

      rm -f "$filename" 2>/dev/null

      if [[ $? -eq 0 ]]; then
        removed_bytes=$((removed_bytes + file_size))
        removed_count=$((removed_count + 1))
      fi
    done

    total_removed_count=$((total_removed_count + removed_count))

    current_kb=$(du -s -k "$cache_dir" | awk '{print $1}')
    current_bytes=$((current_kb * 1024))
    current_size_gb=$(echo "scale=2; $current_bytes / (1024*1024*1024)" | bc)
  fi

  if (( total_removed_count > 0 )); then
    echo "✔ 총 ${total_removed_count}개 파일 삭제됨. (최종 크기: ${current_size_gb} GB)"
    local max_bytes_check=$((MAX_CACHE_SIZE_GB * 1024 * 1024 * 1024))
    if (( current_bytes >= max_bytes_check )); then
      echo "❌ 최종 경고: 캐시가 여전히 최대 용량(${MAX_CACHE_SIZE_GB} GB)을 초과합니다. 수동 정리가 필요할 수 있습니다."
    fi
  fi
}


cleanup() {
  echo "🧹 빌더 컨테이너 캐시 정리 중..."

  docker buildx prune -af --builder "$BUILDER_NAME" >/dev/null 2>&1 || true

  if [[ $? -ne 0 ]]; then
      echo "❌ 일부 병렬 빌드가 실패했거나 중단되었습니다."
  else
      echo "✅ 정리/빌드 완료 => services: ${SERVICES[@]}"
  fi
}

trap cleanup EXIT INT TERM


command -v docker >/dev/null || { echo "docker 필요"; exit 127; }
docker buildx version >/dev/null 2>&1 || { echo "docker buildx 필요"; exit 127; }
command -v du >/dev/null || { echo "du(disk usage) 필요"; exit 127; }
command -v bc >/dev/null || { echo "bc(계산기) 필요"; exit 127; }


if docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  cur_drv="$(docker buildx ls | awk -v n="${BUILDER_NAME}" '$1==n {print $2}')"
  if [[ "${cur_drv}" != "docker-container" && -n "${cur_drv}" ]]; then
    docker buildx rm -f "${BUILDER_NAME}" >/dev/null 2>&1 || true
    docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use --bootstrap >/dev/null
  else
    docker buildx use "${BUILDER_NAME}" >/dev/null
    docker buildx inspect --bootstrap >/dev/null
  fi
else
  docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use --bootstrap >/dev/null
fi

if [[ -z "$SERVICE" ]]; then
  echo "🔄 모든 서비스 빌드 (병렬)"
  SERVICES=()
  while IFS= read -r line; do SERVICES+=("$line"); done \
    < <(find "$SERVICES_DIR" -name pyproject.toml -exec dirname {} \; | xargs -n1 basename | sort)
else
  echo "📦 선택 빌드: ${SERVICE} (병렬)"
  IFS=',' read -r -a SERVICES <<< "$SERVICE"
  for i in "${!SERVICES[@]}"; do
    SERVICES[$i]="$(printf '%s' "${SERVICES[$i]}" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
  done
fi

build_service() {
  local svc=$1
  local arch=$2
  local cache_dir="${REPO_ROOT}/${CACHE_ROOT}-${arch}"
  local outdir="$SERVICES_DIR/$svc/.out-$arch"

  [[ -d "$SERVICES_DIR/$svc" ]] || { echo "skip: $svc not found"; return 0; }

  mkdir -p "$cache_dir"
  rm -rf "$outdir"; mkdir -p "$outdir"

  echo "➡️  빌드 시작: ${svc} (${arch}) | 캐시: $(basename "$cache_dir")"

  cmd=(
    docker buildx build
    --builder "$BUILDER_NAME"
    --platform "linux/${arch}"
    --build-arg "SERVICE=${svc}"
    -f "$DOCKERFILE"
    --target artifacts
    --output "type=local,dest=${outdir}"
    --cache-to "type=local,dest=${cache_dir},mode=min"
  )

  if [[ -f "${cache_dir}/index.json" ]]; then
    cmd+=( --cache-from "type=local,src=${cache_dir}" )
  fi

  cmd+=( "$REPO_ROOT" )

  if ! "${cmd[@]}"; then
    echo "❌ 빌드 실패: ${svc} (${arch})" >&2
    return 1
  fi

  mv "${outdir}/run.bin" "$SERVICES_DIR/${svc}/${svc}.${arch}.bin"
  rm -rf "$outdir"
  echo "✔ 완료"
  return 0
}


build_host() {
  local outdir="$HOST_DIR/.out"

  [[ -d "$HOST_DIR" ]] || { echo "skip: host not found"; return 0; }

  rm -rf "$outdir"; mkdir -p "$outdir"

  echo "➡️  빌드 시작: host "

  cmd=(
    uv run --project "${HOST_DIR}" pyinstaller
    --onefile
    --clean
    --name "host.bin"
    --distpath "${outdir}"
    --workpath "${outdir}/.work"
    --specpath "${outdir}"
    ${HOST_DIR}/run.py
  )

  if ! "${cmd[@]}"; then
    echo "❌ 빌드 실패: host" >&2
    return 1
  fi

  mv "${outdir}/host.bin" "$HOST_DIR/host.bin"
  rm -rf "$outdir"
  echo "✔ 완료"
  return 0
}

echo "🔄 로컬 캐시 용량 기반 사전 정리 시작..."
for arch in $ARCHS; do
  prune_local_cache_by_size "$arch" &
done
wait
echo "✅ 로컬 캐시 사전 정리 완료."

for svc in "${SERVICES[@]}"; do
  if [ "$svc" = "host" ]; then
    build_host "$svc" &
    BUILD_PIDS+=($!)
  else
    for arch in $ARCHS; do
      build_service "$svc" "$arch" &
      BUILD_PIDS+=($!)
    done
  fi
done


wait
