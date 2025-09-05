#!/usr/bin/env bash
set -euo pipefail

SERVICE=()
ARCH=""

# 인자 파싱
while [[ $# -gt 0 ]]; do
  case "$1" in
    --arch=)    
      ARCH="${2:-}"; shift 2 ;;
    *)
      SERVICE+=("$1")
      shift
      ;;
  esac
done

# 워크스페이스 루트(바젤 런컨텍스트에서도 안정적으로 추적)
WS_ROOT="$(pwd)"
if git rev-parse --show-toplevel >/dev/null 2>&1; then
  WS_ROOT="$(git rev-parse --show-toplevel)"
fi
cd "$WS_ROOT"

# 서비스 목록 수집
if [[ ${#SERVICE[@]} -eq 0 ]]; then
  echo "🔄 모든 서비스에 대해 빌드 중..."
  mapfile -t SERVICES < <(find backend/services -name pyproject.toml -exec dirname {} \; | xargs -n1 basename | sort -u)
else
  echo "📦 서비스 '${SERVICE}' 빌드 중..."
  SERVICES=$SERVICE
fi

# 아키 목록
if [[ -z "${ARCH}" ]]; then
  ARCHES=(amd64 arm64)
else
  ARCHES=("${ARCH}")
fi

# Dockerfile.build 위치(필요하면 경로 수정)
DFILE="backend/services/Dockerfile.build"
if [[ ! -f "${DFILE}" ]]; then
  # 서비스별로 Dockerfile.build가 서비스 폴더 안에 있다면 아래 라인으로 교체:
  # DFILE="backend/services/${s}/Dockerfile.build"
  echo "Dockerfile.build not found at ${DFILE}"; exit 1
fi

for s in "${SERVICES[@]}"; do
  for a in "${ARCHES[@]}"; do
    echo "📦 ${s} (${a}) 빌드 → 바이너리만 추출"
    outdir="backend/services/${s}/.out-${a}"
    echo "outdir: ${outdir}"
    rm -rf "${outdir}" && mkdir -p "${outdir}"

    # 원본 루프와 동일하게 buildx artifacts 타겟만 가져오기
    DOCKER_BUILDKIT=1 docker buildx build \
      --platform "linux/${a}" \
      --build-arg "SERVICE=${s}" \
      -f "${DFILE}" \
      --target artifacts \
      --output "type=local,dest=${outdir}" \
      . >/dev/null

    mv "${outdir}/run.bin" "backend/services/${s}/${s}.${a}.bin"
    rm -rf "${outdir}"
  done
done

echo "✅ 완료"
