#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
echo "🔍 Root: $ROOT"
BE="$ROOT/backend"

if [ ! -d "$BE/.venv" ]; then
  echo "⚙️ Creating root .venv (dev only)"
  (cd "$BE" && uv lock && uv sync --group dev)
fi


PACKAGES=()
while IFS= read -r -d '' d; do
  PACKAGES+=("$(basename "$d")")
done < <(find "$BE/packages" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)

SERVICES=()
while IFS= read -r -d '' d; do
  SERVICES+=("$(basename "$d")")
done < <(find "$BE/services" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)

source "$BE/.venv/bin/activate"

for pkg in "${PACKAGES[@]}"; do
  if [ -f "$BE/packages/$pkg/pyproject.toml" ]; then
    echo "📦 Installing $pkg"
    uv pip install -e "$BE/packages/$pkg"
  else
    echo "⏭️ $pkg (no pyproject)"
  fi
done

for svc in "${SERVICES[@]}"; do
  if [ -f "$BE/services/$svc/pyproject.toml" ]; then
    echo "💉 Hydrating $svc"
    (cd "$BE/services/$svc" && uv lock && uv pip compile pyproject.toml -o requirements.txt)
    (cd "$BE/services/$svc" && uv pip install -r requirements.txt)
  else
    echo "⏭️ $svc (no pyproject)"
  fi
done

# SITE_PACKAGES="$(python - <<'PY'
# import site; print([p for p in (site.getsitepackages()+[site.getusersitepackages()]) if 'site-packages' in p][0])
# PY
# )"
# PTH_FILE="$SITE_PACKAGES/backend_paths.pth"
# {
#   echo "$BE/packages"      # 내부 패키지 루트
#   for d in "${SERVICES[@]}"; do echo "$d"; done  # 서비스 루트
# } > "$PTH_FILE"

# echo "✅ Hydrated (no editable). PTH: $PTH_FILE"

echo "✅ Root .venv hydrated."