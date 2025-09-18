#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
echo "🔍 Root: $ROOT"
BE="$ROOT/backend"

if ! command -v pyenv >/dev/null 2>&1; then
  echo "❌ pyenv not found. Please install pyenv first."
  exit 1
fi

wanted_python_version="$(cat "$ROOT/.python-version" | tr -d '\r\n')"
current_pyton_version="$(pyenv version-name 2>/dev/null || true)"

if [ "$wanted_python_version" != "$current_pyton_version" ]; then
  echo "🔄 Switching Python $wanted_python_version → $current_pyton_version"
  pyenv install -s "$wanted_python_version"
  ( cd "$ROOT" && pyenv local "$wanted_python_version" )
  pyenv rehash
  hash -r
else
  echo "✅ Python version already matches: $current_pyton_version"
fi

if [ ! -d "$BE/.venv" ]; then
  echo "⚙️ Creating root .venv (dev only)"
  (cd "$BE" && uv lock && uv sync --group dev --frozen)
fi


PACKAGES=()
while IFS= read -r -d '' d; do
  PACKAGES+=("$(basename "$d")")
done < <(find "$BE/packages" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)

SERVICES=()
while IFS= read -r -d '' d; do
  SERVICES+=("$(basename "$d")")
done < <(find "$BE/services" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)

if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
  # Windows (Git Bash, MSYS, etc.)
  source "$BE/.venv/Scripts/activate"
else
  # Unix/Linux/macOS
  source "$BE/.venv/bin/activate"
fi

for pkg in "${PACKAGES[@]}"; do
  if [ -f "$BE/packages/$pkg/pyproject.toml" ]; then
    echo "📦 Installing $pkg"
    uv pip install -e "$BE/packages/$pkg" --link-mode=copy
  else
    echo "⏭️ $pkg (no pyproject)"
  fi
done

export UV_PROJECT_DIR="$BE"

for svc in "${SERVICES[@]}"; do
  if [ -f "$BE/services/$svc/pyproject.toml" ]; then
    echo "💉 Hydrating $svc"
    (cd "$BE" && uv lock --project "services/$svc"  --frozen)
    (cd "$BE" && uv export --project "services/$svc" --frozen -o "services/$svc/requirements.txt")
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