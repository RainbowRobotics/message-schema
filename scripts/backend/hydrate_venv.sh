#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"   # rby2/ 까지
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
    uv pip install -r "$BE/services/$svc/requirements.txt"
  else
    echo "⏭️ $svc (no pyproject)"
  fi
done

echo "✅ Root .venv hydrated."