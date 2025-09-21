#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
echo "🔍 Root: $ROOT"
BE="$ROOT/backend"

REQUIRED_PY=""
if [[ -f "$ROOT/.python-version" ]]; then
  REQUIRED_PY="$(tr -d '\r\n ' < "$ROOT/.python-version")"
else
  REQUIRES_LINE="$(grep -E '^\s*requires-python\s*=' "$BE/pyproject.toml" | head -n1 || true)"
  if [[ -n "$REQUIRES_LINE" ]]; then
    REQUIRED_PY="$(echo "$REQUIRES_LINE" | sed -n 's/.*"\(.*\)".*/\1/p' | tr ',' '\n' | grep -Eo '[0-9]+\.[0-9]+' | head -n1)"
  fi
fi

echo "📌 Required Python: $REQUIRED_PY"

venv_python_path() {
  if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    echo "$BE/.venv/Scripts/python.exe"
  else
    echo "$BE/.venv/bin/python"
  fi
}

ensure_uv_installed() {
  if ! command -v uv >/dev/null 2>&1; then
    echo "❌ uv not found. Install uv first: backend/README.md를 참고 하세요."
    exit 1
  fi
}

create_or_recreate_venv() {
  echo "⚙️  Installing Python $REQUIRED_PY via uv and creating venv"
  (
    cd "$BE"
    uv python install "$REQUIRED_PY"

    rm -rf "$BE/.venv"
    uv venv --python "$REQUIRED_PY" .venv
  )
}

venv_matches_required() {
  local py
  py="$(venv_python_path)"
  [[ -x "$py" ]] || return 1
  # 주/부 버전만 비교(예: 3.12)
  local ver
  ver="$("$py" -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')"
  [[ "$ver" == "$REQUIRED_PY" ]]
}

ensure_uv_installed

if [[ ! -d "$BE/.venv" ]]; then
  create_or_recreate_venv
else
  if venv_matches_required; then
    echo "✅ Existing .venv matches Python $REQUIRED_PY"
  else
    echo "♻️  .venv Python mismatch → recreating"
    create_or_recreate_venv
  fi
fi

if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
  # Windows (Git Bash/MSYS)
  source "$BE/.venv/Scripts/activate"
else
  # Unix/macOS
  source "$BE/.venv/bin/activate"
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