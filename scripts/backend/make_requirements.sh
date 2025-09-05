#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

bazel run //third-party/backend:requirements.update 

sed -i.bak '/^-e[[:space:]]/d' "$ROOT/third-party/backend/requirements_lock.txt"

echo "✅ Requirements updated."