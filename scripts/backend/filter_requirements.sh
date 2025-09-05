#!/usr/bin/env bash
set -euo pipefail

bazel build //third-party:requirements_filtered >/dev/null

: "${BUILD_WORKSPACE_DIRECTORY:?missing}"
SRC="bazel-bin/third-party/requirements.txt"
DST="$BUILD_WORKSPACE_DIRECTORY/third-party/requirements.txt"

cp -f "$SRC" "$DST"
echo "Wrote $DST"
