#!/bin/sh
# Build the firmware. Any extra arguments are passed through to make.
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
. "$SCRIPT_DIR/env.sh"

python "$SCRIPT_DIR/sync_sources.py"

cd "$BUILD_DIR"
make -j8 all "$@"
