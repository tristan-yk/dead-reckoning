#!/bin/sh
# Build, then flash over ST-LINK and reset into the new image.
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
. "$SCRIPT_DIR/env.sh"

"$SCRIPT_DIR/build.sh"

if ! "$PROGRAMMER" -l | grep -q "ST-LINK SN"; then
  echo "No ST-LINK detected. Plug the Nucleo board in over USB and retry." >&2
  exit 1
fi

"$PROGRAMMER" -c port=SWD mode=UR -w "$ELF" -v -rst
