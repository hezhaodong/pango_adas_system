#!/bin/sh
# Backward-compatible 1080p120 entry point.  Keep all launch protection in
# run_camera_1080p120.sh so both historical command names use the same path.
set -eu

APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec "$APP_DIR/run_camera_1080p120.sh" "$@"
