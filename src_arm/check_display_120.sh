#!/bin/sh
set -eu

TARGET_MODE="1920x1080"
TARGET_HZ="120"

connected=""
for status_file in /sys/class/drm/card*-HDMI-A-*/status; do
    [ -r "$status_file" ] || continue
    if [ "$(cat "$status_file")" = "connected" ]; then
        connected=${status_file%/status}
        break
    fi
done

if [ -z "$connected" ]; then
    echo "FAIL: ARM HDMI output is not connected." >&2
    exit 1
fi

echo "HDMI connected: ${connected##*/}"

if ! command -v modetest >/dev/null 2>&1; then
    echo "FAIL: modetest is missing; install/use libdrm-tests first." >&2
    exit 1
fi

drm_modes=$(modetest -M rockchip -c 2>/dev/null || true)
if [ -z "$drm_modes" ]; then
    echo "FAIL: Rockchip DRM connector modes could not be read." >&2
    exit 1
fi

echo "High-refresh modes exposed by monitor EDID/DRM:"
printf '%s\n' "$drm_modes" \
    | grep -E '#[0-9]+[[:space:]]+[0-9]+x[0-9]+[[:space:]]+(120|144|165)([.]0+)?' \
    || echo "  (none)"

if printf '%s\n' "$drm_modes" \
    | grep -Eq "#[0-9]+[[:space:]]+${TARGET_MODE}[[:space:]]+${TARGET_HZ}([.]0+)?"; then
    echo "PASS: ARM HDMI output monitor supports ${TARGET_MODE}@${TARGET_HZ}."
    echo "After launching, confirm: [display-gl] ... qt_reported_hz=${TARGET_HZ} drm_hz=${TARGET_HZ}"
    echo "Also confirm: [display-pacer] frameSwapped_vsync_pump=enabled"
    exit 0
fi

echo "FAIL: connected monitor/DRM does not expose ${TARGET_MODE}@${TARGET_HZ}." >&2
exit 2
