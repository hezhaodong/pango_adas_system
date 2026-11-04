#!/bin/sh
set -eu

APP_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MODULE_NAME=pango_pci_driver
MODULE_FILE="$APP_DIR/pango_pci_driver.ko"
APP_FILE="$APP_DIR/camera_capture_v2"
RUN_FILE="$APP_DIR/run_camera.sh"
STAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR=${RECOVERY_LOG_DIR:-/mnt/usb}
if [ ! -d "$LOG_DIR" ] || [ ! -w "$LOG_DIR" ]; then
    LOG_DIR=/tmp
fi
LOG_FILE="$LOG_DIR/rk_pcie_system_recovery_${STAMP}.log"
APP_LOG="$LOG_DIR/camera_after_recovery_${STAMP}.log"

camera_pids()
{
    FOUND=""
    for PROC_DIR in /proc/[0-9]*; do
        EXE=$(readlink "$PROC_DIR/exe" 2>/dev/null || true)
        case "${EXE##*/}" in
        camera_capture_v2|"camera_capture_v2 (deleted)")
            FOUND="$FOUND ${PROC_DIR##*/}"
            ;;
        esac
    done
    echo "$FOUND"
}

device_holder_pids()
{
    if ! command -v fuser >/dev/null 2>&1; then
        echo ""
        return
    fi
    # fuser prints the device label/access flags on stderr and the numeric
    # holder PIDs on stdout.  Suppress stderr so this function returns only
    # whitespace-separated PIDs that are safe to pass to kill.
    fuser /dev/pango_pci_driver 2>/dev/null || true
}

log_holder()
{
    HOLDER_PID=$1
    [ -d "/proc/$HOLDER_PID" ] || return
    HOLDER_EXE=$(readlink "/proc/$HOLDER_PID/exe" 2>/dev/null || true)
    HOLDER_CMD=$(tr '\000' ' ' < "/proc/$HOLDER_PID/cmdline" 2>/dev/null || true)
    echo "[rk-system-recovery] device-holder pid=$HOLDER_PID exe=$HOLDER_EXE cmd=$HOLDER_CMD"
}

echo "RK PCIe recovery started; log: $LOG_FILE"
exec >>"$LOG_FILE" 2>&1

echo "[rk-system-recovery] begin=$(date -Iseconds) app_dir=$APP_DIR"
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: run as root"
    exit 1
fi
if [ ! -f "$MODULE_FILE" ]; then
    echo "ERROR: missing $MODULE_FILE"
    exit 1
fi
if [ ! -x "$APP_FILE" ]; then
    echo "ERROR: missing/non-executable $APP_FILE"
    exit 1
fi

uname -a
cat /proc/cmdline
sha256sum "$MODULE_FILE" "$APP_FILE" "$RUN_FILE"
modinfo "$MODULE_FILE" 2>/dev/null | grep -E '^(filename|version|vermagic|depends):' || true
SYSTEM_MODULE=$(modinfo -n "$MODULE_NAME" 2>/dev/null || true)
if [ -n "$SYSTEM_MODULE" ] && [ -f "$SYSTEM_MODULE" ]; then
    sha256sum "$SYSTEM_MODULE"
fi
find /boot -maxdepth 3 -type f -name '*.dtb' -exec sha256sum {} \; 2>/dev/null || true
echo "[before] modules"
lsmod | grep -E "^${MODULE_NAME}[[:space:]]" || true
echo "[before] pci"
lspci -nn -d 0755:0755 || true
lspci -vv -d 0755:0755 || true

# SIGTERM is handled by main.cpp.  It lets PcieDevice::closeDevice() stop FPGA
# DMA before unmapping/freeing the coherent ring, avoiding writes into freed
# memory while the driver is removed.
APP_PIDS=$(camera_pids)
if [ -n "$APP_PIDS" ]; then
    echo "[rk-system-recovery] graceful-stop pids=$APP_PIDS"
    kill -TERM $APP_PIDS
    WAIT_COUNT=0
    while [ -n "$(camera_pids)" ] && [ "$WAIT_COUNT" -lt 100 ]; do
        sleep 0.1
        WAIT_COUNT=$((WAIT_COUNT + 1))
    done
    if [ -n "$(camera_pids)" ]; then
        echo "ERROR: camera_capture_v2 did not release DMA within 10 seconds"
        exit 1
    fi
fi

HOLDER_PIDS=$(device_holder_pids)
if [ -n "$HOLDER_PIDS" ]; then
    echo "[rk-system-recovery] device still in use after graceful app stop"
    for HOLDER_PID in $HOLDER_PIDS; do
        log_holder "$HOLDER_PID"
    done

    echo "[rk-system-recovery] terminating device holders pids=$HOLDER_PIDS"
    kill -TERM $HOLDER_PIDS 2>/dev/null || true
    WAIT_COUNT=0
    while [ -n "$(device_holder_pids)" ] && [ "$WAIT_COUNT" -lt 50 ]; do
        sleep 0.1
        WAIT_COUNT=$((WAIT_COUNT + 1))
    done

    HOLDER_PIDS=$(device_holder_pids)
    if [ -n "$HOLDER_PIDS" ]; then
        echo "[rk-system-recovery] force-killing device holders pids=$HOLDER_PIDS"
        for HOLDER_PID in $HOLDER_PIDS; do
            log_holder "$HOLDER_PID"
        done
        kill -KILL $HOLDER_PIDS 2>/dev/null || true
        sleep 1
    fi

    HOLDER_PIDS=$(device_holder_pids)
    if [ -n "$HOLDER_PIDS" ]; then
        echo "ERROR: /dev/pango_pci_driver remains in use after TERM/KILL pids=$HOLDER_PIDS"
        for HOLDER_PID in $HOLDER_PIDS; do
            log_holder "$HOLDER_PID"
        done
        fuser -v /dev/pango_pci_driver || true
        exit 1
    fi
    echo "[rk-system-recovery] device holders released"
fi

PCI_DEVICE=""
for DEV_PATH in /sys/bus/pci/devices/*; do
    [ -r "$DEV_PATH/vendor" ] || continue
    [ -r "$DEV_PATH/device" ] || continue
    VENDOR=$(cat "$DEV_PATH/vendor")
    DEVICE=$(cat "$DEV_PATH/device")
    if [ "$VENDOR" = "0x0755" ] && [ "$DEVICE" = "0x0755" ]; then
        PCI_DEVICE=$DEV_PATH
        break
    fi
done

if [ -z "$PCI_DEVICE" ]; then
    echo "ERROR: PCI endpoint 0755:0755 is not present"
    exit 1
fi

BDF=$(basename "$PCI_DEVICE")
DRIVER_SYSFS="/sys/bus/pci/drivers/$MODULE_NAME"
if [ ! -d "$DRIVER_SYSFS" ] || [ ! -w "$DRIVER_SYSFS/bind" ]; then
    echo "ERROR: missing PCI driver bind interface $DRIVER_SYSFS"
    exit 1
fi

# Do not unload/reload this vendor module: its current class_create helper
# loses the class pointer, so rmmod leaves /sys/class/pango_pci_driver behind
# and the next insmod reports EEXIST.  Also do not use the endpoint's sysfs
# reset here; on this FPGA it can reset source-selection state and make the
# otherwise healthy HDMI input disappear.  Unbind/bind reruns only the PCI
# remove/probe callbacks while keeping both the module class and FPGA fabric.
if [ -L "$PCI_DEVICE/driver" ]; then
    echo "[rk-system-recovery] pci-driver-unbind $BDF"
    echo "$BDF" > "$DRIVER_SYSFS/unbind"
    sleep 1
fi
echo "[rk-system-recovery] pci-driver-bind $BDF"
echo "$BDF" > "$DRIVER_SYSFS/bind"
sleep 1

WAIT_COUNT=0
while [ ! -e /dev/pango_pci_driver ] && [ "$WAIT_COUNT" -lt 50 ]; do
    sleep 0.1
    WAIT_COUNT=$((WAIT_COUNT + 1))
done
if [ ! -e /dev/pango_pci_driver ]; then
    echo "ERROR: /dev/pango_pci_driver was not recreated"
    dmesg | tail -n 120
    exit 1
fi

echo "[after] modules"
lsmod | grep -E "^${MODULE_NAME}[[:space:]]" || true
echo "[after] pci"
lspci -nn -d 0755:0755 || true
lspci -vv -d 0755:0755 || true
dmesg | tail -n 120

echo "[rk-system-recovery] starting camera log=$APP_LOG"
cd "$APP_DIR"
nohup /bin/sh "$RUN_FILE" >"$APP_LOG" 2>&1 &
APP_PID=$!
sleep 8
if ! kill -0 "$APP_PID" 2>/dev/null; then
    echo "ERROR: camera failed after recovery"
    tail -n 160 "$APP_LOG" || true
    exit 1
fi

echo "[rk-system-recovery] success pid=$APP_PID app_log=$APP_LOG"
echo "[rk-system-recovery] end=$(date -Iseconds) recovery_log=$LOG_FILE"
