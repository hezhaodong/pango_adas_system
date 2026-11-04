#!/bin/sh
# 一键板端脚本（不 rmmod！学长文档明确警告 rmmod 会触发驱动 bug 导致内核 Oops）
# 关键：WiFi 控制端会把 FPGA 输入源切到 FIBER（无信号 => 卡/花），且网卡名不是 wlan0，
#       所以必须按 wl* 通配关掉所有无线网卡；另外关掉 PCIe ASPM L1（会给 DMA 加几百微秒延迟）。
dmesg -n 1
for i in $(ls /sys/class/net 2>/dev/null | grep -E '^wl'); do
    ifconfig "$i" down 2>/dev/null
done
if [ -w /sys/module/pcie_aspm/parameters/policy ]; then
    echo performance > /sys/module/pcie_aspm/parameters/policy 2>/dev/null
fi
pkill -f camera_capture 2>/dev/null
sleep 2
echo 3c0800000.pcie > /sys/bus/platform/drivers/rk-pcie/unbind 2>/dev/null
sleep 3
echo 3c0800000.pcie > /sys/bus/platform/drivers/rk-pcie/bind
sleep 6
setpci -s 0002:21:00.0 4.w=0x0006
if [ ! -e /dev/pango_pci_driver ]; then
    insmod /userdata/high_speed_pcie/pango_pci_driver.ko
    sleep 2
fi
: > /tmp/run.log
ls -l /dev/pango_pci_driver >> /tmp/run.log 2>&1
grep Cma /proc/meminfo >> /tmp/run.log
gcc -O2 -o /tmp/reg_read /userdata/hs_tools/regread.c 2>/dev/null
/tmp/reg_read 0xf0200000 200 > /tmp/rg.txt 2>&1
grep -E "^0x(110|2A0) " /tmp/rg.txt >> /tmp/run.log
cd /userdata/high_speed_pcie || exit 1
setsid sh ./run_camera.sh < /dev/null > /tmp/rc.log 2>&1 &
sleep 22
sh /userdata/hs_tools/shot.sh >> /tmp/run.log 2>&1
grep -hE "capture-rate|disp-prof" /tmp/rc.log | tail -2 >> /tmp/run.log
cat /tmp/run.log
