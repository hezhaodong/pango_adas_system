#!/bin/sh
# verify.sh —— 板端一站式：跑全链路 + 打印关键证据（不 rmmod）
# 用法： sh /userdata/hs_tools/verify.sh
set -x 2>/dev/null

[ -x /tmp/reg_read ] || gcc -O2 -o /tmp/reg_read /userdata/hs_tools/regread.c 2>/dev/null

echo "== STEP1: allinone =="
sh /userdata/hs_tools/allinone.sh > /tmp/allinone.out 2>&1
echo "allinone rc=$?"
tail -6 /tmp/allinone.out

echo "== STEP2: registers =="
/tmp/reg_read 0xf0200000 400 | grep -E "^0x(110|140|2A0|2D0|310) "

echo "== STEP3: rates =="
grep -E "capture-rate|display-sched" /tmp/rc.log 2>/dev/null | tail -2

echo "== STEP4: cma =="
grep Cma /proc/meminfo

echo "== STEP5: snap =="
ls -l /tmp/snap.jpg 2>/dev/null
echo "VERIFY_DONE"
