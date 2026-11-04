#!/bin/sh
# build_on_board.sh — 在 RK3568 板上把 ADAS 增量功能编进 camera_capture_v2
# 用法（在板子的 /userdata/high_speed_pcie 里执行）：
#     sh /userdata/high_speed_pcie/build_on_board.sh
# 说明：
#   * 只做"打补丁 + qmake + make"，不改学长源文件里的原有逻辑
#   * 补丁器幂等，重复执行不会重复插入；首次会生成 *.adas_bak 备份
#   * 想回滚：for f in *.adas_bak; do mv "$f" "${f%.adas_bak}"; done
set -e

PROJ_DIR="${1:-/userdata/high_speed_pcie}"
ADD_DIR="${2:-/userdata/high_speed_pcie}"

cd "$PROJ_DIR"
echo "== 工程目录: $(pwd)"

need="adas_decision.h adas_bridge.h adas_bridge.cpp adas_lane_type.h adas_lane_type.cpp \
adas_glue.h adas_glue.cpp adas_overlay.h adas_overlay.cpp adas_patch.py"
for f in $need; do
    if [ ! -f "$ADD_DIR/$f" ] && [ ! -f "./$f" ]; then
        echo "缺少文件: $f（请先把 08-ADAS新增源码 里除 test/ 外的文件拷到 $ADD_DIR）" >&2
        exit 1
    fi
    if [ -f "$ADD_DIR/$f" ] && [ ! -f "./$f" ]; then
        cp "$ADD_DIR/$f" "./$f"
        echo "  拷入 $f"
    fi
done

echo "== 打补丁"
python3 ./adas_patch.py .

echo "== qmake"
qmake camera_display.pro -o Makefile

echo "== make -j4（首次约 2~4 分钟，增量几秒）"
make -j4

if [ -x ./camera_capture_v2 ]; then
    echo "== 编译成功: $(ls -l ./camera_capture_v2 | awk '{print $5}') 字节"
    echo "   启动: cd $(pwd) && sh ./run_camera.sh"
else
    echo "!! 未生成 camera_capture_v2，请看上面的编译错误" >&2
    exit 1
fi
