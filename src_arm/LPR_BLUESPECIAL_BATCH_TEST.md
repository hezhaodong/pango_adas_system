# BlueSpecial RK3568 U盘批量测试

该程序直接读取已经裁剪为 `94x24` 的车牌图片，不运行 YOLO。它会遍历
`extracted_datasets_split_v4` 下所有包含 `test.txt` 和 `test_images/` 的子集，
使用 `bluespecial.rknn` 做板端 NPU 推理并统计：

- 整牌完全匹配准确率；
- 基于编辑距离的字符准确率；
- 每个 CCPD 子集以及全部子集的独立指标；
- 原 LPR 链路平均、P50、P95 延时和包含U盘读取的整体 FPS；
- 每张图片的标签、预测、耗时和错误信息 CSV。

测试程序直接调用视频链路使用的 `init_lprnet_model()` 和
`inference_lprnet_model()`，因此预处理、RKNN 推理、重复字符处理、车牌规则修复和
时序兜底均与 `DetectionWorker` 一致。每张静态图片使用独立的 `frame_id/track_id`，
不会把上一张图片的时序状态带到下一张。

U盘数据是已经裁剪好的 `94x24` 车牌图，因此测试入口对应原视频链路中的
“YOLO 已经裁出车牌”之后。它评估原链路的 LPR 阶段，不统计 YOLO 检测率。

## 1. 准备文件

把以下内容放到 RK3568 的同一个工作目录：

- `lpr_bluespecial_batch_test.cpp`
- `lpr_bluespecial_batch_test.pro`
- `lprnet.cc`
- `lprnet.h`
- `rknn_api.h`
- `bluespecial.rknn`

确认板端已经安装 `librknnrt.so` 和 OpenCV 4 开发库。

## 2. 在板端编译

为了不覆盖摄像头程序已有的 `Makefile`，给批测程序使用独立文件名：

```sh
cd /path/to/ARM_final1
qmake -o Makefile.lpr_batch lpr_bluespecial_batch_test.pro
make -f Makefile.lpr_batch -j4
```

## 3. 确认U盘挂载点

```sh
lsblk -f
findmnt
```

Windows 中的 `F:\extracted_datasets_split_v4` 在板端会表现为类似下面的路径：

```text
/media/usb/extracted_datasets_split_v4
```

实际路径以 `findmnt` 输出为准。

## 4. 先做100张冒烟测试

```sh
./lpr_bluespecial_batch_test \
  --model ./bluespecial.rknn \
  --dataset /media/usb/extracted_datasets_split_v4 \
  --split ccpd_db \
  --limit 100 \
  --output /media/usb/bluespecial_smoke.csv
```

## 5. 测试全部图片

```sh
./lpr_bluespecial_batch_test \
  --model ./bluespecial.rknn \
  --dataset /media/usb/extracted_datasets_split_v4 \
  --output /media/usb/bluespecial_board_results.csv
```

本数据集当前有 15162 条测试标签，其中实际存在 15158 张图片；4张缺失图片会计入
`missing`，不会进入准确率分母。完整测试结束后生成：

```text
bluespecial_board_results.csv
bluespecial_board_results.summary.txt
```

如果U盘以只读方式挂载，将 `--output` 改到板端可写目录，例如
`/tmp/bluespecial_board_results.csv`。
