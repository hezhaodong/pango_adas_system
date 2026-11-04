# CCPD2019 板端端到端测试

测试流程与 `ARM_final1` 原视频识别链路一致：

```text
CCPD原图 -> ObbDetector(YOLO OBB) -> 原旋转框矫正 -> inference_lprnet_model
```

现有视频链路源文件不做修改。测试程序链接原来的 `obb_detector.cpp`、
`obb_postprocess.cpp` 和 `lprnet.cc`；由于旋转框矫正函数原来是
`detection_worker.cpp` 内部静态函数，测试器中保留了一份逐行等价实现。

车牌真值不会从额外标签文件读取，而是在板端从每张 CCPD 图片文件名的第5段
（例如 `0_0_18_33_23_32_32`）解析。文件名中的四角坐标也用于计算 YOLO OBB
与真实车牌的多边形 IoU，默认以 IoU >= 0.5 判为检测成功。

## 固定随机样本

`ccpd2019_random1000_per_subset_seed20260802.txt` 已经在电脑端生成：

- 7个子集；
- 每个子集随机1000张；
- 总计7000张；
- 固定随机种子 `20260802`。

板端只读取这份清单中的7000个相对路径，不遍历整个 `CCPD2019` 目录。清单不含
车牌标签，标签仍由板端程序解析原始文件名。

## 板端编译

```sh
cd /mnt/usb/ARM_final1

g++ -O3 -std=gnu++17 \
  -I. -I/usr/include/opencv4 \
  ccpd_e2e_test.cpp obb_detector.cpp obb_postprocess.cpp lprnet.cc \
  -o ccpd_e2e_test \
  -lrknnrt -ldl -pthread \
  -lopencv_core -lopencv_imgproc -lopencv_imgcodecs
```

也可以使用独立 qmake 文件：

```sh
qmake -o Makefile.ccpd_e2e ccpd_e2e_test.pro
make -f Makefile.ccpd_e2e -j4
```

## 先测试20张

模型默认路径与原视频程序完全相同。

```sh
./ccpd_e2e_test \
  --dataset /mnt/usb/CCPD2019 \
  --manifest ./ccpd2019_random1000_per_subset_seed20260802.txt \
  --output /mnt/usb/ccpd_e2e_smoke.csv \
  --limit 20
```

## 全部7000张

```sh
./ccpd_e2e_test \
  --dataset /mnt/usb/CCPD2019 \
  --manifest ./ccpd2019_random1000_per_subset_seed20260802.txt \
  --output /mnt/usb/ccpd2019_e2e_results.csv
```

默认模型文件：

```text
/userdata/model/yolov8n_obb.rknn
/userdata/model/yolov8_obb_labels_list.txt
/userdata/model/bluespecial.rknn
/userdata/model/lprnet.rknn
/userdata/model/doubleyellow.rknn
```

输出包含：YOLO IoU>=0.5检测率、检测成功后的LPR整牌准确率、全样本端到端整牌
准确率、字符准确率、YOLO/LPR/整链路延时，以及每张图的详细CSV。
