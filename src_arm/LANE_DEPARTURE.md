# YOLOPv2 前视车道偏离检测

该功能直接使用 YOLOPv2 的 `laneMask`，不加载额外模型。算法仅适用于安装位置固定、视角朝前的车载摄像头。

处理流程：在画面下半部稀疏扫描车道掩码，逐行选择摄像头中心两侧最近的标线，使用带离群点剔除的直线拟合左右边界，经时序平滑后计算本车道中心。摄像头中心相对车道宽度的偏移连续超过阈值时触发告警。

## 板端编译

新增了 `lane_departure_analyzer.cpp/.h`，因此更新代码后必须重新运行 qmake 并编译：

```sh
cd /mnt/usb/ARM_final1.7
qmake camera_display.pro
make -j4
```

## 无需重新编译的标定参数

默认启用，默认摄像头中心为画面宽度的 50%，偏移阈值为车道宽度的 15%，连续 3 个 YOLOPv2 结果确认报警。

```sh
# 示例：摄像头光轴位于画面宽度 52% 处
export HS_CAMERA_CENTER_RATIO=0.52

# 示例：偏移达到车道宽度 18% 才报警
export HS_CAMERA_LANE_DEPARTURE_THRESHOLD=0.18

# 示例：连续 4 次偏移才报警
export HS_CAMERA_LANE_CONFIRM_FRAMES=4

./camera_capture_v2
```

临时关闭车道偏离算法：

```sh
export HS_CAMERA_DISABLE_LANE_DEPARTURE=1
./camera_capture_v2
```

参数会在打开 YOLOPv2 功能、首次加载模型时读取。修改参数后应重启程序。

## 前方道路方向

道路方向不再使用逆透视变换，也不需要配置梯形比例。程序在原始 `laneMask`
中生成本车道中心点序列，以 RANSAC 剔除跳到相邻车道的中心点，再拟合二次曲线。
程序只在画面高度约 45%--72% 的远处区域计算曲线的一阶、二阶导数和有符号曲率，
并使用远处车道宽度归一化。中心线不再参与判断；远处左右边界曲率同向时才判断
左弯或右弯，两侧都接近零时判断直行。

WiFi 的 `yolo` 识别 JSON 新增 `lane_departure` 对象，其中包含 `valid`、`alert`、`direction`、`offset_ratio`、拟合边界坐标和置信度。
