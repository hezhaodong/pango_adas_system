# ============================================================
#  高速PCIe摄像头采集系统 v2.1 — OBB + YOLOPv2 traffic perception
#  目标平台: RK3568 (ARM Linux) / Qt 5 / RKNN
# ============================================================

QT       += core gui widgets network
CONFIG   += c++17
TARGET   = camera_capture_v2

# Final build: compile out legacy qDebug() diagnostics.  Keep the targeted
# one-shot FPGA HDMI watchdog diagnostics; those use qWarning() and only run
# after a real frame timeout.
DEFINES += QT_NO_DEBUG_OUTPUT ARM_FINAL_FPGA_DIAGNOSTICS

# ---- 源文件 ----
SOURCES += \
    main.cpp \
    mainwindow.cpp \
    pcie_device.cpp \
    frame_capture.cpp \
    image_converter.cpp \
    obb_detector.cpp \
    obb_postprocess.cpp \
    overlay_renderer.cpp \
    eglfs_video_window.cpp \
    detection_worker.cpp \
    video_frame_widget.cpp \
    lprnet.cc \
    plate_preprocess_rknn.cpp \
    yolov8_detector.cpp \
    lane_departure_analyzer.cpp \
    yolov8_worker.cpp \
    helmet_worker.cpp \
    traffic_detector.cpp \
    traffic_worker.cpp \
    wifi_receiver.cpp \
    ip_camera_receiver.cpp \
    wifi_discovery.cpp \
    wifi_control.cpp

HEADERS += \
    mainwindow.h \
    pcie_device.h \
    frame_buffer.h \
    frame_capture.h \
    image_converter.h \
    obb_detector.h \
    obb_postprocess.h \
    overlay_renderer.h \
    eglfs_video_window.h \
    detection_worker.h \
    video_frame_widget.h \
    lprnet.h \
    yolov8_detector.h \
    lane_departure_analyzer.h \
    yolov8_worker.h \
    helmet_worker.h \
    traffic_detector.h \
    traffic_worker.h \
    wifi_receiver.h \
    ip_camera_receiver.h \
    wifi_discovery.h \
    wifi_control.h

# ---- RKNN SDK (RK3568 NPU) ----
# rknn_api.h 已拷入项目目录, librknnrt.so 在 /usr/lib
unix:!macx {
    # Bundled Rockchip RGA userspace library.  RGA copies each completed PCIe
    # DMA frame into cached ARM memory without the ~12-ms uncached CPU read.
    RGA_ROOT = $$PWD/third_party/librga
    INCLUDEPATH += $$RGA_ROOT/include
    LIBS += $$RGA_ROOT/lib/librga.a
    LIBS += -lrknnrt -ldl -ldrm
    LIBS += -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui
    INCLUDEPATH += /usr/include/opencv4 /usr/include/libdrm

    target.path = /opt/camera_capture/bin
    INSTALLS += target
}

# ---- 调试 / 发布 ----
CONFIG(debug, debug|release) {
    DEFINES += DEBUG_MODE
    message("调试模式")
}

CONFIG(release, debug|release) {
    DEFINES += NDEBUG
    message("发布模式")
}

# Keep the deployable traffic model and its provenance with the project.
DISTFILES += \
    YOLOPV2_INTEGRATION.md \
    YOLOV8_YIELD_INTEGRATION.md \
    LANE_DEPARTURE.md \
    models/traffic_signal_crosswalk/traffic_signal_crosswalk_int8.rknn \
    models/traffic_signal_crosswalk/labels.txt \
    models/traffic_signal_crosswalk/model_metadata.json \
    models/traffic_signal_crosswalk/README.md \
    models/detect3d/chen-yolo26n-3d-seed0-rk3568-fp16.rknn \
    models/detect3d/README.md
