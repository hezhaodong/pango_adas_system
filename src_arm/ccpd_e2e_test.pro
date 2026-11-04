TEMPLATE = app
TARGET = ccpd_e2e_test

CONFIG += console c++17 release
CONFIG -= app_bundle qt debug

SOURCES += \
    ccpd_e2e_test.cpp \
    obb_detector.cpp \
    obb_postprocess.cpp \
    lprnet.cc

HEADERS += \
    obb_detector.h \
    obb_postprocess.h \
    lprnet.h \
    rknn_api.h

INCLUDEPATH += . /usr/include/opencv4
QMAKE_CXXFLAGS_RELEASE += -O3
LIBS += -lrknnrt -ldl -pthread
LIBS += -lopencv_core -lopencv_imgproc -lopencv_imgcodecs

target.path = /opt/camera_capture/bin
INSTALLS += target
