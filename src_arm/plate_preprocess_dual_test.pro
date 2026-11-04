TEMPLATE = app
TARGET = plate_preprocess_dual_test

CONFIG += console c++17 release
CONFIG -= app_bundle qt debug

SOURCES += \
    plate_preprocess_dual_test.cpp \
    plate_preprocess_rknn.cpp \
    lprnet.cc

HEADERS += \
    plate_preprocess_rknn.h \
    lprnet.h \
    rknn_api.h

INCLUDEPATH += . /usr/include/opencv4
QMAKE_CXXFLAGS_RELEASE += -O3
LIBS += -lrknnrt -ldl -pthread
LIBS += -lopencv_core -lopencv_imgproc -lopencv_imgcodecs

target.path = /opt/camera_capture/bin
INSTALLS += target
