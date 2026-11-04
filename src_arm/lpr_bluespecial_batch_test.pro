TEMPLATE = app
TARGET = lpr_bluespecial_batch_test

CONFIG += console c++17 release
CONFIG -= app_bundle qt debug

SOURCES += \
    lpr_bluespecial_batch_test.cpp \
    lprnet.cc

HEADERS += \
    lprnet.h \
    rknn_api.h

INCLUDEPATH += . /usr/include/opencv4

QMAKE_CXXFLAGS_RELEASE += -O3
LIBS += -lrknnrt -ldl
LIBS += -lopencv_core -lopencv_imgproc -lopencv_imgcodecs

target.path = /opt/camera_capture/bin
INSTALLS += target
