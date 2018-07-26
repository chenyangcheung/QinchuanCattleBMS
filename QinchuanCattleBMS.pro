#-------------------------------------------------
#
# Project created by QtCreator 2018-07-24T08:29:07
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QinchuanCattleBMS
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    snapshotthread.cpp

HEADERS  += mainwindow.h \
    snapshotthread.h

FORMS    += mainwindow.ui

# You need to set an environment variable called OPENCV_VERSION
# whose value is OPENCV3 or OPENCV2 according to your opencv version
CONFIG += $$(OPENCV_VERSION)

OPENCV3 {
    win32 {
        message("Using win32 configuration")

        # change this variable according to your path of opencv
        OPENCV_PATH = D:/opencv3.0/opencv # Note: update with the correct OpenCV version
        # change this variable according to your version of opencv
        LIBS_PATH = "$$OPENCV_PATH/build/x64/vc14/lib" #project compiled using Visual C++ 2015 64bit compiler

        CONFIG(debug, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                       -lopencv_world310d
           }

        CONFIG(release, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                        -lopencv_world310
           }
    }

}

OPENCV2 {
    win32 {
        message("Using win32 configuration")

        # change this variable according to your path of opencv
        OPENCV_PATH = C:/opencv # Note: update with the correct OpenCV version
        # change this variable according to your version of opencv
        LIBS_PATH = "$$OPENCV_PATH/build/x64/vc14/lib"

        CONFIG(debug, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                        -lopencv_core2413d \
                        -lopencv_imgproc2413d \
                        -lopencv_highgui2413d
           }

        CONFIG(release, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                        -lopencv_core2413 \
                        -lopencv_imgproc2413 \
                        -lopencv_highgui2413
           }
    }
}

INCLUDEPATH += \
    $$OPENCV_PATH/build/include/

message("OpenCV version: $$(OPENCV_VERSION)")
message("OpenCV path: $$OPENCV_PATH")
message("Includes path: $$INCLUDEPATH")
message("Libraries: $$LIBS")

# VLC configuration
win32 {
        message("Using win32 configuration")

        # change this variable according to your path of opencv
        VLC_PATH = C:/VLC-Qt_1.1.0_win64_msvc2015 # Note: update with the correct OpenCV version
        # change this variable according to your version of opencv
        LIBS_PATH = "$$VLC_PATH/lib"

        CONFIG(debug, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                        -lVLCQtCored \
                        -lVLCQtWidgetsd
           }

        CONFIG(release, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                        -lVLCQtCore \
                        -lVLCQtWidgets
           }
}

INCLUDEPATH += \
    $$VLC_PATH/include/

message("VLC path: $$VLC_PATH")
message("Includes path: $$INCLUDEPATH")
message("Libraries: $$LIBS")
