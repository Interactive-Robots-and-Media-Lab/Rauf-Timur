QT += core
QT -= gui
CONFIG += c++11

TARGET = kin_tass
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    planeextractor.cpp

INCLUDEPATH += ../libfreenect2/include

unix:!macx: LIBS += -L$$PWD/../libfreenect2/build/lib/ -lfreenect2

INCLUDEPATH += $$PWD/../libfreenect2/build/lib
DEPENDPATH += $$PWD/../libfreenect2/build/lib

unix:!macx: LIBS += -L$$PWD/../libfreenect2/build/lib/ -lfreenect2-openni2

INCLUDEPATH += $$PWD/../libfreenect2/build
DEPENDPATH += $$PWD/../libfreenect2/build

unix:!macx: LIBS += -L$$PWD/../opencv-2.4.13/build/lib/ -lopencv_core -lopencv_stitching

INCLUDEPATH += $$PWD/../opencv-2.4.13/build/lib
DEPENDPATH += $$PWD/../opencv-2.4.13/build/lib

LIBS += -L/$$PWD/../opencv-2.4.13/build/lib/ -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_calib3d -lopencv_contrib -lopencv_features2d -lopencv_flann -lopencv_gpu -lopencv_legacy -lopencv_ml -lopencv_nonfree -lopencv_objdetect -lopencv_ocl -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videostab

unix:!macx: LIBS += -L$$PWD/../../../usr/local/lib/ -ltesseract

INCLUDEPATH += $$PWD/../../../usr/local/include
DEPENDPATH += $$PWD/../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../usr/local/lib/ -llept

INCLUDEPATH += $$PWD/../../../usr/local/include
DEPENDPATH += $$PWD/../../../usr/local/include

HEADERS += \
    planeextractor.hpp \
    kinect2io.hpp
