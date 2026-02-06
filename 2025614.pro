QT       += core gui printsupport charts widgets concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets serialport

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    dataworker.cpp \
    main.cpp \
    mainwindow.cpp \
    usbworker.cpp

HEADERS += \
    dataworker.h \
    fftw3.h \
    mainwindow.h \
    usbworker.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
# 设置头文件路径
INCLUDEPATH += /home/loongson/workspace/SORT2/libusb-1.0.28/USBEXERSE
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /opt/fftw-loongarch/include
# 设置库路径
LIBS += -L/home/loongson/workspace/SORT2/libusb-1.0.28/USBEXERSE -lusb-1.0
LIBS += -L/opt/fftw-loongarch/lib -lfftw3
