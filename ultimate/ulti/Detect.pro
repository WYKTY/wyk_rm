#-------------------------------------------------
#
# Project created by QtCreator 2022-06-11T11:56:20
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Detect
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        mainwindow.cpp \
    anti_gyro.cpp \
    main.cpp \
    detecte.cpp

HEADERS += \
        mainwindow.h \
    anti_gyro.h \
    define.h \
    detecte.h

FORMS += \
        mainwindow.ui

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv*

INCLUDEPATH += /home/cloud/Desktop/k0/eigen-3.4.0

