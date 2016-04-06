#-------------------------------------------------
#
# Project created by QtCreator 2016-02-12T22:27:51
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = opencv_project
CONFIG   += console
CONFIG   -= app_bundle
CONFIG  += c++11

TEMPLATE = app


SOURCES += main.cpp \
    puzzle.cpp \
    puzzlecutter.cpp

INCLUDEPATH += "D:/Programs/opencv/opencv_bin/install/include"
LIBS += "D:/Programs/opencv/opencv_bin/bin/*.dll"

HEADERS += \
    geometry.h \
    puzzlecutter.h \
    puzzle.h \
    debug.h

