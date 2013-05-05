#-------------------------------------------------
#
# Project created by QtCreator 2013-05-05T15:21:04
#
#-------------------------------------------------

QT       += core gui

TARGET = qtgui
TEMPLATE = app

SOURCES += main.cpp\
    mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

# ICE stuff
LIBS += -lIce -lIceUtil
INCLUDEPATH += ../common
HEADERS += ../common/gnuradio.h
SOURCES += ../common/gnuradio.cpp
QMAKE_CXXFLAGS += -fpermissive

