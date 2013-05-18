#-------------------------------------------------
#
# Project created by QtCreator 2013-05-05T15:21:04
#
#-------------------------------------------------

QT       += core gui network

TARGET = qtgui
TEMPLATE = app

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    plotter.cpp \
    statistics_client.cpp

HEADERS  += \
    mainwindow.h \
    plotter.h \
    statistics_client.h

FORMS    += \
    mainwindow.ui

# ICE stuff
LIBS += -lIce -lIceUtil
INCLUDEPATH += ../common
HEADERS += ../common/gnuradio.h
SOURCES += ../common/gnuradio.cpp
QMAKE_CXXFLAGS += -fpermissive

