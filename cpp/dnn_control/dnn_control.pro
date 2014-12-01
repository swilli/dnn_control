TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    simulator.cpp \
    asteroid.cpp \
    sensorsimulator.cpp \
    spacecraftcontroller.cpp \
    utility.cpp \
    odesystem.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    simulator.h \
    asteroid.h \
    sensorsimulator.h \
    spacecraftcontroller.h \
    constants.h \
    utility.h \
    odesystem.h

LIBS += -lgsl -lgslcblas
