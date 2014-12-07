TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += -std=c++11

SOURCES += main.cpp \
    simulator.cpp \
    asteroid.cpp \
    sensorsimulator.cpp \
    spacecraftcontroller.cpp \
    odesystem.cpp \
    utility.cpp \
    test.cpp

include(deployment.pri)
qtcAddDeployment()

HEADERS += \
    utility.h \
    simulator.h \
    asteroid.h \
    sensorsimulator.h \
    spacecraftcontroller.h \
    constants.h \
    odesystem.h \
    vector.h \
    test.h \
    odeint.h

LIBS += -lgsl -lgslcblas

