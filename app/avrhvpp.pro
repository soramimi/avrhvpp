TARGET = avrhvpp
TEMPLATE = app
CONFIG += console c++1z
CONFIG -= app_bundle
CONFIG -= qt

DESTDIR = $$PWD/_bin

SOURCES += \
        main.cpp \
        serial.cpp

HEADERS += \
	../common/commands.h \
	../common/target.h \
	serial.h

DISTFILES += \
    ../AGENTS.md \
    ../README.md
