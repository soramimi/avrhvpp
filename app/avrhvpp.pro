TARGET = avrhvpp
TEMPLATE = app
CONFIG += console c++1z
CONFIG -= app_bundle
CONFIG -= qt

DESTDIR = $$PWD/_bin

SOURCES += \
        AvrIsp.cpp \
        AvrUpdi.cpp \
        Connection.cpp \
        main.cpp \
        serial.cpp

HEADERS += \
	../common/commands.h \
	../common/target.h \
	AvrIsp.h \
	AvrUpdi.h \
	Connection.h \
	main.h \
	target.h \
	serial.h

DISTFILES += \
    ../AGENTS.md \
    ../README.md
