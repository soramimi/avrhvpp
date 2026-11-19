
INCLUDEPATH += /usr/avr/include

HEADERS += \
	../../common/commands.h \
    ../avrgpio.h \
    ../avrhvpp.h \
    ../avrpara.h \
    ../avrupdi.h \
    ../usb.h \
    ../uart.h \
    ../waitloop.h

SOURCES += \
    ../avrhvpp.cpp \
    ../avrpara.cpp \
    ../avrupdi.cpp \
    ../usb.c \
    ../main.cpp \
    ../uart.cpp \
    ../waitloop.cpp

DISTFILES += \
    ../Makefile
    