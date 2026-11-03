
INCLUDEPATH += /usr/avr/include

HEADERS += \
	../../common/target.h \
    ../avrgpio.h \
    ../avrhvpp.h \
    ../avrpara.h \
    ../usb.h \
    ../waitloop.h

SOURCES += \
    ../avrhvpp.cpp \
    ../avrpara.cpp \
    ../usb.c \
    ../main.cpp \
    ../waitloop.cpp
