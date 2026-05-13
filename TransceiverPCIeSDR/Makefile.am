if HAVE_PCIESDR
include $(top_srcdir)/Makefile.common

noinst_LTLIBRARIES = libumtstransceiver.la
noinst_HEADERS = \
	RadioInterface.h \
	RadioDevice.h \
	Transceiver.h \
	SampleBuffer.h \
	PCIeSDRDevice.h \
	Resampler.h \
	convolve.h \
	convert.h

AM_CPPFLAGS = $(STD_DEFINES_AND_INCLUDES) -I$(PCIESDR_INCLUDEDIR)/kernel -I$(PCIESDR_INCLUDEDIR)/api
libumtstransceiver_la_CFLAGS = -Wall $(AM_CFLAGS) -std=gnu99 -march=native
libumtstransceiver_la_CPPFLAGS = -Wall $(AM_CPPFLAGS)
libumtstransceiver_la_CXXFLAGS = -Wall $(AM_CXXFLAGS)
libumtstransceiver_la_SOURCES = \
	RadioInterface.cpp \
	Transceiver.cpp \
	PCIeSDRDevice.cpp \
	SampleBuffer.cpp \
	Resampler.cpp \
	convolve.c \
	convert.c
endif
