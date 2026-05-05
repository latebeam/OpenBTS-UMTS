# OpenBTS provides an open source alternative to legacy telco protocols and
# traditionally complex, proprietary hardware systems.
#
# Copyright 2008, 2009 Free Software Foundation, Inc.
# Copyright 2011, 2014 Range Networks, Inc.
#
# This software is distributed under the terms of the GNU Affero General
# Public License version 3. See the COPYING and NOTICE files in the main
# directory for licensing information.
#
# This use of this software may be subject to additional restrictions.
# See the LEGAL file in the main directory for details.

include $(top_srcdir)/Makefile.common

EXTRA_DIST = \
	example.config \
	README.common

noinst_LTLIBRARIES = libcommon.la

libcommon_la_CXXFLAGS = $(AM_CXXFLAGS) -O3 -lsqlite3
libcommon_la_SOURCES = \
	BitVector.cpp \
	TurboCoder.cpp \
	ByteVector.cpp \
	LinkedLists.cpp \
	Sockets.cpp \
	TCPSocket.cpp\
	Threads.cpp \
	Timeval.cpp \
	Logger.cpp \
	URLEncode.cpp \
	Configuration.cpp \
	sqlite3util.cpp \
	Utils.cpp

noinst_PROGRAMS = \
	BitVectorTest \
	InterthreadTest \
	SocketsTest \
	TimevalTest \
	RegexpTest \
	VectorTest \
	ConfigurationTest \
	LogTest \
	URLEncodeTest \
	F16Test \
	ViterbiCompareTest

noinst_HEADERS = \
	BitVector.h \
	TurboCoder.h \
	ViterbiR2O9Fec.h \
	ByteVector.h \
	Interthread.h \
	LinkedLists.h \
	Sockets.h \
	TCPSocket.h \
	Threads.h \
	Timeval.h \
	Regexp.h \
	Vector.h \
	Configuration.h \
	F16.h \
	URLEncode.h \
	Logger.h \
	Utils.h \
	ScalarTypes.h \
	sqlite3util.h

# libcommon references libfec (via SoftVector::decode(ViterbiR2O9Fec&,...)).
# libfec.a must come AFTER libcommon.la on the link line — left-to-right
# resolution. Putting it in _LDADD (after libcommon.la) handles that.
# $(LIBFEC_A) is defined in Makefile.common and points to the vendored
# static library at $(top_builddir)/third_party/libfec/libfec.a.
LIBFEC_LDADD = $(LIBFEC_A)

URLEncodeTest_SOURCES = URLEncodeTest.cpp
URLEncodeTest_LDADD = libcommon.la $(LIBFEC_LDADD)

BitVectorTest_SOURCES = BitVectorTest.cpp
BitVectorTest_LDADD = libcommon.la $(LIBFEC_LDADD)

InterthreadTest_SOURCES = InterthreadTest.cpp
InterthreadTest_LDADD = libcommon.la $(LIBFEC_LDADD)
InterthreadTest_LDFLAGS = -lpthread

SocketsTest_SOURCES = SocketsTest.cpp
SocketsTest_LDADD = libcommon.la $(LIBFEC_LDADD)
SocketsTest_LDFLAGS = -lpthread

TimevalTest_SOURCES = TimevalTest.cpp
TimevalTest_LDADD = libcommon.la $(LIBFEC_LDADD)

VectorTest_SOURCES = VectorTest.cpp
VectorTest_LDADD = libcommon.la $(LIBFEC_LDADD)

RegexpTest_SOURCES = RegexpTest.cpp
RegexpTest_LDADD = libcommon.la $(LIBFEC_LDADD)

ConfigurationTest_SOURCES = ConfigurationTest.cpp
ConfigurationTest_LDADD = libcommon.la $(LIBFEC_LDADD)

LogTest_SOURCES = LogTest.cpp
LogTest_LDADD = libcommon.la $(LIBFEC_LDADD)

F16Test_SOURCES = F16Test.cpp

ViterbiCompareTest_SOURCES = ViterbiCompareTest.cpp
ViterbiCompareTest_LDADD = libcommon.la $(LIBFEC_LDADD)

MOSTLYCLEANFILES += testSource testDestination
