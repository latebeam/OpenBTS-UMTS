# OpenBTS provides an open source alternative to legacy telco protocols and 
# traditionally complex, proprietary hardware systems.
#
# Copyright 2008 Free Software Foundation, Inc.
# Copyright 2014 Range Networks, Inc.
#
# This software is distributed under the terms of the GNU Affero General 
# Public License version 3. See the COPYING and NOTICE files in the main 
# directory for licensing information.
#
# This use of this software may be subject to additional restrictions.
# See the LEGAL file in the main directory for details.

include $(top_srcdir)/Makefile.common

ACLOCAL_AMFLAGS = -I config

# Order must be preserved.
# third_party/libfec MUST come before CommonLibs because libcommon.a links
# against the vendored libfec.a.
# Transceiver is linked in-process into OpenBTS-UMTS via libumtstransceiver.la
# from TransceiverPCIeSDR/. RAD1 and UHD transceivers are no longer built.
SUBDIRS = \
	config \
	third_party/libfec \
	ASN \
	CommonLibs \
	Globals \
	CLI \
	SIP \
	GSM \
	UMTS \
	SGSNGGSN \
	SMS \
	TransceiverPCIeSDR \
	TRXManager \
	Control \
	NodeManager \
	apps \
	doc

EXTRA_DIST = \
	AUTHORS \
	ChangeLog \
	COPYING \
	INSTALL \
	LEGAL \
	README

install: all
	$(MAKE) -C ./apps install

FORCE:
