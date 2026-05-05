# Vendored libfec K=9 rate-1/2 portable Viterbi decoder.
#
# Original source: Phil Karn KA9Q, libfec 3.0.1 (LGPLv2.1) — http://www.ka9q.net/code/fec/
# License: LGPLv2.1, see lesser.txt
# Local modifications: see README.UMTS-CHANGES.md
#
# This builds a static convenience library libfec.a that gets statically
# linked into OpenBTS-UMTS. No runtime dependency on a system libfec.

# Note: this directory must NOT include $(top_srcdir)/Makefile.common —
# the AGPL CXXFLAGS/-DTIMESTAMP_ISO from there don't apply to this LGPL'd
# vendored C code, and pulling them in causes spurious -ldl on .o targets.

noinst_LIBRARIES = libfec.a

libfec_a_CFLAGS = -O3 -fPIC -Wno-unused-result

# Only the portable-C K=9 rate-1/2 Viterbi backend is built. Other libfec
# backends (rate-1/3 K=9, K=7, Reed-Solomon, AltiVec, MMX, SSE) are not
# vendored. To enable SIMD later, add viterbi29_sse2.c (etc.) here, drop
# cpu_features_stub.c, and add a real cpuid-based cpu_features().
libfec_a_SOURCES = \
	viterbi29.c \
	viterbi29_port.c \
	cpu_mode_x86.c \
	cpu_features_stub.c \
	fec.c

EXTRA_DIST = \
	fec.h \
	lesser.txt \
	README.UMTS-CHANGES.md

noinst_HEADERS = fec.h
