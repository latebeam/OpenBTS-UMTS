#!/bin/bash
# vendor-libfec.sh
#
# Populate third_party/libfec/ with the LGPL'd libfec source files needed
# to build the K=9 rate-1/2 portable Viterbi decoder, applying the UMTS
# polynomial patch and the cpu_features stub.
#
# Run this once after cloning the repo, then `git add` the resulting files
# (this script need only be run again when refreshing libfec from upstream).
#
# Usage:
#   ./vendor-libfec.sh /path/to/extracted/fec-3.0.1
#
# To obtain libfec 3.0.1:
#   wget http://www.ka9q.net/code/fec/fec-3.0.1.tar.bz2
#   tar xjf fec-3.0.1.tar.bz2
#   ./vendor-libfec.sh fec-3.0.1/
#
# Files copied:        fec.h viterbi29.c viterbi29_port.c cpu_mode_x86.c lesser.txt
# Files patched:       fec.h (V29POLYA -> 0x11d, V29POLYB -> 0x1af for UMTS)
# Files NOT vendored:  cpu_features.s (32-bit asm, replaced by cpu_features_stub.c)
#                      All other libfec sources (RS, K=7, SIMD backends, etc.)

set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 /path/to/extracted/fec-3.0.1" >&2
    exit 1
fi

SRC="$1"
DST="$(cd "$(dirname "$0")" && pwd)"

# Sanity-check the source dir
for f in fec.h viterbi29.c viterbi29_port.c cpu_mode_x86.c fec.c lesser.txt; do
    if [[ ! -f "$SRC/$f" ]]; then
        echo "error: $SRC/$f not found — is this an extracted libfec 3.0.1 tree?" >&2
        exit 1
    fi
done

echo "==> Copying libfec sources from $SRC to $DST"
cp "$SRC/fec.h"            "$DST/fec.h"
cp "$SRC/viterbi29.c"      "$DST/viterbi29.c"
cp "$SRC/viterbi29_port.c" "$DST/viterbi29_port.c"
cp "$SRC/cpu_mode_x86.c"   "$DST/cpu_mode_x86.c"
cp "$SRC/lesser.txt"       "$DST/lesser.txt"

echo "==> Applying UMTS polynomial patch to fec.h"
# UMTS TS 25.212 §4.2.3.1 rate-1/2 K=9 polys: G0 = 0561 octal, G1 = 0753 octal.
# These match the bit-reversed values used in BitVector.cpp's mCoeffs[].
# Stock libfec 3.0.1 ships with CCSDS polys (0x6d, 0x4f) — different code,
# decodes UMTS data with ~50% BER. Patch unconditionally.
python3 - "$DST/fec.h" <<'PY'
import sys, re, pathlib
p = pathlib.Path(sys.argv[1])
src = p.read_text()
# Match either CCSDS defaults or already-UMTS-patched values, replace both.
src = re.sub(r'(#define\s+V29POLYA\s+)0x[0-9a-fA-F]+',
             r'\g<1>0x11d /* UMTS TS 25.212: G0 = 0561 octal */', src)
src = re.sub(r'(#define\s+V29POLYB\s+)0x[0-9a-fA-F]+',
             r'\g<1>0x1af /* UMTS TS 25.212: G1 = 0753 octal */', src)
p.write_text(src)
print("  patched.")
PY

echo "==> Annotating modified fec.h"
# Insert a one-line "modified" notice at top so anyone reading the file knows.
if ! grep -q "Modified for OpenBTS-UMTS" "$DST/fec.h"; then
    tmpfile="$(mktemp)"
    cat > "$tmpfile" <<'HEADER'
/* Modified for OpenBTS-UMTS: V29POLYA/V29POLYB swapped to UMTS TS 25.212
 * rate-1/2 K=9 polynomials (G0 = 0561 octal = 0x11d, G1 = 0753 octal = 0x1af).
 * See third_party/libfec/README.UMTS-CHANGES.md for details. */

HEADER
    cat "$DST/fec.h" >> "$tmpfile"
    mv "$tmpfile" "$DST/fec.h"
fi

echo "==> Verifying patch landed"
grep -E "^#define\s+V29POLY[AB]" "$DST/fec.h" || {
    echo "error: V29POLY defines not found after patch" >&2
    exit 1
}

echo
echo "Done. Files in $DST:"
ls -la "$DST"
echo
echo "Next:  cd <repo>; autoreconf -i; ./configure; make"
