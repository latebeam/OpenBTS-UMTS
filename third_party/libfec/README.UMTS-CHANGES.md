# Vendored libfec for OpenBTS-UMTS

## Origin

Phil Karn's libfec, version 3.0.1 (October 2006).
Upstream: <http://www.ka9q.net/code/fec/>
Original copyright: Phil Karn, KA9Q, 2002–2006.
License: **GNU Lesser General Public License v2.1** (see `lesser.txt`).

Only the K=9 rate-1/2 portable-C Viterbi decoder is vendored.
The Reed-Solomon, K=7, K=15, rate-1/3, and SIMD backends are NOT included.

## What this provides

Functions exposed via `fec.h`:
- `create_viterbi29(int len)` — allocate decoder state
- `init_viterbi29(void *vp, int starting_state)` — reset metrics
- `update_viterbi29_blk(void *vp, unsigned char *syms, int nbits)` — ACS over a block
- `chainback_viterbi29(void *vp, unsigned char *data, unsigned int nbits, unsigned int endstate)` — traceback
- `delete_viterbi29(void *vp)` — free state

These are wrapped by `CommonLibs/ViterbiR2O9Fec.h` into a C++ class that
`SoftVector::decode(...)` uses for UMTS rate-1/2 K=9 channel decoding
(replaces the slower heap-based `ViterbiR2O9` in `BitVector.cpp`).

## Local modifications from upstream libfec 3.0.1

| File | Change | Reason |
|---|---|---|
| `fec.h` | `#define V29POLYA 0x11d` (was `0x6d`) | Match UMTS TS 25.212 §4.2.3.1 G0 = 0561 octal |
| `fec.h` | `#define V29POLYB 0x1af` (was `0x4f`) | Match UMTS TS 25.212 §4.2.3.1 G1 = 0753 octal |
| `cpu_features.s` | **removed** | 32-bit x86 asm, won't assemble on x86-64 |
| `cpu_features_stub.c` | **added** | C replacement for the removed asm; returns 0 (no SIMD), forcing the portable C dispatch |

Polynomial values (`0x11d`, `0x1af`) are the bit-reversed form of `0561`/`0753`
octal that matches the convention used in OpenBTS-UMTS's `ViterbiR2O9` encoder
(`CommonLibs/BitVector.cpp` `mCoeffs[]`).

## Validating after a libfec source update

If you ever pull a newer upstream libfec, **re-apply the polynomial changes**
in `fec.h`, then run:

```
cd CommonLibs
./ViterbiCompareTest
```

The harness must report `mismatch=0` on all clean-channel trials.

## Files in this directory

| File | Source | License |
|---|---|---|
| `lesser.txt` | upstream libfec 3.0.1 | LGPLv2.1 (this is the license text) |
| `fec.h` | upstream libfec 3.0.1, with UMTS poly changes | LGPLv2.1 |
| `viterbi29.c` | upstream libfec 3.0.1 | LGPLv2.1 |
| `viterbi29_port.c` | upstream libfec 3.0.1 | LGPLv2.1 |
| `cpu_mode_x86.c` | upstream libfec 3.0.1 | LGPLv2.1 |
| `cpu_features_stub.c` | written for this project | LGPLv2.1 (matches surrounding libfec) |
| `Makefile.am` | written for this project | AGPLv3 (project license) |
| `README.UMTS-CHANGES.md` | this file | AGPLv3 (project license) |

The libfec sources retain their original Karn copyright headers; do not strip them.

## Why static linking

`libfec.a` is statically linked into OpenBTS-UMTS so deployments don't need a
system-installed `libfec.so`. This satisfies LGPLv2.1 §6 because the libfec
source (this directory) is shipped alongside the binary, allowing the user to
relink with a modified libfec.
