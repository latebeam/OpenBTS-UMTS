/*
 * Copyright 2026 Late Beam
 *
 * This software is distributed under the terms of the GNU General Public
 * License version 3. See the COPYING and NOTICE files in the current
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 *
 *
 * libfec-backed rate-1/2 K=9 Viterbi decoder for UMTS.
 *
 * IMPORTANT: stock libfec uses CCSDS polynomials (G0=555, G1=517 octal),
 * which do NOT match UMTS TS 25.212 §4.2.3.1 (G0=561, G1=753 octal).
 *
 * To use this wrapper, rebuild libfec with UMTS polynomials:
 *   In libfec/viterbi29_port.c (and any other viterbi29_*.c backends in use):
 *     #define V29POLYA 0x71   // 0561 octal, lower 8 bits (MSB implicit)
 *     #define V29POLYB 0xeb   // 0753 octal, lower 8 bits (MSB implicit)
 *   then `make && sudo make install`.
 *
 * Alternative: vendor viterbi29_port.c into this tree with the poly
 * redefines and build with the project (no runtime dependency).
 *
 * Validation: see BitVectorViterbiCompare.cpp for a harness that runs
 * identical input through both ViterbiR2O9 and ViterbiR2O9Fec and
 * checks bit-identical output.
 */

#ifndef VITERBIR2O9FEC_H
#define VITERBIR2O9FEC_H

#include <stdint.h>
#include <stddef.h>

extern "C" {
#include <fec.h>
}

class ViterbiR2O9Fec {
public:
        ViterbiR2O9Fec(unsigned maxBits = 4096)
                : mVp(NULL), mMaxBits(0)
        {
                resize(maxBits);
        }

        ~ViterbiR2O9Fec()
        {
                if (mVp) delete_viterbi29(mVp);
        }

        // Decode nBits output bits from 2*(nBits+tailSyms/2) soft symbols
        // in syms[]. UMTS appends 8 zero tail bits => endstate=0.
        // outPacked is bit-packed MSB-first, ceil(nBits/8) bytes.
        void decode(const uint8_t *syms, unsigned nBits,
                    unsigned tailBits, uint8_t *outPacked)
        {
                const unsigned totalBits = nBits + tailBits;
                if (totalBits > mMaxBits) resize(totalBits);
                init_viterbi29(mVp, 0);
                update_viterbi29_blk(mVp,
                                     const_cast<unsigned char*>(syms),
                                     totalBits);
                chainback_viterbi29(mVp, outPacked, nBits, 0);
        }

        unsigned constraint() const { return 9; }
        unsigned rate_denom() const { return 2; }

private:
        void resize(unsigned maxBits)
        {
                if (mVp) delete_viterbi29(mVp);
                mVp = create_viterbi29(maxBits);
                mMaxBits = maxBits;
        }

        void *mVp;
        unsigned mMaxBits;

        ViterbiR2O9Fec(const ViterbiR2O9Fec&);
        ViterbiR2O9Fec& operator=(const ViterbiR2O9Fec&);
};

#endif // VITERBIR2O9FEC_H
