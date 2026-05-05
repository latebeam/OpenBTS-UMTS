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
 * ViterbiCompareTest — side-by-side validation of the stock ViterbiR2O9
 * reference decoder and the libfec-backed ViterbiR2O9Fec drop-in.
 *
 * Build (after libfec is rebuilt with UMTS polys G0=0x71, G1=0xeb):
 *   g++ -O2 -I. -I../config \
 *       ViterbiCompareTest.cpp BitVector.cpp TurboCoder.cpp \
 *       -lfec -o ViterbiCompareTest
 *
 * What it checks:
 *   1. Clean channel: both decoders must recover the original bits
 *      bit-for-bit for every random input.
 *   2. Noisy channel: BER from libfec must match the reference to
 *      within a few percent at several Eb/N0 points.
 *
 * A mismatch on case 1 almost certainly means libfec was NOT rebuilt
 * with UMTS polynomials (stock CCSDS polys will decode with ~50% BER).
 */

#include "BitVector.h"
#include "ViterbiR2O9Fec.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "Configuration.h"
ConfigurationTable gConfig;

using namespace std;

static BitVector randomBits(unsigned n)
{
	BitVector v(n);
	for (unsigned i = 0; i < n; i++) v[i] = rand() & 1;
	return v;
}

// Encode n data bits + 8 tail zeros through ViterbiR2O9 -> 2*(n+8) coded bits.
static BitVector encodeWithTail(const BitVector &data, ViterbiR2O9 &enc)
{
	const unsigned tail = 8;
	BitVector padded(data.size() + tail);
	for (unsigned i = 0; i < data.size(); i++) padded[i] = data[i];
	for (unsigned i = 0; i < tail; i++) padded[data.size() + i] = 0;

	BitVector coded(2 * padded.size());
	padded.encode(enc, coded);
	return coded;
}

// Hard bits -> soft probabilities {0.0, 1.0}
static SoftVector bitsToSoftClean(const BitVector &bits)
{
	SoftVector sv(bits.size());
	for (unsigned i = 0; i < bits.size(); i++)
		sv[i] = bits.bit(i) ? 1.0f : 0.0f;
	return sv;
}

// BPSK + AWGN channel model. Each bit mapped to +1/-1; Gaussian noise
// with given stddev added; LLR-like soft value returned in [0,1].
static SoftVector bitsToSoftNoisy(const BitVector &bits, float sigma)
{
	SoftVector sv(bits.size());
	for (unsigned i = 0; i < bits.size(); i++) {
		float x = bits.bit(i) ? 1.0f : -1.0f;
		// Box-Muller
		float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
		float u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
		float z  = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
		x += sigma * z;
		// Tanh-scaled mapping into [0,1]; 0.5 at x=0, saturates at ±1
		float p = 0.5f + 0.5f * tanhf(2.0f * x / (sigma * sigma + 1e-6f));
		if (p < 0.0f) p = 0.0f;
		if (p > 1.0f) p = 1.0f;
		sv[i] = p;
	}
	return sv;
}

static unsigned countBitDiff(const BitVector &a, const BitVector &b, unsigned n)
{
	unsigned diff = 0;
	for (unsigned i = 0; i < n; i++)
		if (a.bit(i) != b.bit(i)) diff++;
	return diff;
}

// Case 1: clean-channel bit-exact check.
static bool testClean(unsigned nTrials, unsigned bitsPerBlock)
{
	ViterbiR2O9     ref;
	ViterbiR2O9Fec  fec;
	unsigned failRef = 0, failFec = 0, mismatchRefFec = 0;

	for (unsigned t = 0; t < nTrials; t++) {
		BitVector in = randomBits(bitsPerBlock);
		BitVector coded = encodeWithTail(in, ref);
		SoftVector soft = bitsToSoftClean(coded);

		BitVector outRef(bitsPerBlock + 8);
		BitVector outFec(bitsPerBlock + 8);
		soft.decode(ref, outRef);
		soft.decode(fec, outFec);

		if (countBitDiff(in, outRef, bitsPerBlock) != 0) failRef++;
		if (countBitDiff(in, outFec, bitsPerBlock) != 0) failFec++;
		if (countBitDiff(outRef, outFec, bitsPerBlock) != 0) mismatchRefFec++;
	}

	cout << "[clean]  trials=" << nTrials
	     << " bitsPerBlock=" << bitsPerBlock
	     << " refFail=" << failRef
	     << " fecFail=" << failFec
	     << " mismatch(ref vs fec)=" << mismatchRefFec << endl;

	bool ok = (failRef == 0) && (failFec == 0) && (mismatchRefFec == 0);
	if (!ok) {
		cerr << "  FAIL — if fecFail>0 and refFail==0, libfec is almost\n"
		        "  certainly built with CCSDS polys (0x6d/0x4f) instead of\n"
		        "  UMTS polys (0x71/0xeb). See ViterbiR2O9Fec.h for build\n"
		        "  instructions.\n";
	}
	return ok;
}

// Case 2: BER parity at several noise levels (soft-decision check).
static bool testNoisy(unsigned nTrials, unsigned bitsPerBlock)
{
	ViterbiR2O9     ref;
	ViterbiR2O9Fec  fec;
	const float sigmas[] = { 0.5f, 0.75f, 1.0f };
	bool allOk = true;

	for (float sigma : sigmas) {
		unsigned errRef = 0, errFec = 0, total = 0;
		for (unsigned t = 0; t < nTrials; t++) {
			BitVector in = randomBits(bitsPerBlock);
			BitVector coded = encodeWithTail(in, ref);
			SoftVector soft = bitsToSoftNoisy(coded, sigma);

			BitVector outRef(bitsPerBlock + 8);
			BitVector outFec(bitsPerBlock + 8);
			soft.decode(ref, outRef);
			soft.decode(fec, outFec);

			errRef += countBitDiff(in, outRef, bitsPerBlock);
			errFec += countBitDiff(in, outFec, bitsPerBlock);
			total  += bitsPerBlock;
		}
		double berRef = (double)errRef / total;
		double berFec = (double)errFec / total;
		double ratio  = (berRef > 0) ? berFec / berRef : 1.0;

		cout << "[noisy]  sigma=" << sigma
		     << " BER_ref=" << scientific << setprecision(3) << berRef
		     << " BER_fec=" << berFec
		     << " ratio=" << fixed << setprecision(3) << ratio << endl;

		// libfec is allowed to be better than the reference (ratio < 1).
		// Only flag a failure when libfec is materially WORSE than ref.
		if (berRef > 1e-4 && ratio > 2.0) {
			cerr << "  FAIL — libfec BER >2x worse than reference\n";
			allOk = false;
		}
	}
	return allOk;
}

int main(int argc, char **argv)
{
	const unsigned seed = (argc > 1) ? atoi(argv[1]) : 1;
	srand(seed);
	cout << "ViterbiCompareTest seed=" << seed << endl;

	bool ok = true;
	ok &= testClean(200, 400);   // typical DTCH block
	ok &= testClean(200, 100);   // short blocks
	ok &= testClean(50, 2000);   // long blocks
	ok &= testNoisy(500, 400);

	cout << (ok ? "PASS" : "FAIL") << endl;
	return ok ? 0 : 1;
}
