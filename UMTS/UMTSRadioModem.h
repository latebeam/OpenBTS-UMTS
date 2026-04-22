/*
 * OpenBTS provides an open source alternative to legacy telco protocols and
 * traditionally complex, proprietary hardware systems.
 *
 * Copyright 2011, 2014 Range Networks, Inc.
 *
 * This software is distributed under the terms of the GNU Affero General 
 * Public License version 3. See the COPYING and NOTICE files in the main
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#ifndef UMTSRADIOMODEM_H
#define UMTSRADIOMODEM_H

#include "sigProcLib.h"
//#include <map>
#include "LinkedLists.h"
#include "Sockets.h"
#include "UMTSCodes.h"
#include <Configuration.h>
#include <queue>
#include <numeric>
#include <algorithm>
#include <memory>

#define SLOT_AVG_RTT_MAX_COUNT 72
#define AVG_RTT_MAX_COUNT 144
#define SLOTS_PER_FRAME 15
#include <set>

extern ConfigurationTable gConfig;

static const int MAX_DCH_THREADS = 8;

namespace UMTS {

typedef int16_t radioData_t;

/** a priority queue of radioVectors, i.e. UMTS bursts, sorted so that earliest element is at top */
class TxBitsQueue : public InterthreadPriorityQueue<TxBitsBurst> {

public:

  /** the top element of the queue */
  UMTS::Time nextTime() const;

  /**
    Get stale burst, if any.
    @param targTime The target time.
    @return Pointer to burst older than target time, removed from queue, or NULL.
  */
  TxBitsBurst* getStaleBurst(const UMTS::Time& targTime);

  /**
    Get current burst, if any.
    @param targTime The target time.
    @return Pointer to burst at the target time, removed from queue, or NULL.
  */
  TxBitsBurst* getCurrentBurst(const UMTS::Time& targTime);

};

struct FECDispatchInfo {
	void *fec; // actually DCHFEC;
	RxBitsBurst *burst;
	float tfciFrame[30];
	FECDispatchInfo() { RN_MEMCHKNEW(FECDispatchInfo); }
	~FECDispatchInfo() { RN_MEMCHKDEL(FECDispatchInfo); }
};

struct RACHProcessorInfo {
        void *burst; // actually DCHFEC;
        UMTS::Time burstTime;
        RACHProcessorInfo() { RN_MEMCHKNEW(RACHProcessorInfo); }
        ~RACHProcessorInfo() { RN_MEMCHKDEL(RACHProcessorInfo); }
};

struct DCHProcessorInfo {
        std::shared_ptr<const signalVector> burst;
        UMTS::Time burstTime;
	void *fec;
        DCHProcessorInfo() { RN_MEMCHKNEW(DCHProcessorInfo); }
        ~DCHProcessorInfo() { RN_MEMCHKDEL(DCHProcessorInfo); }
};

struct DCHLoopInfo {
	void *radioModem;
	int threadId;
};

// Assuming one sample per chip.  

class DPDCH
{
        public:
        void* fec;
        UMTS::Time frameTime;
        signalVector descrambledBurst;
	signalVector rawBurst;
        float tfciBits[32];
        float tpcBits[30];
        bool active;
	float bestTOA;
	complex bestChannel;
	float bestSNR;
        float lastTOA;
        float powerMultiplier;

        unsigned int mRxRadioFrameCounter;
        std::deque<float> dequeRTT;
        std::deque<float> dequeSlotAvgRTT;

        // Bootstrap state — after signal loss (lastTOA=-10000), require
        // N consecutive strong samples to agree on TOA before trusting
        // the new lock.  Prevents "bad lock" on noise peaks.
        float bootstrapTOA;        // candidate TOA being verified
        int   bootstrapCount;      // consecutive agreeing samples

        DPDCH(void* wFEC, UMTS::Time wTime) :
                fec(wFEC),
                frameTime(wTime),
                descrambledBurst(gFrameLen),
                rawBurst(gFrameLen + gSlotLen),
                active(true),
                bestTOA(-10000.0),
                bestChannel(1.0),
                bestSNR(-1000.0),
                lastTOA(-10000.0),
                powerMultiplier(1.0),
                mRxRadioFrameCounter(0),
                bootstrapTOA(-10000.0),
                bootstrapCount(0)
        {
        }

        ~DPDCH()
        {}

        void pushRTT(int slotIdx, float RTT)
        {
                dequeRTT.push_back(RTT);
                if (dequeRTT.size() > AVG_RTT_MAX_COUNT)
                        dequeRTT.pop_front();

                if (slotIdx >= (SLOTS_PER_FRAME - 1)) {
                        onPushLastSlotRTT();
                }
        }

        void onPushLastSlotRTT()
        {
                // Calculate avg of this frame
                std::deque<float>::reverse_iterator itRTT;
                float sumRTT = 0;
                int iSlotCount = 0;
                for (itRTT = dequeRTT.rbegin(); itRTT != dequeRTT.rend(); ++itRTT)
                {
                        sumRTT += *itRTT;
                        if (++iSlotCount >= SLOTS_PER_FRAME)
                                break;
                }
                dequeSlotAvgRTT.push_back(sumRTT / (float)iSlotCount);
                if (dequeSlotAvgRTT.size() > SLOT_AVG_RTT_MAX_COUNT) // bigger than superframe
                        dequeSlotAvgRTT.pop_front();
        }

        static float getAvgRTT(const std::deque<float> &rRTT) {
                float RTT = 0;
                if (!rRTT.empty()) {
                        float sumRTT = std::accumulate(rRTT.begin(), rRTT.end(), 0.0f);
                        RTT = sumRTT / (float)rRTT.size();
                }
                return RTT;
        }

        float getAvgRTT() const {
                return getAvgRTT(dequeRTT);
        }

        float getSlotAvgRTT() const {
                return getAvgRTT(dequeSlotAvgRTT);
        }
};


// TODO: The RadioModem needs a loop to call transmitSlot repeatedly.
class RadioModem

{

public:

/*	class DPDCH
	{
        public:
        void* fec;
        UMTS::Time frameTime;
        signalVector descrambledBurst;
        float tfciBits[32];
	float tpcBits[30];    
        bool active;
	float TOA;
	float powerMultiplier;

        DPDCH(void* wFEC, UMTS::Time wTime): fec(wFEC), frameTime(wTime)
        {active = true; descrambledBurst = signalVector(gFrameLen); TOA = -10000.0;powerMultiplier=1.0;}

        ~DPDCH()
        {}

	};
*/
	std::map<void*,DPDCH*> gActiveDPDCH;
	Mutex gActiveDPDCHLock;	// protects gActiveDPDCH from concurrent DCH processor threads


	signalVector *mPreallocBurst;  // pre-allocated burst for receiveBurst()
	UDPSocket& mDataSocket;


        RadioModem(UDPSocket& wDataSocket);

        /* (pointer to channel map,
                    map of scrambling codes,
                    priority queue of TxBitsBurst objects)
        */
        // gather up submitted slots for transmission at timestamp
        // return underrun to indicate that queue contains bursts that are too old
        void transmitSlot(UMTS::Time timestamp, bool &underrun);

        // public method to add TxBitsBurst burst for transmission
        // return underrun to indicate that burst is too late, and what time the clock should be updated to
        void addBurst (TxBitsBurst *wBurst, bool &underrun, Time &updateTime);

	// receive burst from UDP packet
        void receiveBurst(void);

        /*struct FECDispatchInfo {
                DCHFEC *fec;
                RxBitsBurst *burst;
        };*/

        InterthreadQueueWithWait<FECDispatchInfo> mDispatchQueue;
        InterthreadQueueWithWait<RACHProcessorInfo> mRACHQueue;
        InterthreadQueueWithWait<DCHProcessorInfo> mDCHQueue[MAX_DCH_THREADS];

        friend void *FECDispatchLoopAdapter(RadioModem*);
        friend void *RACHLoopAdapter(RadioModem*);
        friend void *DCHLoopAdapter(DCHLoopInfo*);

        static constexpr float mRACHThreshold = 10.0;

private:

        // Per-signature state tracking
        struct RACHSignatureState {
            int consecutiveCount;
            float consecutiveTOA;
            UMTS::Time lastDetectionTime;

            RACHSignatureState() : consecutiveCount(0), consecutiveTOA(0.0) {}
        };

        std::map<int, RACHSignatureState> mRACHSignatureStates;
        Mutex mRACHStateLock;

/*
        // Queue for multiple pending RACH messages
        struct PendingRACHMessage {
            int signature;
            int subchannel;
            float TOA;
            int controlSpreadingCodeIndex;
            int dataSpreadingCodeIndex;
            UMTS::Time detectionTime;
            UMTS::Time responseTime;

            // Add per-slot TOA tracking
            float slotTOAs[15];
            int slotsDecoded;
            float avgTOA;

            PendingRACHMessage() : slotsDecoded(0), avgTOA(0.0) {
                for (int i = 0; i < 15; i++) slotTOAs[i] = 0.0;
            }
        };
*/
        struct PendingRACHMessage {
            int signature;
            int subchannel;
            float TOA;
            int controlSpreadingCodeIndex;
            int dataSpreadingCodeIndex;
            UMTS::Time detectionTime;
            UMTS::Time responseTime;
            int slotsDecoded;
            float slotTOAs[15];
            float avgTOA;
            signalVector descrambledFrame;  // Per-message descrambling buffer
            float tfciBits[30];             // Per-message TFCI storage

            PendingRACHMessage()
                : signature(0), subchannel(0), TOA(0.0),
                  controlSpreadingCodeIndex(0), dataSpreadingCodeIndex(0),
                  slotsDecoded(0), avgTOA(0.0),
                  descrambledFrame(gFrameLen)  // Initialize buffer
            {
                for (int i = 0; i < 15; i++) slotTOAs[i] = 0.0;
                for (int i = 0; i < 30; i++) tfciBits[i] = 0.0;
            }
        };

        std::deque<PendingRACHMessage> mPendingRACHMessages;
        Mutex mPendingRACHLock;


        // Base configuration values
        int mRACHBaseSignature;

	// receive data
        void receiveSlot (signalVector *wBurst, UMTS::Time wTime);


        // map between a hash and an array of 15 signalVectors of varying length
        // hash function is (scramblingcode*6)+nP
        std::map<int,signalVector**> mUplinkPilotWaveformMap;
        std::map<int,UplinkScramblingCode*> mUplinkScramblingCodes;
        // Protects both maps above. DCHLoopAdapter runs MAX_DCH_THREADS
        // threads concurrently; without this lock, concurrent insert on one
        // thread can corrupt map internals while another thread is reading.
        Mutex mUplinkWaveformLock;

        inline int waveformMapHash(int scramblingCode, int nP) { return scramblingCode*6+nP;}

        signalVector *mRACHTable[16];

        //      ChannelMap   *mMap; // ???
        TxBitsQueue *mTxQueue;

        // Going to assume we are only using one signature
        bool mRACHSignatureMask[16];
        bool mRACHSubchannelMask[12];

        // indices into scrambling tables
        int mUplinkRACHScramblingCodeIndex;
        int mUplinkPRACHScramblingCodeIndex;
        UMTS::Time mAICHRACHOffset;
        int mAICHSpreadingCodeIndex;
        int mRACHSearchSize;
        // RACH correlator length
        int mRACHCorrelatorSize;
	int mRACHPreambleOffset;
	int mRACHPilotsOffset;
	signalVector *mRACHMessagePilotWaveforms[16][gFrameSlots]; // [signature][slot]
	int mRACHMessageControlSpreadingFactorLog2;
	int mRACHMessageDataSpreadingFactorLog2;
	int mRACHMessageControlSpreadingCodeIndex;
	int mRACHMessageDataSpreadingCodeIndex;
	int mRACHMessageSlots;
	bool mRACHMessagePending;
	UMTS::Time mNextRACHMessageStart;
        int8_t mRACHMessageAlignedScramblingCodeI[gFrameLen];
        int8_t mRACHMessageAlignedScramblingCodeQ[gFrameLen];
	double mExpectedRACHTOA;

        int mDownlinkScramblingCodeIndex;
        DownlinkScramblingCode* mDownlinkScramblingCode;
        int8_t mDownlinkAlignedScramblingCodeI[gFrameLen];
        int8_t mDownlinkAlignedScramblingCodeQ[gFrameLen];
        int mDownlinkSpreadingCodeIndex;

        // uplink is behind the downlink for DPCH.
        static const int mDPCHOffset = 1024;

        // latest transmit timestamp
	public:
        UMTS::Time mLastTransmitTime;
		unsigned txqsize() { return mTxQueue->size(); }

	private:

       	int mDelaySpread;
        int mDPCCHCorrelationWindow;
        int mDPCCHSearchSize;

        int mSSCHGroupNum;
	radioData_t *mDownlinkSCHWaveformsI[gFrameSlots];
	radioData_t *mDownlinkSCHWaveformsQ[gFrameSlots];
        radioData_t *mDownlinkPilotWaveformsI;
        radioData_t *mDownlinkPilotWaveformsQ;

	static const radioData_t mCPICHAmplitude = 5;
  	static const radioData_t mPSCHAmplitude = 2;  // usually 3dB below CPICH, but can be signifcantly lower (PSCH not scrambled)
  	static const radioData_t mSSCHAmplitude = 5;  // usually 3dB below CPICH (keep in mind SSCH not scrambled)
  	static const radioData_t mCCPCHAmplitude = 5;
	static const radioData_t mAICHAmplitude = 20; // FIXME: Is this right?
	static const radioData_t mDCHAmplitude = 10;
	Thread mFECDispatcher;
	Thread mRACHProcessor;
	Thread mDCHProcessor[MAX_DCH_THREADS];

	/* Generate a table of pilot sequences for lookup and later correlation 
	   Defined Sec. 5.2.1.1 of 25.211, dependes upon higher layer parameters and the slot */
	signalVector* UplinkPilotWaveforms(int scramblingCode, int codeIndex, int numPilots, int slotIx);

	/* Generate a table of RACH preambles
           Need to know scrambling code assigned to RACH preambles and message part */
	void generateRACHPreambleTable(int startIx, int filtLen);

	void generateRACHMessagePilots(int filtLen);

	/* Generate and combine SCH (P-SCH and S-SCH) and CPICH waveforms for repeated transmission */
	void generateDownlinkPilotWaveforms();

	/* Estimate the channel (phase, amplitude, time offset) */
	float estimateChannel(signalVector *wBurst,
                                 signalVector *matchedFilter,
                                 unsigned maxTOA,
                                 unsigned startTOA,
                                 complex *channel,
                                 float *TOA);

	/* Accumulate a vector into an existing vector */
	void accumulate(radioData_t *addI, radioData_t *addQ, int addLen, radioData_t *accI, radioData_t *accQ);


        /* Scramble a transmit burst...essentially a series of sign changes on array of floats. 
           Sign change can be implemented simply as a bit flip. */
	void scramble(radioData_t *wBurstI, radioData_t *wBurstQ, int len,
                          int8_t *codeI, int8_t *codeQ, int codeLen,
                          radioData_t **rBurstI, radioData_t **rBurstQ);

	/* Used to scramble pre-computed RACH waveforms */
	void scrambleRACH(radioData_t *wBurstI, int len,
                              int8_t *codeI, int codeLen,
                              radioData_t **rBurstI);


	/* Descramble a receive burst...essentially a series of sign changes on array of floats?  Nope, they are complex multiplies.*/
	signalVector* descramble(signalVector &wBurst, int8_t *codeI, int8_t *codeQ, signalVector *retVec = NULL);

	/* Despread a descrambled burst...essentially an integrate(add/subtract) and dump operation on floats. */
	signalVector *despread(signalVector &wBurst,
                               const int8_t *code,
                               int codeLength,
			       bool useQ,
			       signalVector *retVec = NULL);
	
	/* Spread a scrambled burst...essentially an Kronecker product */
	void spread(BitVector &wBurst, int8_t *code, int codeLen, radioData_t *accI, radioData_t *accQ, int accLen, radioData_t gain = 1);

	void spreadOneBranch(BitVector &wBurst, int8_t *code, int codeLen, radioData_t *acc, int accLen);

	public: 

	/* Detect a RACH preamble, and return AICH in following access slot */
	bool detectRACHPreamble(signalVector &wBurst, UMTS::Time wTime, float detectionThreshold);

	/* Decode expected RACH message */
        bool decodeRACHMessage(signalVector &wBurst, UMTS::Time wTime, float detectionThreshold);

        /* Decode expected DCH burst */
        bool decodeDCH(signalVector &wBurst,
                           UMTS::Time wTime,
                           int uplinkScramblingCodeIndex,
                           int numPilots,
                           DPDCH *currDPDCH);

	bool decodeDPDCHFrame(DPDCH &frame,
			      int uplinkScramblingCodeIndex,
                              int uplinkSpreadingFactorLog2,
			      int uplinkSpreadingCodeIndex);
	void radioModemStart();
};	

//void* FECDispatchLoopAdapter(RadioModem* rm);


}

void* FECDispatchLoopAdapter(UMTS::RadioModem* rm);

void* RACHLoopAdapter(UMTS::RadioModem* rm);

void* DCHLoopAdapter(UMTS::DCHLoopInfo* dli);


#endif
