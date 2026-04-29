/**file Radiomodem, for physical later processing bits <--> chips */

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

#include "UMTSRadioModemSequences.h"
#include "UMTSRadioModem.h"
#include "UMTSConfig.h"

#include "Transceiver.h"
#include "UMeasInterface.h"

//#include "RAD1Device.h"

#include "Logger.h"


using namespace UMTS;

UMTS::Time TxBitsQueue::nextTime() const
{
  UMTS::Time retVal;
  ScopedLock lock(mLock);
  while (mQ.size()==0) mWriteSignal.wait(mLock);
  return mQ.top()->time();
}

TxBitsBurst* TxBitsQueue::getStaleBurst(const UMTS::Time& targTime)
{
  ScopedLock lock(mLock);
  if ((mQ.size()==0)) {
    return NULL;
  }
  if (mQ.top()->time() < targTime) {
    TxBitsBurst* retVal = mQ.top();
    mQ.pop();
    return retVal;
  }
  return NULL;
}


TxBitsBurst* TxBitsQueue::getCurrentBurst(const UMTS::Time& targTime)
{
  ScopedLock lock(mLock);
  if ((mQ.size()==0)) {
    return NULL;
  }
  if (mQ.top()->time() == targTime) {
    TxBitsBurst* retVal = mQ.top();
    mQ.pop();
    return retVal;
  }
  return NULL;
}


// Assuming one sample per chip.

Transceiver *trx;
RadioBurstFIFO *mT;
RadioBurstFIFO *mR;

const int FILTLEN = 17;
float invFilt[FILTLEN] = {0.010688,-0.021274,0.040037,-0.06247,0.10352,-0.15486,0.23984,-0.42166,0.97118,-0.42166,0.23984,-0.15486,0.10352,-0.06247,0.040037,-0.021274,0.010688};
float invFiltRcv[FILTLEN] = {0.017157,-0.028391,0.029818,-0.043222,0.047142,-0.063790,0.075942,-0.136568,0.964300,-0.136568,0.075942,-0.063790,0.047142,-0.043222,0.029818,-0.028391,0.017157};
signalVector *inverseCICFilter;
signalVector *rcvInverseCICFilter;
signalVector *txHistoryVector;
signalVector *rxHistoryVector;

RadioModem::RadioModem(UDPSocket& wDataSocket)
	:mDataSocket(wDataSocket)
{
  sigProcLibSetup(1);
  mUplinkPilotWaveformMap.clear();
  mUplinkScramblingCodes.clear();

inverseCICFilter = new signalVector(FILTLEN);
//RN_MEMLOG(signalVector,inverseCICFilter);
signalVector::iterator itr = inverseCICFilter->begin();
inverseCICFilter->isRealOnly(true);
inverseCICFilter->setSymmetry(ABSSYM);
for (int i = 0; i < FILTLEN; i++)
  *itr++ = complex(invFilt[i],0.0);
txHistoryVector = new signalVector(FILTLEN-1);

rcvInverseCICFilter = new signalVector(FILTLEN);
//RN_MEMLOG(signalVector,inverseCICFilter);
itr = rcvInverseCICFilter->begin();
rcvInverseCICFilter->isRealOnly(true);
rcvInverseCICFilter->setSymmetry(ABSSYM);
for (int i = 0; i < FILTLEN; i++)
  *itr++ = complex(invFiltRcv[i],0.0);
rxHistoryVector = new signalVector(FILTLEN-1);


  mTxQueue = new TxBitsQueue;
  mPreallocBurst = NULL;

  for (int i = 0; i < 16; i++) {
	mRACHSignatureMask[i] = false;
        if (i < 12) mRACHSubchannelMask[i] = false;
  }

  mDelaySpread = gConfig.getNum("UMTS.Radio.MaxExpectedDelaySpread");

  mDownlinkScramblingCodeIndex = 16*gConfig.getNum("UMTS.Downlink.ScramblingCode");
  LOG(INFO) << "DownlinkScramblingCodeIndex: " << mDownlinkScramblingCodeIndex;
  mDownlinkScramblingCode = new DownlinkScramblingCode(mDownlinkScramblingCodeIndex);

  mUplinkPRACHScramblingCodeIndex = mDownlinkScramblingCodeIndex + gConfig.getNum("UMTS.PRACH.ScramblingCode"); //4.3.3.2 of 25.213

  // Number of RACH signatures and subchannels from config
  int numSigs = gConfig.getNum("UMTS.PRACH.Signature");
  if (numSigs < 1) numSigs = 1;
  if (numSigs > 16) numSigs = 16;
  for (int sig = 0; sig < numSigs; sig++) {
      mRACHSignatureMask[sig] = true;
  }

  int numSubChan = gConfig.getNum("UMTS.PRACH.Subchannel");
  if (numSubChan < 1) numSubChan = 1;
  if (numSubChan > 12) numSubChan = 12;
  for (int sub = 0; sub < numSubChan; sub++) {
      mRACHSubchannelMask[sub] = true;
  }

  mAICHRACHOffset = UMTS::Time(0,cAICHRACHOffset); //FIXME:  make sure this is in the config and SIB5.
  mAICHSpreadingCodeIndex = cAICHSpreadingCodeIndex; // FIXME: needs to be in config and mirror what's in SIB5
  mRACHSearchSize = 100; //gConfig.getNum("UMTS.Radio.MaxExpectedDelaySpread");
  mRACHCorrelatorSize = 256*4;
  mRACHPreambleOffset = 256;
  mRACHPilotsOffset = 256;
  unsigned RACHsf = gConfig.getNum("UMTS.PRACH.SF");
  mRACHMessageControlSpreadingFactorLog2 = 8; // only possible spreading factor
  mRACHMessageDataSpreadingFactorLog2 = (int) round(log2(RACHsf));

  // Initialize RACH pilot waveforms array
  for (int sig = 0; sig < 16; sig++) {
      for (int slot = 0; slot < gFrameSlots; slot++) {
          mRACHMessagePilotWaveforms[sig][slot] = NULL;
      }
  }

  mRACHMessageSlots = 30/2; // FIXME:: needs this from config or higher layers

  mDPCCHCorrelationWindow = 0;
  mDPCCHSearchSize = 256; //2*256/8;
  mSSCHGroupNum = mDownlinkScramblingCodeIndex/128; // 3GPP 25.213 Sec. 5.2.2

  generateDownlinkPilotWaveforms();

  mLastTransmitTime = UMTS::Time(0,0);

  // 3GPP 25.213, Sec. 5.1.4
  // This looks like it doesn't hold true, that the alignment does not need to be shifted
  for (unsigned i = 0; i < gFrameLen; i++) {
    unsigned alignedIx = (i+gFrameLen-0*256) % gFrameLen;
    mDownlinkAlignedScramblingCodeI[i] = *(mDownlinkScramblingCode->ICode()+alignedIx);
    mDownlinkAlignedScramblingCodeQ[i] = *(mDownlinkScramblingCode->QCode()+alignedIx);
  }

  generateRACHPreambleTable(mRACHPreambleOffset,mRACHCorrelatorSize);
  generateRACHMessagePilots(mRACHCorrelatorSize);
}

// pat 1-5-2013: We cant start up a transceiver in the constructor above because other constructors
// at the same level (specifically, the socket that trx is going to use) are not initialized.
// See comments in ARFCNManager.
void* RACHLoopAdapter(RadioModem *modem);
void* FECDispatchLoopAdapter(RadioModem *modem);

void RadioModem::radioModemStart()
{


  mDispatchQueue.clear();
  mFECDispatcher.start((void*(*)(void*)) FECDispatchLoopAdapter, this);
  mRACHQueue.clear();
  mRACHProcessor.start((void*(*)(void*)) RACHLoopAdapter, this);
  for (int i = 0; i < MAX_DCH_THREADS; i++) {
    mDCHQueue[i].clear();
    DCHLoopInfo *dli = new DCHLoopInfo; 
    dli->radioModem = (void *) this;
    dli->threadId = i;
    mDCHProcessor[i].start((void*(*)(void*)) DCHLoopAdapter, dli);
  }
}

void* FECDispatchLoopAdapter(RadioModem *modem)
{
	while(1) {
		FECDispatchInfo *q = (FECDispatchInfo*) (modem->mDispatchQueue).read();
		//printf("q: %0x %0x %0x\n",q,q->fec,q->burst); fflush(stdout);
		if (q!=NULL) { 
		  DCHFEC* fec = (DCHFEC*) (q->fec);
		  if (fec->active()) 
#define FRAMEBURSTS
#ifdef FRAMEBURSTS
			fec->l1WriteLowSideFrame(*(q->burst),(q->tfciFrame));
#else
			fec->l1WriteLowSide(*(q->burst));
#endif
		  // FIXME!!!  (pat) I am removing this delete[] because I dont think
		  // it is correct: the SoftVector at burst deletes its own memory.
		  // delete[] q->burst->begin();
		  // (harvind) the RxBitsBurst destructor does not destroy the inherited SoftVector's data.
		  // Instead of explicitly deleting it, we'll use the clear() command which safely deletes it.
		  //  dynamic_cast<SoftVector*>(q->burst)->clear();
		  // Nope. That doesn't work, nor do calls to clear and resize.  WTF?  Back to the old solution.
		  delete[] q->burst->begin();
		  delete q->burst;
		  delete q;
		}
	}
	return NULL;
}

void* RACHLoopAdapter(RadioModem *modem)
{
        while(1) {
                RACHProcessorInfo *q = (RACHProcessorInfo*) (modem->mRACHQueue).read();

  		// if RACH message part is expected, the decode one of the 15 or 30 consecutive slots./
  		modem->decodeRACHMessage(*((signalVector*)q->burst), q->burstTime, 5.0);

		delete (signalVector *) (q->burst);
		delete q;
        }
        return NULL;
}

void* DCHLoopAdapter(DCHLoopInfo *dli)
{
  UMTS::RadioModem *modem = (UMTS::RadioModem *)(dli->radioModem);
  int threadId = dli->threadId;
  while(1)
  {
    DCHProcessorInfo *q = (DCHProcessorInfo*) (modem->mDCHQueue[threadId]).read();
    UMTS::Time wTime = q->burstTime;
    DCHFEC *currDCH = (DCHFEC*) q->fec;

    // Check if the DCH is still active — it may have been closed while
    // this burst was queued (race condition with channel release).
    {
      ScopedLock lock(gActiveDCH.mLock);
      bool found = false;
      for (DCHListType::const_iterator it = gActiveDCH.begin(); it != gActiveDCH.end(); ++it) {
        if (*it == currDCH) { found = true; break; }
      }
      if (!found) {
        delete q;
        continue;
      }
    }

    // Thread-local mutable copy — decodeDCH modifies the burst.
    signalVector burstLocal(*q->burst);
    delete q;

    int slotIx = wTime.TN();
    DPDCH *currDPDCH = NULL;
    {
      ScopedLock lock(modem->gActiveDPDCHLock);
      if ((slotIx == 0) && (modem->gActiveDPDCH.find((void *)currDCH) == modem->gActiveDPDCH.end()))
      {
        modem->gActiveDPDCH[(void *)currDCH] = new DPDCH((void*)currDCH,wTime);
      }
      std::map<void*,DPDCH*>::iterator it = modem->gActiveDPDCH.find((void *)currDCH);
      if (it != modem->gActiveDPDCH.end())
        currDPDCH = it->second;
    }
    if (!currDPDCH)
    {
      continue;
    }
    if (slotIx == 0)
    {
      currDPDCH->frameTime = wTime;
      currDPDCH->active = true;
      currDPDCH->bestSNR = -1000.0;
    }
    if (!currDPDCH->active)
    {
      continue;
    }
    int uplinkScramblingCodeIndex = currDCH->getPhCh()->SrCode();
    int numPilots = currDCH->getPhCh()->getUlDPCCH()->mNPilot;
    currDPDCH->active = modem->decodeDCH(burstLocal,
                                         wTime,
                                         uplinkScramblingCodeIndex,
                                         numPilots,
                                         currDPDCH);

    // Update UL SNR for TPC power control, and refresh the activity
    // timestamp so the inactivity check in receiveSlot() knows the
    // UE is actually transmitting on this channel.
    if (currDPDCH->bestSNR > -100.0f) {
      currDCH->mLastUlSNR = currDPDCH->bestSNR;
      if (currDPDCH->bestSNR > 3.0f)
        currDCH->mLastActivityFN = wTime.FN();
    }

    if (slotIx == gFrameSlots - 1) // got a frame, let's decode it
    {
      // First, need to figure out TFCI
      int TFCI = findTfci(currDPDCH->tfciBits,currDCH->l1ul()->mNumTfc);

      // (pat) The uplink spreading factor can depend on the TFC of this particular uplink vector.
      // We need to decode the DPCCH first then look up the SF based on the TFCI bits.  Someday.
      int uplinkSpreadingFactorLog2 = currDCH->l1ul()->getFPI(0, TFCI)->mSFLog2;
      int uplinkSpreadingCodeIndex = (1 << uplinkSpreadingFactorLog2) / 4;
      // 4.3.1.2.1 of 25.213
      // DPDCH is always index of SF/4
      // N DPDCH is always a SF of 4, index is 1 if N < 2, 3 if N < 4, 2 if N < 6
      // gonna assume single DPDCH per DCH
#if 0
      LOG(NOTICE) << "numTFCI: " << currDCH->l1ul()->mNumTfc << " TFCI: " << TFCI << ", SF: " << (1 << uplinkSpreadingFactorLog2) << ", scram: " << uplinkScramblingCodeIndex << ", code: " << uplinkSpreadingCodeIndex << ", time:" << wTime;
#endif
      //LOG(INFO) << "TPC: " << currDPDCH->tpcBits[0] << " " << currDPDCH->tpcBits[1];

      if (TFCI != 0)
      {
        currDCH->l1ul()->mReceived = true;
        modem->decodeDPDCHFrame(*currDPDCH,
                                uplinkScramblingCodeIndex,
                                uplinkSpreadingFactorLog2,
                                uplinkSpreadingCodeIndex);
      }
    }
  }

  return NULL;
}

void RadioModem::generateRACHMessagePilots(int filtLen)
{
    int pilotSeqLen = 8*256;
    radioData_t zeroIBurst[gSlotLen];
    memset(zeroIBurst,0,gSlotLen*sizeof(radioData_t));

    // Generate pilot waveforms for each enabled signature
    for (int signature = 0; signature < 16; signature++) {
        if (!mRACHSignatureMask[signature]) continue;

        // Calculate spreading code for THIS signature (per 25.213 Sec 4.3.1.3)
        int controlSpreadingCodeIndex = 16*signature + 15;

        for (unsigned slot = 0; slot < gFrameSlots; slot++) {
            radioData_t pilotSeqQ[pilotSeqLen];
            memset(pilotSeqQ,0,pilotSeqLen*sizeof(radioData_t));

            radioData_t *IBurst = NULL;
            radioData_t *QBurst = NULL;

            // Use slot index, not signature, for pilot pattern
            spreadOneBranch((BitVector&) gRACHMessagePilots[slot],  // ← This is correct
                           (int8_t*) gOVSFTree.code(mRACHMessageControlSpreadingFactorLog2,
                                                    controlSpreadingCodeIndex),
                           (1 << mRACHMessageControlSpreadingFactorLog2),
                           (radioData_t *) pilotSeqQ,
                           (1 << mRACHMessageControlSpreadingFactorLog2));

            scramble(zeroIBurst, pilotSeqQ, pilotSeqLen,
                    mRACHMessageAlignedScramblingCodeI+gSlotLen*slot,
                    mRACHMessageAlignedScramblingCodeQ+gSlotLen*slot,
                    pilotSeqLen,&IBurst,&QBurst);

            signalVector RACHpilot(filtLen);
            signalVector::iterator itr = RACHpilot.begin();
            for (unsigned i = 0; i < RACHpilot.size(); i++)
              *itr++ = complex(IBurst[i+mRACHPilotsOffset],QBurst[i+mRACHPilotsOffset]);

            // Store per-signature, per-slot pilot waveform
            mRACHMessagePilotWaveforms[signature][slot] = reverseConjugate(&RACHpilot);

            delete[] IBurst;
            delete[] QBurst;
        }

    }
}

signalVector*  RadioModem::UplinkPilotWaveforms(int scramblingCode, int codeIndex, int numPilots, int slotIx)
{
  ScopedLock lock(mUplinkWaveformLock);

  int hashValue = waveformMapHash(scramblingCode,numPilots-3);
  if (mUplinkPilotWaveformMap.find(hashValue)!=mUplinkPilotWaveformMap.end()) {
	return mUplinkPilotWaveformMap[hashValue][slotIx];
  }
  else { // need to generate set of pilot vectors for scrambling code
    // NOTE: Pilot sequences are on the Q channel
    // Create a zeroed out I burst.
    radioData_t zeroIBurst[gSlotLen];
    memset(zeroIBurst,0,gSlotLen*sizeof(radioData_t));

    signalVector **newPilots = new signalVector*[gFrameSlots];

    // Figure out scrambling code 
    int Np = numPilots-3;
    if ( mUplinkScramblingCodes.find(scramblingCode)==mUplinkScramblingCodes.end() ) {
	mUplinkScramblingCodes[scramblingCode] = new UplinkScramblingCode(scramblingCode);
    }
    int8_t* scramI = (int8_t*) (mUplinkScramblingCodes[scramblingCode])->ICode();
    int8_t* scramQ = (int8_t*) (mUplinkScramblingCodes[scramblingCode])->QCode();
    unsigned seqSz = mDPCCHSearchSize; //numPilots*256;
    radioData_t pilotSeq[numPilots*256];
    for (unsigned slot = 0; slot < gFrameSlots; slot++) {
	memset(pilotSeq,0,numPilots*256*sizeof(radioData_t));
	spreadOneBranch((BitVector&) gPilotPatterns[Np][slot],
			 (int8_t *) gOVSFTree.code(8,codeIndex),
			 256,(radioData_t *) pilotSeq,numPilots*256);
        radioData_t *Iside = NULL;
	radioData_t *Qside = NULL;
	scramble((radioData_t *) zeroIBurst,(radioData_t *) pilotSeq, numPilots*256,
		scramI+gSlotLen*slot, scramQ+gSlotLen*slot, 
		numPilots*256, &Iside,&Qside);
	signalVector pilotChips(seqSz);
	signalVector::iterator itr = pilotChips.begin();
	// FIXME: need to specify correlation size here
	for (unsigned i = 0; i < seqSz; i++) 
	  *itr++ = complex(Iside[i+384],Qside[i+384]);
        newPilots[slot] = reverseConjugate(&pilotChips);
	delete[] Iside;
	delete[] Qside;
    }
    mUplinkPilotWaveformMap[hashValue] = newPilots;
    return newPilots[slotIx];
  }
}

void RadioModem::generateRACHPreambleTable(int startIx, int filtLen)
{
  unsigned scramblingCodeIx = mUplinkPRACHScramblingCodeIndex;
  LOG(INFO) << "RACH scramblingCodeIx: " << scramblingCodeIx;
  UplinkScramblingCode *sc;
  {
    ScopedLock lock(mUplinkWaveformLock);
    if (!mUplinkScramblingCodes[scramblingCodeIx]) {
        mUplinkScramblingCodes[scramblingCodeIx] = new UplinkScramblingCode(scramblingCodeIx);
        for (unsigned i = 0; i < gFrameLen; i++) {
            unsigned alignedIx = (i+4096);
            mRACHMessageAlignedScramblingCodeI[i] = *(mUplinkScramblingCodes[scramblingCodeIx]->ICode()+alignedIx);
            mRACHMessageAlignedScramblingCodeQ[i] = *(mUplinkScramblingCodes[scramblingCodeIx]->QCode()+alignedIx);
        }
    }
    sc = mUplinkScramblingCodes[scramblingCodeIx];
  }

  for (int signature = 0; signature < 16; signature++) {
    radioData_t repeatedRACHPreambleI[256*16];
    for (unsigned int i = 0; i < 256*16; i++) {
      repeatedRACHPreambleI[i] = (gRACHSignatures[signature].bit(i % 16) ? -1 : 1);
    }
    radioData_t *RACHIside = NULL;

    scrambleRACH(repeatedRACHPreambleI, 4096, (int8_t *) sc->ICode(),
	     256*16,&RACHIside);

    signalVector RACHmodBurst(filtLen);
    signalVector::iterator RACHmodBurstItr = RACHmodBurst.begin();
    for (int i = startIx; i < startIx+filtLen; i++) {
      float arg = ((float) M_PI/4.0F) + ((float) M_PI/2.0F) * (float) (i % 4);
      *RACHmodBurstItr = complex((float) RACHIside[i]*cos(arg),(float) RACHIside[i]*sin(arg));
      RACHmodBurstItr++;
    }
    delete[] RACHIside;
    mRACHTable[signature] = reverseConjugate(&RACHmodBurst);
  }
}

void RadioModem::generateDownlinkPilotWaveforms()
{

   radioData_t basicWaveformI[gSlotLen];
   radioData_t basicWaveformQ[gSlotLen];
   memset(basicWaveformI,0,gSlotLen*sizeof(radioData_t));
   memset(basicWaveformQ,0,gSlotLen*sizeof(radioData_t));

   // Note that P-SCH and S-SCH are multiplied by -1 to indicate that PCCPCH is not STTD encoded
   radioData_t aSTTD = -1;

   // Generate P-SCH
   PrimarySyncCode psc;
   for (int i = 0; i < 256; i++) { 
     basicWaveformI[i] = basicWaveformQ[i] = mPSCHAmplitude * aSTTD * ((psc.code())[i]);
     //LOG(INFO) << "basicWaveform[" << i << "] = " << (int) basicWaveformI[i];
   }

   for (unsigned slot = 0; slot < gFrameSlots; slot++) {
     // Generate S-SCH, using group number 
     SecondarySyncCode ssc(16*(gSSCAllocations[mSSCHGroupNum][slot]-1)); // 3GPP 25.213 Sec. 5.2.3.1
     mDownlinkSCHWaveformsI[slot] = new radioData_t[gSlotLen];  
     mDownlinkSCHWaveformsQ[slot] = new radioData_t[gSlotLen];
     memcpy(mDownlinkSCHWaveformsI[slot],basicWaveformI,gSlotLen*sizeof(radioData_t));
     memcpy(mDownlinkSCHWaveformsQ[slot],basicWaveformQ,gSlotLen*sizeof(radioData_t));
     for (int i = 0; i < 256; i++) { 
        //LOG(INFO) << "mDPWI[ " << slot << "][" << i << "] = " << (int) (mDownlinkSCHWaveformsI[slot])[i];
	(mDownlinkSCHWaveformsI[slot])[i] += mSSCHAmplitude * aSTTD * ((ssc.code())[i]);
        (mDownlinkSCHWaveformsQ[slot])[i] += mSSCHAmplitude * aSTTD * ((ssc.code())[i]);
	//LOG(INFO) << "mDPWI[ " << slot << "][" << i << "] = " << (int) (mDownlinkSCHWaveformsQ[slot])[i];
     }
   }

   // Generate CPICH, use 20 '0' bits per slot.
   // Primary-CPICH uses C_ch,256,0 channelization code
   BitVector CPICH("00000000000000000000");
   int8_t *code = (int8_t *) gOVSFTree.code(8,0);
   mDownlinkPilotWaveformsI = new radioData_t[gSlotLen];
   mDownlinkPilotWaveformsQ = new radioData_t[gSlotLen];
   memset(mDownlinkPilotWaveformsI,0,gSlotLen*sizeof(radioData_t));
   memset(mDownlinkPilotWaveformsQ,0,gSlotLen*sizeof(radioData_t));
   spread(CPICH, code, 256, mDownlinkPilotWaveformsI, mDownlinkPilotWaveformsQ, gSlotLen, mCPICHAmplitude);

   // Generate empty PICH, use 20 '0' bits per slot.  last 12 are supposed to be DTX, but oh well
   // PICH uses channelization code in SIB5, assumed its SF 256 and code 255.
   //BitVector PICH("00000000000000000000");
   //int8_t *codePICH = (int8_t *) gOVSFTree.code(8,255);
   //spread(PICH, codePICH, 256, mDownlinkPilotWaveformsI, mDownlinkPilotWaveformsQ, gSlotLen, mCPICHAmplitude);


}


/* NOTE: Make sure matchedfilter is already reversed and conjugated (i.e. run through reverseConjugate) */
float RadioModem::estimateChannel(signalVector *wBurst,
				 signalVector *matchedFilter,
				 unsigned maxTOA,
				 unsigned startTOA,
				 complex *channel,
				 float *TOA)
{
    // Reuse buffer to avoid per-call heap allocation (called 1500+/sec per DCH).
    static thread_local signalVector correlatedPilots(256);
    if (correlatedPilots.size() != maxTOA) correlatedPilots.resize(maxTOA);
    correlate(wBurst, matchedFilter, &correlatedPilots,
		CUSTOM, true, (matchedFilter->size()-1)+startTOA,maxTOA);
    if (channel && TOA) {
	float meanPower = 1.0;
	*channel = peakDetect(correlatedPilots,TOA,&meanPower);
	LOG(DEBUG) << "TOA: " << *TOA << "chan: " ;
	/*for(int i = -10; i < 10; i++) {
		if (floor(*TOA+i) >= maxTOA) continue;
		if (floor(*TOA+i) < 0) continue; 
		if (startTOA < 500.0) LOG(INFO) << correlatedPilots[floor(*TOA+i)] << " SNR " << correlatedPilots[floor(*TOA+i)].abs();
	}*/
	//LOG(INFO) << "TOA: " << *TOA;
	*TOA = *TOA + (float) startTOA;
	//LOG(INFO) << "TOA: " << *TOA;
	if (meanPower != 0.0) 
          return channel->norm2()/meanPower;
	else
	  return -100.0;
    }
    return 0.0;
}


void RadioModem::accumulate(radioData_t *addI, radioData_t *addQ, int addLen, radioData_t *accI, radioData_t *accQ)
{
      for (int i = 0; i < addLen;i++) {
	accI[i] += addI[i];
	accQ[i] += addQ[i];
      }
}


void RadioModem::spread(BitVector &wBurst, int8_t *code, int codeLen, radioData_t *accI, radioData_t *accQ, int accLen, radioData_t gain)
{
      int8_t *codePtrEnd = code+codeLen;
      for (unsigned i = 0; i < wBurst.size();i++) {
        unsigned byt = wBurst[i];
        if (byt == 0x7f) continue; // DTX symbol
        radioData_t *acc = ((i % 2 == 0) ? accI : accQ) + (i/2)*codeLen;
	int8_t *codePtr = code;
	radioData_t compositeGain = (2*(byt & 0x01)-1)*gain;
        while(codePtr < codePtrEnd) {
            *acc += compositeGain * *codePtr++;
	    acc++;
	}
      }
}

void RadioModem::spreadOneBranch(BitVector &wBurst, int8_t *code, int codeLen, radioData_t *acc, int accLen)
{
     // Assuming gains of each channel is +1.0
      for (unsigned i = 0; i < wBurst.size();i++) {
        bool invert = wBurst.bit(i);
        if (!invert) {
          for (int c = 0; c < codeLen; c++)
            acc[i*codeLen + c] += code[c];
        }
        else {
          for (int c = 0; c < codeLen; c++)
            acc[i*codeLen + c] -= code[c];
        }
      }
}


void RadioModem::scramble(radioData_t *wBurstI, radioData_t *wBurstQ, int len,
			  int8_t *codeI, int8_t *codeQ, int codeLen,
			  radioData_t **rBurstI, radioData_t **rBurstQ)

{
  if (*rBurstI == NULL) {
	*rBurstI = new radioData_t[len];
	memset(*rBurstI,0,sizeof(radioData_t)*len);
  }
  if (*rBurstQ == NULL) {
	*rBurstQ = new radioData_t[len];
        memset(*rBurstQ,0,sizeof(radioData_t)*len);
  }
  radioData_t *IBurst = (*rBurstI);
  radioData_t *QBurst = (*rBurstQ);
  int8_t* codeEnd = codeI + codeLen;
  while (codeI < codeEnd) {
    *IBurst += (*wBurstI * *codeI - *wBurstQ * *codeQ);
    *QBurst += (*wBurstI * *codeQ + *wBurstQ * *codeI);
    IBurst++;wBurstI++;codeI++;
    QBurst++;wBurstQ++;codeQ++;
  }
}

void RadioModem::scrambleRACH(radioData_t *wBurstI, int len,
                              int8_t *codeI, int codeLen,
                              radioData_t **rBurstI)

{
  if (*rBurstI == NULL) {
        *rBurstI = new radioData_t[len];
        //memset(*rBurstI,0,sizeof(radioData_t)*len);
  }
  for (int i = 0; i < len; i++) {
    radioData_t cI = codeI[i];
    (*rBurstI)[i] = (wBurstI[i]*cI);
  }
}


signalVector* RadioModem::descramble(signalVector &wBurst, int8_t *codeI, int8_t *codeQ, signalVector *retVec)
{
  if (retVec == NULL)
      retVec = new signalVector(wBurst.size());

  RN_MEMLOG(signalVector,retVec);
  signalVector::iterator wBurstItr = wBurst.begin();
  signalVector::iterator retItr = retVec->begin();

  for (unsigned int i = 0; i < wBurst.size(); i++) {
    *retItr++ = *wBurstItr++ * complex(*codeI++,- *codeQ++);
  }
  return retVec;
}

signalVector *RadioModem::despread(signalVector &wBurst,
				   const int8_t *code,
			           int codeLength,
				   bool useQ,
				   signalVector *retVec)
{
  int finalLength = wBurst.size()/codeLength;
  if (retVec == NULL) {
    retVec = new signalVector(finalLength);
    RN_MEMLOG(signalVector,retVec);
  }

  signalVector::iterator retItr = retVec->begin();
  signalVector::iterator burstItr = wBurst.begin();

  if (!useQ) {
    for (int i = 0; i < finalLength; i++) {
      *retItr = 0;
      for (int j = 0; j < codeLength; j++) {
	*retItr += (burstItr->real()*code[j]);
	burstItr++;
      }
      retItr++;
    }
  }
  else {
    for (int i = 0; i < finalLength; i++) {
      *retItr = 0;
      for (int j = 0; j < codeLength; j++) {
        *retItr += (burstItr->imag()*code[j]);
        burstItr++;
      }
      retItr++;
    }
  }

  return retVec;
}

bool RadioModem::detectRACHPreamble(signalVector &wBurst, UMTS::Time wTime, float detectionThreshold)
{
  UMTS::Time cpTime = wTime;
  wTime = wTime + mAICHRACHOffset;

  const unsigned MAX_PENDING_RACH = 20;

  // Check that this is a valid access slot
  bool accessSlotSet1 = (wTime.FN() % 2 == 0) && (wTime.TN() % 2 == 0);
  bool accessSlotSet2 = (wTime.FN() % 2 == 1) && (wTime.TN() % 2 == 1);
  if (!(accessSlotSet1 || accessSlotSet2)) return false;
  
  // Find a valid subchannel first
  int validSubchannel = -1;
  for (int i = 0; i < 12; i++) {
    if (!mRACHSubchannelMask[i]) continue;

    bool validSlot = (accessSlotSet1 && (gRACHSubchannels[i][wTime.FN() % 8]*2 == (int) wTime.TN())) ||
                     (accessSlotSet2 && ((gRACHSubchannels[i][wTime.FN() % 8]*2 % (int)gFrameSlots) == (int) wTime.TN()));
    if (validSlot) { validSubchannel = i; break; }
  }
  if (validSubchannel < 0) return false;

  // Correlate ALL enabled signatures and collect hits
  struct RACHHit { int sig; float SNR; float TOA; };
  std::vector<RACHHit> hits;

  for (int j = 0; j < 16; j++) {
    if (!mRACHSignatureMask[j]) continue;

    complex channel;
    float TOA;
    float SNR = estimateChannel(&wBurst, mRACHTable[j], mRACHSearchSize,
                                 mRACHPreambleOffset, &channel, &TOA);
    TOA -= mRACHPreambleOffset;

    if (SNR > 6) {
      LOG(INFO) << "signature: " << j << " SNR: " << SNR << " TOA: " << TOA << " time: " << wTime;
    }

    ScopedLock lock(mRACHStateLock);
    RACHSignatureState &state = mRACHSignatureStates[j];

    if (SNR < detectionThreshold) {
      state.consecutiveCount = 0;
      state.consecutiveTOA = 0.0;
      continue;
    }

    if (state.consecutiveCount > 0 && fabs(state.consecutiveTOA - TOA) > 2.0) {
      state.consecutiveCount = 0;
      state.consecutiveTOA = 0.0;
    }

    state.consecutiveCount++;
    state.consecutiveTOA = TOA;
    state.lastDetectionTime = wTime;

    if (state.consecutiveCount >= 1) {
      RACHHit h; h.sig = j; h.SNR = SNR; h.TOA = TOA;
      hits.push_back(h);
    }
  }

  // False positive filter: when multiple signatures fire with similar SNR,
  // it's interference (DCH UL, clock leak), not a real preamble.
  // A real preamble adds 5+ SNR to ONE signature above the others.
  if (hits.size() > 1) {
    float minSNR = hits[0].SNR, maxSNR = hits[0].SNR;
    for (size_t h = 1; h < hits.size(); h++) {
      if (hits[h].SNR < minSNR) minSNR = hits[h].SNR;
      if (hits[h].SNR > maxSNR) maxSNR = hits[h].SNR;
    }
    float diff = maxSNR - minSNR;
    if (diff < 5.0f) {
      ScopedLock lock(mRACHStateLock);
      for (size_t h = 0; h < hits.size(); h++) {
        mRACHSignatureStates[hits[h].sig].consecutiveCount = 0;
        mRACHSignatureStates[hits[h].sig].consecutiveTOA = 0.0;
      }
      return false;
    }
    // Real preamble: keep only the strongest signature
    int bestIdx = 0;
    for (size_t h = 1; h < hits.size(); h++) {
      if (hits[h].SNR > hits[bestIdx].SNR) bestIdx = h;
    }
    RACHHit best = hits[bestIdx];
    hits.clear();
    hits.push_back(best);
  }

  // Process surviving hits
  for (size_t h = 0; h < hits.size(); h++) {
    int j = hits[h].sig;
    float TOA = hits[h].TOA;

    // Check if we can accept more RACH messages
    {
      ScopedLock pendingLock(mPendingRACHLock);
      if (mPendingRACHMessages.size() >= MAX_PENDING_RACH) {
        LOG(WARNING) << "RACH queue full (" << mPendingRACHMessages.size()
                     << " pending), rejecting signature " << j;
        ScopedLock lock(mRACHStateLock);
        mRACHSignatureStates[j].consecutiveCount = 0;
        mRACHSignatureStates[j].consecutiveTOA = 0.0;
        continue;
      }
    }

    // Calculate spreading codes for THIS signature (per 25.213 Sec 4.3.1.3)
    int controlSpreadingCodeIndex = 16*j + 15;
    int dataSpreadingCodeIndex = (gConfig.getNum("UMTS.PRACH.SF") * j) / 16;

    // Calculate AICH response time
    UMTS::Time mAICHResponseTime = wTime;
    while (mAICHResponseTime <= mLastTransmitTime + UMTS::Time(0,2)) {
      mAICHResponseTime = mAICHResponseTime + UMTS::Time(2,0);
    }

    // Send AICH ACK preambles (congestion rejection is done later at the
    // RRC layer via sendRrcConnectionReject with waitTime).
    bool dummy;
    Time uselessTime;

    TxBitsBurst *out1 = new TxBitsBurst(gAICHSignatures[j].segment(0,20), 256,
                                         mAICHSpreadingCodeIndex, mAICHResponseTime, false);
    RN_MEMLOG(TxBitsBurst, out1);
    out1->AICH(true);
    addBurst(out1, dummy, uselessTime);

    TxBitsBurst *out2 = new TxBitsBurst(gAICHSignatures[j].segment(20,12), 256,
                                         mAICHSpreadingCodeIndex,
                                         mAICHResponseTime + UMTS::Time(0,1), false);
    RN_MEMLOG(TxBitsBurst, out2);
    out2->AICH(true);
    addBurst(out2, dummy, uselessTime);

    // Add to pending RACH messages queue
    {
      ScopedLock pendingLock(mPendingRACHLock);
      PendingRACHMessage pending;
      pending.signature = j;
      pending.subchannel = validSubchannel;
      pending.TOA = TOA;
      pending.controlSpreadingCodeIndex = controlSpreadingCodeIndex;
      pending.dataSpreadingCodeIndex = dataSpreadingCodeIndex;
      pending.detectionTime = wTime;
      pending.responseTime = mAICHResponseTime + UMTS::Time(0,3);
      mPendingRACHMessages.push_back(pending);
    }

    // Reset state for this signature
    {
      ScopedLock lock(mRACHStateLock);
      mRACHSignatureStates[j].consecutiveCount = 0;
      mRACHSignatureStates[j].consecutiveTOA = 0.0;
    }

    return true;
  }

  return false;
}


float RACHTFCI[32];
signalVector descrambledRACHFrame(gFrameLen);

bool RadioModem::decodeRACHMessage(signalVector &wBurst, UMTS::Time wTime, float detectionThreshold)
{
  float avgPwr;
  energyDetect(wBurst, (2560+1024)/4, 10.0, &avgPwr);

  bool processedAny = false;
  std::vector<int> completedIndices;  // Track which messages completed

  // Process all pending RACH messages in their valid time windows
  {
    ScopedLock lock(mPendingRACHLock);

    if (mPendingRACHMessages.empty()) return false;

    // Iterate through all pending messages
    for (size_t i = 0; i < mPendingRACHMessages.size(); i++) {
      PendingRACHMessage &currentRACH = mPendingRACHMessages[i];

      // Check if message window has expired
      if (currentRACH.responseTime + UMTS::Time(0, mRACHMessageSlots) <= wTime) {
        LOG(WARNING) << "RACH message window expired for signature " << currentRACH.signature
                     << ", expected at " << currentRACH.responseTime
                     << ", now " << wTime;
        completedIndices.push_back(i);  // Mark for removal
        continue;
      }

      // Check if it's time to process this message
      if (wTime < currentRACH.responseTime) continue;
      if (wTime >= currentRACH.responseTime + UMTS::Time(0, mRACHMessageSlots)) continue;

      // Process this slot for this RACH message
      int slotIx = (wTime.TN() + gFrameSlots - currentRACH.responseTime.TN()) % gFrameSlots;

      complex channel;
      float TOA;

      // Use adaptive search window based on previous slots
      float searchCenter = mRACHPilotsOffset;
      float searchWindow = 40.0;

      if (currentRACH.slotsDecoded > 0) {
        searchCenter = mRACHPilotsOffset + currentRACH.avgTOA;
        searchWindow = 10.0;
      } else {
        searchCenter = mRACHPilotsOffset + currentRACH.TOA;
        searchWindow = 20.0;
      }

      float SNR = estimateChannel(&wBurst,
                                 mRACHMessagePilotWaveforms[currentRACH.signature][slotIx],
                                 searchWindow * 2 + 1,
                                 searchCenter - searchWindow,
                                 &channel,
                                 &TOA);

      const float idealCorrelationAmplitude = 2*mRACHMessagePilotWaveforms[currentRACH.signature][slotIx]->size();
      channel = channel/idealCorrelationAmplitude;
      TOA -= mRACHPilotsOffset;

      if (currentRACH.slotsDecoded < 3 && SNR < 5.0) {
          completedIndices.push_back(i);
          continue;
      }

      // Store this slot's TOA
      currentRACH.slotTOAs[slotIx] = TOA;
      currentRACH.slotsDecoded++;

      // Update running average TOA
      float sum = 0.0;
      for (int j = 0; j < currentRACH.slotsDecoded; j++) {
        sum += currentRACH.slotTOAs[j];
      }

      currentRACH.avgTOA = sum / currentRACH.slotsDecoded;

      if (channel == complex(0,0)) channel = complex(1e6,1e6);

      signalVector RACHBurst(wBurst);
      delayVector(RACHBurst, round(-TOA));

      signalVector truncBurst(RACHBurst.begin(), 0, gSlotLen);
      scaleVector(truncBurst, complex(1.0,0.0)/channel);

      // Use per-message descrambling buffer
      signalVector descrambledRACH = currentRACH.descrambledFrame.segment(gSlotLen*slotIx, gSlotLen);

      descramble(truncBurst,
                 mRACHMessageAlignedScramblingCodeI + gSlotLen*slotIx,
                 mRACHMessageAlignedScramblingCodeQ + gSlotLen*slotIx,
                 &descrambledRACH);

      // Despread control channel
      signalVector *despreadRACHControl = despread(descrambledRACH,
                                                    gOVSFTree.code(mRACHMessageControlSpreadingFactorLog2,
                                                                   currentRACH.controlSpreadingCodeIndex),
                                                    (1 << mRACHMessageControlSpreadingFactorLog2),
                                                    true);

      // Store TFCI bits in per-message storage
      currentRACH.tfciBits[0+2*slotIx] = -0.5*((*despreadRACHControl)[8].real()/(float)(1 << mRACHMessageControlSpreadingFactorLog2))+0.5;
      currentRACH.tfciBits[1+2*slotIx] = -0.5*((*despreadRACHControl)[9].real()/(float)(1 << mRACHMessageControlSpreadingFactorLog2))+0.5;

      delete despreadRACHControl;

      processedAny = true;

      // Check if this is the last slot
      if (slotIx == gFrameSlots-1) {
        // Decode complete frame
        unsigned tfci = findTfci(currentRACH.tfciBits, 2);
        unsigned sfLog2 = mRACHMessageDataSpreadingFactorLog2 + (tfci==0);
        unsigned sfIndex = currentRACH.dataSpreadingCodeIndex * (1+(tfci==0));

        // Despread data channel using per-message buffer
        signalVector *despreadRACHData = despread(currentRACH.descrambledFrame,
                                                   gOVSFTree.code(sfLog2, sfIndex),
                                                   (1 << sfLog2),
                                                   false);

        unsigned slotSize = despreadRACHData->size()/gFrameSlots;

        for (unsigned j = 0; j < gFrameSlots; j++) {
          float dataBits[slotSize];

          for (unsigned k = 0; k < slotSize; k++)
            dataBits[k] = (-0.5)*((*despreadRACHData)[k+j*slotSize].real()/(float)(1 << sfLog2))+0.5;

          UMTS::Time slotTime = wTime;
          slotTime.decTN(currentRACH.responseTime.TN());
          slotTime.decTN(gFrameSlots-1);
          slotTime.incTN(j);

          if (currentRACH.responseTime.FN() % 2)
            slotTime.decTN(gFrameSlots);

          RxBitsBurst dataBurst(sfLog2, dataBits, slotTime, currentRACH.avgTOA, 0);
          dataBurst.mTfciBits[0] = currentRACH.tfciBits[0+2*j];
          dataBurst.mTfciBits[1] = currentRACH.tfciBits[1+2*j];
          gNodeB.mRachFec->l1WriteLowSide(dataBurst);
        }

        delete despreadRACHData;

        // Mark for removal
        completedIndices.push_back(i);
      }
    }

    // Remove completed/expired messages (iterate backwards to maintain indices)
    for (int i = completedIndices.size() - 1; i >= 0; i--) {
      int idx = completedIndices[i];
      mPendingRACHMessages.erase(mPendingRACHMessages.begin() + idx);
    }
  }

  return processedAny;
}

bool RadioModem::decodeDCH(signalVector &wBurst,
                           UMTS::Time wTime,
                           int uplinkScramblingCodeIndex,
                           int numPilots,
                           DPDCH *currDPDCH)
{
  // correlate pilots on Q-channel for slot
  int slotIx = wTime.TN();
  complex channel;
  float TOA;
  signalVector *uplinkPilots = UplinkPilotWaveforms(uplinkScramblingCodeIndex, 0, numPilots, slotIx);
  // FIXME: this start TOA should be adaptive based on previous TOA results
  float startTOA = (float)mDPCHOffset + 384; // uplink DCH is offset by 1024 chips
  startTOA += 22.0; // seems to be constant...Tx+Rx group delay of the HW perhaps.
  float SNR;
  float corrWindow = 40.0;
  bool validTOAGuess = (currDPDCH->lastTOA > -5000.0);
  if (validTOAGuess) {
    startTOA = currDPDCH->lastTOA + 384;
    corrWindow = 5.0;
  } else {
    // No valid TOA yet — wide search (corrWindow=40) is 7x more expensive
    // than narrow (corrWindow=5).  After 1 second (15 frames) without
    // finding a signal, only search once per frame (slot 0) instead of
    // every slot.  This cuts CPU for phantom/new channels from ~20% to ~2%.
    if (currDPDCH->mRxRadioFrameCounter > 15 && slotIx != 0)
      return true;
  }

  SNR = estimateChannel(&wBurst, uplinkPilots, corrWindow * 2 + 1, (startTOA - corrWindow), &channel, &TOA);

  const float idealCorrelationAmplitude = 2 * uplinkPilots->size();
  channel = channel / idealCorrelationAmplitude;
  TOA = TOA-384;

#if 1 // RTT processing
  float RTT = TOA - 22.0; // TODO: It seems that there is such constant Tx+Rx group delay ?!
  // Filter TOA samples before pushing to RTT average:
  //   1. SNR must be >= 12 dB.
  //   2. TOA must be within ±2 chips of lastTOA (continuity check).
  //   3. After signal loss (lastTOA invalidated), require 3 consecutive
  //      strong samples to agree on TOA within ±2 chips before trusting
  //      the new lock — prevents "bad lock" on random noise peaks.
  static const int BOOTSTRAP_REQUIRED = 3;
  if (SNR >= 12.0f) {
    UMTS::DCHFEC *dchFEC = static_cast<UMTS::DCHFEC*>(currDPDCH->fec);
    if (currDPDCH->lastTOA > -5000.0f) {
      // Normal tracking: already locked, apply continuity filter
      if (fabsf(TOA - currDPDCH->lastTOA) < 2.0f) {
        currDPDCH->pushRTT(slotIx, RTT);
        if (dchFEC) dchFEC->mLastRttSampleTime = Timeval();
      }
    } else {
      // Bootstrap mode: collecting agreeing samples
      if (currDPDCH->bootstrapCount == 0 ||
          fabsf(TOA - currDPDCH->bootstrapTOA) >= 2.0f) {
        // First candidate, or disagreed with previous — restart
        currDPDCH->bootstrapTOA = TOA;
        currDPDCH->bootstrapCount = 1;
      } else {
        // Agreed with current candidate
        currDPDCH->bootstrapCount++;
        if (currDPDCH->bootstrapCount >= BOOTSTRAP_REQUIRED) {
          // Lock acquired — set lastTOA and push this sample
          currDPDCH->lastTOA = TOA;
          currDPDCH->pushRTT(slotIx, RTT);
          if (dchFEC) dchFEC->mLastRttSampleTime = Timeval();
          currDPDCH->bootstrapCount = 0;
        }
      }
    }
  }

  // Update only once per 1 sec
  if ((slotIx == 0) && (++currDPDCH->mRxRadioFrameCounter % 100 == 0))
  {
      UMTS::DCHFEC *dchFEC = static_cast<UMTS::DCHFEC*>(currDPDCH->fec);
      if (dchFEC != NULL)
      {
          if (dchFEC->getPhCh() &&
              (dchFEC->getPhCh()->getMeasInterface() != NULL) &&
              (dchFEC->getPhCh()->getRttMeasHandle() > 0))
          {
              LOG(NOTICE) << "DPDCH: " << currDPDCH
                          << " UE#" << dchFEC->getPhCh()->getRttMeasHandle()
                          << " RxRadioFrameCntr: " << currDPDCH->mRxRadioFrameCounter
                          << " SlidingAvgRTT: " << currDPDCH->getAvgRTT()
                          << " SlidingSlotAvgRTT: " << currDPDCH->getSlotAvgRTT();

              RttMeasMsg *rttMeasMsg = dchFEC->getPhCh()->getMeasInterface()->allocRttMeasMsg();
              rttMeasMsg->mHandle = dchFEC->getPhCh()->getRttMeasHandle();
              rttMeasMsg->mRTT = currDPDCH->getAvgRTT(); //currDPDCH->getSlotAvgRTT();
              dchFEC->getPhCh()->getMeasInterface()->writeRttMeasMsg(rttMeasMsg);
        }
    }
  }
#else
  LOG(INFO) << "slotIx: " << slotIx << ", SNR: " << SNR << ", lastTOA: " << currDPDCH->lastTOA << ", TOA: " << TOA << " " << corrWindow << ", c: " << channel << " abs: " << channel.abs();
#endif

  // if viable correlation, demodulate data on I-channel and send to RACH decoder
  //if (SNR < detectionThreshold) return false;

  if (channel==complex(0,0))
    channel = complex(1e6, 1e6); // don't divide by zero.

  signalVector rawData = currDPDCH->rawBurst.segment(gSlotLen * slotIx, wBurst.size());
  wBurst.copyTo(rawData);

  // Per-slot: use integer-chip shift only (cheap memmove).
  // The old code ran a 21-tap sinc convolution on ~3600 samples EVERY slot
  // = 75K complex multiply-adds × 1500 slots/sec = 113M ops/sec per DCH.
  // The fractional delay is applied once per frame in decodeDPDCHFrame().
  // For control bit extraction (TFCI/TPC at SF=256), integer-chip
  // alignment is sufficient — the spreading gain absorbs sub-chip error.
  {
    int intDelay = (int) round(-TOA);
    if (intDelay < 0) {
      int shift = -intDelay;
      if (shift < (int)wBurst.size()) {
        memmove(wBurst.begin(), wBurst.begin() + shift,
                (wBurst.size() - shift) * sizeof(complex));
      }
    } else if (intDelay > 0) {
      int shift = intDelay;
      if (shift < (int)wBurst.size()) {
        memmove(wBurst.begin() + shift, wBurst.begin(),
                (wBurst.size() - shift) * sizeof(complex));
        memset(wBurst.begin(), 0, shift * sizeof(complex));
      }
    }
  }

  signalVector truncBurst(wBurst.begin(),0,gSlotLen);

  UplinkScramblingCode *sc;
  {
    ScopedLock lock(mUplinkWaveformLock);
    if (!mUplinkScramblingCodes[uplinkScramblingCodeIndex])
      mUplinkScramblingCodes[uplinkScramblingCodeIndex] = new UplinkScramblingCode(uplinkScramblingCodeIndex);
    sc = mUplinkScramblingCodes[uplinkScramblingCodeIndex];
  }

  signalVector descrambleResult = currDPDCH->descrambledBurst.segment(gSlotLen * slotIx, gSlotLen);

  // Fused scale + descramble: precompute 4 possible (invChannel × scramCode)
  // values and do ONE complex multiply per chip instead of two separate passes.
  {
    complex invCh = complex(1.0, 0.0) / channel;
    complex lut[4];
    lut[0] = invCh * complex( 1.0, -1.0);
    lut[1] = invCh * complex( 1.0,  1.0);
    lut[2] = invCh * complex(-1.0, -1.0);
    lut[3] = invCh * complex(-1.0,  1.0);
    int8_t *codeI = (int8_t *)sc->ICode() + gSlotLen * slotIx;
    int8_t *codeQ = (int8_t *)sc->QCode() + gSlotLen * slotIx;
    complex *src = truncBurst.begin();
    complex *dst = descrambleResult.begin();
    for (unsigned int i = 0; i < gSlotLen; i++) {
      int idx = ((codeI[i] > 0) ? 0 : 2) | ((codeQ[i] > 0) ? 0 : 1);
      dst[i] = src[i] * lut[idx];
    }
  }

  //if ((wTime.FN() % 100 == 0) && (!wTime.TN()))
  //  LOG(INFO) << "despread DCH data: " << *despreadDCHData;

  signalVector descrambleResultTFCITPC = descrambleResult.segment((1 << 8) * 0, (1 << 8) * 10);

  // Reuse a thread-local buffer for the per-slot control despread.
  // Size is always 10 (2560 chips / SF256).  Avoids ~3000 heap
  // alloc+free per second with 2 active DCH channels.
  static thread_local signalVector sDespreadCtrl(10);
  signalVector *despreadDCHControl = despread(descrambleResultTFCITPC,
                                              gOVSFTree.code(8, 0),
                                              (1 << 8),
                                              true,
                                              &sDespreadCtrl);

  // FIXME: assume slot format 0...need to adapt accordingly
  float bitscale = -0.5*(1.0/(float) (1 << 8)/2.0);
  currDPDCH->tfciBits[0 + 2 * slotIx] = bitscale * ((*despreadDCHControl)[0 + 6].real()) + 0.5;
  currDPDCH->tfciBits[1 + 2 * slotIx] = bitscale * ((*despreadDCHControl)[1 + 6].real()) + 0.5;
  currDPDCH->tpcBits[0 + 2 * slotIx] =  bitscale * ((*despreadDCHControl)[2 + 6].real()) + 0.5;
  currDPDCH->tpcBits[1 + 2 * slotIx] =  bitscale * ((*despreadDCHControl)[3 + 6].real()) + 0.5;
  //LOG(INFO) << "decodeDCH stop: " << wTime;

  if (slotIx != 0)
    return true;

  if (currDPDCH->bestSNR < SNR)
  {
    currDPDCH->bestTOA = TOA;
    currDPDCH->bestSNR = SNR;
    currDPDCH->bestChannel = channel;
  }

  // Update logic with bootstrap:
  //  - SNR < 3: signal lost — invalidate lastTOA, reset bootstrap
  //  - SNR 3..12: marginal — keep previous lastTOA for tracking
  //  - SNR >= 12 and already locked: update lastTOA (tracks UE movement)
  //  - SNR >= 12 and not locked: bootstrap logic in the RTT block handles it
  if (SNR > 3.0f) {
    if (SNR >= 12.0f && currDPDCH->lastTOA > -5000.0f) {
      currDPDCH->lastTOA = TOA;  // trusted tracking update
    }
  } else {
    currDPDCH->lastTOA = -10000.0f;
    currDPDCH->bootstrapCount = 0;  // reset bootstrap on total signal loss
  }

  return true;
}

bool RadioModem::decodeDPDCHFrame(DPDCH &frame,
				  int uplinkScramblingCodeIndex,
                                  int uplinkSpreadingFactorLog2,
				  int uplinkSpreadingCodeIndex)
{

        // Guard against division by zero — bestChannel can be (0,0) if
        // no valid pilot was detected in this frame (e.g. UE disconnected
        // mid-frame or interference corrupted all slots).
        if (frame.bestChannel == complex(0,0)) {
          frame.bestChannel = complex(1e6, 1e6);
        }

        // Per-frame delay: use integer-chip shift (cheap memmove) instead
        // of the full 21-tap sinc convolution on 38400 samples.
        // The sinc convolution was 38400×21 = 806K complex ops per frame
        // = 80.6M ops/sec per DCH.  For stationary/nearby UEs the
        // fractional delay adds negligible precision improvement.
        {
          int intDelay = (int) round(-frame.bestTOA);
          if (intDelay < 0) {
            int shift = -intDelay;
            if (shift < (int)frame.rawBurst.size())
              memmove(frame.rawBurst.begin(), frame.rawBurst.begin() + shift,
                      (frame.rawBurst.size() - shift) * sizeof(complex));
          } else if (intDelay > 0) {
            int shift = intDelay;
            if (shift < (int)frame.rawBurst.size()) {
              memmove(frame.rawBurst.begin() + shift, frame.rawBurst.begin(),
                      (frame.rawBurst.size() - shift) * sizeof(complex));
              memset(frame.rawBurst.begin(), 0, shift * sizeof(complex));
            }
          }
        }

        signalVector truncBurst(frame.rawBurst.begin(),0,gFrameLen);

        UplinkScramblingCode *sc;
        {
          ScopedLock lock(mUplinkWaveformLock);
          if (!mUplinkScramblingCodes[uplinkScramblingCodeIndex])
            mUplinkScramblingCodes[uplinkScramblingCodeIndex] = new UplinkScramblingCode(uplinkScramblingCodeIndex);
          sc = mUplinkScramblingCodes[uplinkScramblingCodeIndex];
        }

        // Fused scale + descramble in one pass over the frame.
        // Previously: scaleVector (38400 complex muls) then descramble
        // (38400 complex muls) = two passes = 76.8K complex ops.
        // Now: one pass with combined (1/channel) × scrambling code.
        static thread_local signalVector sDescrambleResult(0);
        if (sDescrambleResult.size() != truncBurst.size()) {
          sDescrambleResult.resize(truncBurst.size());
        }
        signalVector &descrambleResult = sDescrambleResult;

        {
          // Scrambling codes are ±1, so invChannel×complex(±1,∓1) has only
          // 4 possible values.  Precompute them and do ONE complex multiply
          // per sample instead of two (scale + descramble).
          complex invCh = complex(1.0,0.0) / frame.bestChannel;
          complex lut[4];
          lut[0] = invCh * complex( 1.0, -1.0);  // cI=+1, cQ=+1
          lut[1] = invCh * complex( 1.0,  1.0);  // cI=+1, cQ=-1
          lut[2] = invCh * complex(-1.0, -1.0);  // cI=-1, cQ=+1
          lut[3] = invCh * complex(-1.0,  1.0);  // cI=-1, cQ=-1
          int8_t *codeI = (int8_t *) sc->ICode();
          int8_t *codeQ = (int8_t *) sc->QCode();
          complex *src = truncBurst.begin();
          complex *dst = descrambleResult.begin();
          for (unsigned int i = 0; i < (unsigned)gFrameLen; i++) {
            int idx = ((codeI[i] > 0) ? 0 : 2) | ((codeQ[i] > 0) ? 0 : 1);
            dst[i] = src[i] * lut[idx];
          }
        }


	// Reuse a thread-local buffer for frame despread (once per frame per DCH).
	static thread_local signalVector sDespreadData(0);
	int despreadLen = descrambleResult.size() / (1 << uplinkSpreadingFactorLog2);
	if ((int)sDespreadData.size() != despreadLen) {
	  sDespreadData.resize(despreadLen);
	}
	signalVector *despreadDCHData = despread(descrambleResult,
                                                gOVSFTree.code(uplinkSpreadingFactorLog2,
                                                               uplinkSpreadingCodeIndex),
                                                (1 << uplinkSpreadingFactorLog2),
                                                false,
                                                &sDespreadData);

	//LOG(INFO) << "despreadDCHData: " << despreadDCHData->segment(0,1000);

	//FIXME: need to set RSSI
        float bitScale = -0.5/((float) (1 << uplinkSpreadingFactorLog2)*2.0);

#ifndef FRAMEBURSTS
	unsigned numBitsSlot = despreadDCHData->size()/gFrameSlots;
	for (unsigned j = 0; j < gFrameSlots; j++) {
	  float *dataBits = new float[numBitsSlot];
	  for (unsigned i = 0; i < numBitsSlot; i++) {
	      dataBits[i] = bitScale* ((*despreadDCHData)[i+numBitsSlot*j].real()) + 0.5;
          }
          RxBitsBurst* dataBurst = new RxBitsBurst(uplinkSpreadingFactorLog2,
						   dataBits,
						   UMTS::Time(frame.frameTime.FN(),j), 0 /*TOA*/,
						   0 /*RSSI*/);
	  //if (j == 0) LOG(INFO) << "rxbits: " << *(dynamic_cast<SoftVector*>(dataBurst));
          dataBurst->mTfciBits[0] = frame.tfciBits[0+2*j];
          dataBurst->mTfciBits[1] = frame.tfciBits[1+2*j];
          FECDispatchInfo *q = new FECDispatchInfo;
          q->fec = (void*) frame.fec;
          q->burst = dataBurst;
          RN_MEMLOG(RxBitsBurst,dataBurst);
          mDispatchQueue.write(q);
	}
#else
        unsigned numBitsFrame = despreadDCHData->size();
	LOG(INFO) << "numBitsFrame: " << numBitsFrame;
        float *dataBits = new float[numBitsFrame];
        for (unsigned i = 0; i < numBitsFrame; i++) {
            dataBits[i] = bitScale* ((*despreadDCHData)[i].real()) + 0.5;
        }
        RxBitsBurst* dataBurst = new RxBitsBurst(uplinkSpreadingFactorLog2,
                                                   dataBits,
						   numBitsFrame,
                                                   UMTS::Time(frame.frameTime.FN(),0), 0 /*TOA*/,
                                                   0 /*RSSI*/);
        {
          DCHFEC *fec = (DCHFEC*) frame.fec;
          // Check the DCH is still in the active list before using it —
          // it may have been closed by another thread (race condition).
          bool stillActive = false;
          {
            ScopedLock lock(gActiveDCH.mLock);
            for (DCHListType::const_iterator it = gActiveDCH.begin(); it != gActiveDCH.end(); ++it) {
              if (*it == fec) { stillActive = true; break; }
            }
          }
          if (stillActive && fec->active())
            fec->l1WriteLowSideFrame(*dataBurst, frame.tfciBits);
          delete[] dataBurst->begin();
          delete dataBurst;
        }
#endif

	// despreadDCHData points to thread-local sDespreadData — do not delete.

	return true;

}


void RadioModem::receiveBurst(void)
{
        char buffer[MAX_UDP_LENGTH];
        int msgLen = mDataSocket.read(buffer);

        if (msgLen<=0) SOCKET_ERROR;

        // decode
        unsigned char *rp = (unsigned char*)buffer;
        // timeslot number
        unsigned int TN = *rp++;
        // frame number
        int16_t FN = *rp++;
        FN = (FN<<8) + (*rp++);
        // soft symbols — use pre-allocated burst to avoid heap alloc.
        // The old code did new/delete signalVector(3634) per slot =
        // 1500 heap allocs/sec of 29KB each, causing fragmentation
        // and variable latency on the receive thread.
	unsigned int burstLen = gSlotLen+1024+mDelaySpread;
	if (!mPreallocBurst || mPreallocBurst->size() != burstLen) {
	  delete mPreallocBurst;
	  mPreallocBurst = new signalVector(burstLen);
	}
  	complex *burstPtr = mPreallocBurst->begin();
        for (unsigned int i=0; i<burstLen; i++) {
	  *burstPtr++ = complex((float) ((radioData_t) (signed char) (*rp)),
				(float) ((radioData_t) (signed char) (*(rp+1))));
	  rp++; rp++;
        }
	receiveSlot(mPreallocBurst, UMTS::Time(FN,TN));
	//bool underrun;
}

void RadioModem::receiveSlot(signalVector *wwBurst, UMTS::Time wTime) 
{
    signalVector *wBurst = wwBurst;

    // Inline preamble detection on access slots
    UMTS::Time adjTime = wTime + mAICHRACHOffset;
    bool isAccessSlot = ((adjTime.FN() % 2 == 0) && (adjTime.TN() % 2 == 0)) ||
                        ((adjTime.FN() % 2 == 1) && (adjTime.TN() % 2 == 1));
    if (isAccessSlot) {
      detectRACHPreamble(*wBurst, wTime, mRACHThreshold);
    }

    // Queue to RACH processor thread only for slots within a message window.
    // Previously queued ALL slots when any pending message existed (1500/sec),
    // wasting CPU on copies and iterations for slots that get skipped.
    // Queue to RACH processor thread only for slots within a message window.
    // Also expire old pending messages directly here — if we rely only on
    // decodeRACHMessage for expiration, far-future messages never get
    // processed and never expire, eventually filling the queue.
    {
      bool inWindow = false;
      {
        ScopedLock lock(mPendingRACHLock);
        // Expire old pending messages
        for (int i = mPendingRACHMessages.size() - 1; i >= 0; i--) {
          PendingRACHMessage &p = mPendingRACHMessages[i];
          if (p.responseTime + UMTS::Time(0, mRACHMessageSlots) <= wTime) {
            mPendingRACHMessages.erase(mPendingRACHMessages.begin() + i);
          }
        }
        // Check if current slot is within any remaining window
        for (size_t i = 0; i < mPendingRACHMessages.size(); i++) {
          PendingRACHMessage &p = mPendingRACHMessages[i];
          if (wTime >= p.responseTime &&
              wTime < p.responseTime + UMTS::Time(0, mRACHMessageSlots)) {
            inWindow = true;
            break;
          }
        }
      }
      if (inWindow) {
        RACHProcessorInfo *q = new RACHProcessorInfo;
        q->burst = (void*) new signalVector(*wBurst);
        q->burstTime = wTime;
        RN_MEMLOG(signalVector,wBurst);
        mRACHQueue.write(q);
      }
    }

    {
        // Snapshot active DCH under lock, then do allocation and queuing outside.
        struct DchSnap { DCHFEC *fec; int threadId; };
        DchSnap dchSnap[MAX_DCH_THREADS];
        int dchSnapCount = 0;
        {
            ScopedLock lock(gActiveDCH.mLock);
            gActiveDCH.inRxUse = true;

            if ((wTime.TN()==0) && (wTime.FN() % 4 == 0) && (gActiveDPDCH.size() > gActiveDCH.size())) {
                ScopedLock dpdchLock(gActiveDPDCHLock);
                std::set<void*> activeFecs;
                for (DCHListType::const_iterator it = gActiveDCH.begin();
                     it != gActiveDCH.end(); ++it)
                    activeFecs.insert((void*)*it);
                for (std::map<void*,DPDCH*>::iterator it = gActiveDPDCH.begin();
                     it != gActiveDPDCH.end(); ) {
                    if (activeFecs.find(it->first) == activeFecs.end()) {
                        delete it->second;
                        gActiveDPDCH.erase(it++);
                    } else {
                        ++it;
                    }
                }
            }

            // Skip channels with no valid UL signal for 500 frames (~5 sec).
            // mLastActivityFN is set from radio time (wTime.FN()) when SNR>3,
            // and -1 means "just opened" (grace period, always process).
            static const int DCH_INACTIVITY_FRAMES = 500;
            int nowFN = wTime.FN();

            for (DCHListType::const_iterator it = gActiveDCH.begin();
                 it != gActiveDCH.end() && dchSnapCount < MAX_DCH_THREADS; ++it) {
                if (!(*it)->active()) continue;
                int lastFN = (*it)->mLastActivityFN;
                if (lastFN == -1) {
                    // Just opened — set initial radio-time stamp and process
                    (*it)->mLastActivityFN = nowFN;
                } else {
                    int age = UMTS::FNDelta(nowFN, lastFN);
                    if (age > DCH_INACTIVITY_FRAMES) {
                        if (wTime.TN() == 0 && wTime.FN() % 1000 == 0)
                            LOG(NOTICE) << "DCH STALE: ch=" << (void*)*it
                                       << " no UL for " << age << " frames, skipping";
                        continue;
                    }
                }
                int tid = (*it)->getThreadId();
                if (tid < 0 || tid >= MAX_DCH_THREADS) continue;
                dchSnap[dchSnapCount].fec = *it;
                dchSnap[dchSnapCount].threadId = tid;
                dchSnapCount++;
            }
            gActiveDCH.inRxUse = false;
        }
        // Lock released — allocate and queue without blocking transmitSlot.
        if (dchSnapCount > 0) {
            std::shared_ptr<const signalVector> sharedBurst =
                std::make_shared<const signalVector>(*wBurst);
            for (int i = 0; i < dchSnapCount; i++) {
                DCHProcessorInfo *q = new DCHProcessorInfo;
                q->burst = sharedBurst;
                q->burstTime = wTime;
                q->fec = dchSnap[i].fec;
                mDCHQueue[dchSnap[i].threadId].write(q);
            }
        }
    }

    return;
}

radioData_t waveformI[gSlotLen];
radioData_t waveformQ[gSlotLen];
radioData_t* finalWaveformI = new radioData_t[gSlotLen];
radioData_t* finalWaveformQ = new radioData_t[gSlotLen];


void RadioModem::transmitSlot(UMTS::Time nowTime, bool &underrun)
{

  //LOG(DEBUG) << "RM transmitting at time: " << nowTime;

  // start with pilot waveforms
  int slotIx = nowTime.TN();
  memcpy(waveformI,mDownlinkPilotWaveformsI,sizeof(radioData_t)*gSlotLen);
  memcpy(waveformQ,mDownlinkPilotWaveformsQ,sizeof(radioData_t)*gSlotLen);

  underrun = false;


  // dump stale bursts, if any
  while (TxBitsBurst* staleBurst = mTxQueue->getStaleBurst(nowTime)) {
    LOG(NOTICE) << "dumping STALE burst in UMTS Tx Queue, nowTime: " << nowTime << ", burst time:" << staleBurst->time();
    delete staleBurst;
    underrun = true;
  }

  std::map<unsigned int,bool> receivedBursts;
  receivedBursts.clear();
  // if queue contains data at the desired timestamp, spread it and accumulate
  while (TxBitsBurst* next = (TxBitsBurst *) mTxQueue->getCurrentBurst(nowTime)) {
    //LOG(INFO) << "transmitFIFO: wrote burst " << next << " at time: " << nowTime;
    unsigned int startIx = (next->rightJustified()) ? (gSlotLen-(next->size()/2*next->SF())) : 0;
#if 0
  if (next->DCH()) //(next->SF()!=256)
    LOG(INFO) << "time: " << nowTime << ", spreading " << next->log2SF() << ", " << next->codeIndex() << ", " << next->size() << ", " << next->rightJustified() << ", " << next->DCH();
#endif
    radioData_t burstGain;
    if (next->DCH()) burstGain = mDCHAmplitude;
    else if (next->AICH()) burstGain = mCPICHAmplitude;
    else burstGain = mCCPCHAmplitude;
    spread(*next,
	   (int8_t *) gOVSFTree.code (next->log2SF(),next->codeIndex()),
	   next->SF(), waveformI+startIx, waveformQ+startIx, gSlotLen, burstGain);
    receivedBursts[(1 << next->log2SF()) + (next->codeIndex() << 16)] = true;
    delete next;
  }

  // Snapshot active DCH channels under lock, then do CPU-intensive
  // spread() calls WITHOUT holding the lock.  With 2 DCH, holding
  // gActiveDCH.mLock during 6 spread() calls blocked receiveSlot
  // for so long that the system jammed — BCH stopped broadcasting
  // and UEs lost sync.
  struct TxDCHSnap {
    int sf; int log2sf; int spcode;
    unsigned bitsInSlot, npilot, pi, ndata1, ntpc, ntfci;
    bool hasDataBurst;
    float lastUlSNR;
    float tpcTargetSnr;
  };
  TxDCHSnap txSnap[8];
  int txDchCount = 0;

  {
    ScopedLock lock(gActiveDCH.mLock);
    gActiveDCH.inTxUse = true;

    for (DCHListType::const_iterator DCHItr = gActiveDCH.begin();
         DCHItr != gActiveDCH.end() && txDchCount < 8;
         DCHItr++)
    {
        DCHFEC *currDCH = *DCHItr;
	if (!currDCH->active()) continue;
	// Same stale-channel skip as RX side (500 frames = ~5 sec)
	{ int lastFN = currDCH->mLastActivityFN;
	  if (lastFN != -1) {
	    int age = UMTS::FNDelta(nowTime.FN(), lastFN);
	    if (age > 500) continue;
	  }
	}
	TxDCHSnap &s = txSnap[txDchCount];
        s.sf = currDCH->getPhCh()->getDlSF();
	s.log2sf = currDCH->getPhCh()->getDlSFLog2();
        s.spcode = currDCH->getPhCh()->SpCode();
	SlotFormat *dlslot = currDCH->getPhCh()->getDlSlot();
	s.bitsInSlot = dlslot->mBitsPerSlot;
        s.npilot = dlslot->mNPilot;
        s.pi = dlslot->mPilotIndex;
	s.ndata1 = dlslot->mNData1;
	s.ntpc = dlslot->mNTpc;
	s.ntfci = dlslot->mNTfci;
	s.hasDataBurst = (receivedBursts.find(s.sf + (s.spcode << 16)) != receivedBursts.end());
	s.lastUlSNR = currDCH->mLastUlSNR;
	s.tpcTargetSnr = currDCH->mTpcTargetSnr;
	txDchCount++;
    }
    gActiveDCH.inTxUse = false;
  } // lock released — spread() runs without lock

  for (int d = 0; d < txDchCount; d++) {
    TxDCHSnap &s = txSnap[d];
    // SIR-based closed-loop TPC with hysteresis.
    //
    // mLastUlSNR only refreshes at slot 0 of each frame, so issuing 15
    // same-direction TPC bits from one stale measurement = ±15 dB per
    // frame → wild oscillation.  But "active only at slot 0" = ±1 dB
    // per frame, too slow to react to target changes.
    //
    // Compromise: hysteresis band of factor 2 (~±3 dB) around target.
    //   * SNR > 2× target  → DOWN every slot (fast convergence, −15 dB/frame)
    //   * SNR < ½ target   → UP every slot   (fast convergence, +15 dB/frame)
    //   * within ±3 dB of target → hold pattern (alternating bits, ±0)
    // Result: snaps to target within a frame or two of any change, then
    // sits stably inside the ±3 dB band.
    unsigned tpcField;
    float ratio = (s.tpcTargetSnr > 0.0f) ? (s.lastUlSNR / s.tpcTargetSnr) : 1.0f;
    if (ratio > 2.0f) {
      tpcField = 0;                                   // DOWN every slot
    } else if (ratio < 0.5f) {
      tpcField = (1 << s.ntpc) - 1;                   // UP every slot
    } else {
      tpcField = (slotIx & 1) ? 0 : ((1 << s.ntpc) - 1);   // hold (alternate)
    }
    BitVector TPCBits(s.ntpc);
    TPCBits.fillField(0,tpcField,s.ntpc);
    int startIx = s.sf * (s.ndata1/2);
    spread(TPCBits,
           (int8_t *) gOVSFTree.code(s.log2sf, s.spcode),
           s.sf, waveformI+startIx, waveformQ+startIx, gSlotLen, mDCHAmplitude);

    if (s.hasDataBurst) continue;

    BitVector TFCIBits(s.ntfci);
    TFCIBits.fillField(0,0,s.ntfci);
    startIx += s.sf * (s.ntpc/2);
    spread(TFCIBits,
           (int8_t *) gOVSFTree.code(s.log2sf, s.spcode),
           s.sf, waveformI+startIx, waveformQ+startIx, gSlotLen, mDCHAmplitude);

    BitVector radioSlotBits(s.npilot);
    startIx = s.sf * (s.bitsInSlot - s.npilot)/2;
    radioSlotBits.fillField(0, TrCHConsts::sDlPilotBitPattern[s.pi][slotIx], s.npilot);
    spread(radioSlotBits,
           (int8_t *) gOVSFTree.code(s.log2sf, s.spcode),
           s.sf, waveformI+startIx, waveformQ+startIx, gSlotLen, mDCHAmplitude);
  }

  // scramble
  // start with the P-SCH + S-SCH waveforms as they are unscrambled
  // NOTE: The scrambling code is aligned 256 chips into the slot, so that chip "0" falls upon the first non-zero chip of the P-CCPCH.

  memcpy(finalWaveformI, mDownlinkSCHWaveformsI[slotIx],sizeof(radioData_t)*gSlotLen);
  memcpy(finalWaveformQ, mDownlinkSCHWaveformsQ[slotIx],sizeof(radioData_t)*gSlotLen);
  scramble(waveformI, waveformQ, gSlotLen,
	   mDownlinkAlignedScramblingCodeI+gSlotLen*slotIx,
	   mDownlinkAlignedScramblingCodeQ+gSlotLen*slotIx,
	   gSlotLen,&finalWaveformI,&finalWaveformQ);

  static const int bufferSize = 2*gSlotLen+3+1;
  // Static buffer — transmitSlot is single-threaded, no need for
  // malloc/free 1500 times per second.
  static char buffer[bufferSize];
  unsigned char *wp = (unsigned char*)buffer;
  // slot
  *wp++ = nowTime.TN();
  // frame number
  uint16_t FN = nowTime.FN();
  *wp++ = (FN>>8) & 0x0ff;
  *wp++ = (FN) & 0x0ff;

  // Peak detection and scaling to avoid clipping
  {
    radioData_t peak = 0;
    for (unsigned i=0; i<gSlotLen; i++) {
      radioData_t absI = fabs(finalWaveformI[i]);
      radioData_t absQ = fabs(finalWaveformQ[i]);
      if (absI > peak) peak = absI;
      if (absQ > peak) peak = absQ;
    }
    if (peak > 127.0) {
      radioData_t scale = 127.0 / peak;
      for (unsigned i=0; i<gSlotLen; i++) {
        finalWaveformI[i] *= scale;
        finalWaveformQ[i] *= scale;
      }
    }
  }

  // copy data
  signed char *wpp = (signed char *) wp;
  for (unsigned i=0; i<gSlotLen; i++) {
     *wpp++ = (signed char) finalWaveformI[i];
     *wpp++ = (signed char) finalWaveformQ[i];
  }

  buffer[bufferSize-1] = '\0';

  // write to the socket
  mDataSocket.write(buffer,bufferSize);

  mLastTransmitTime = nowTime;
  //LOG(INFO) << LOGVAR(mLastTransmitTime) <<LOGVAR2("clock.FN",gNodeB.clock().FN());

}

void RadioModem::addBurst (TxBitsBurst *wBurst, bool &underrun, Time& updateTime)
{
  underrun = false;
  if (wBurst->time() > mLastTransmitTime) {
    mTxQueue->write(wBurst);
    assert(mTxQueue->size() < 10000);       // (pat) FIXME: Make sure someone is on the other end.
  } else {
    underrun = true;
    updateTime = mLastTransmitTime;
  }
}
