/*
 * OpenBTS provides an open source alternative to legacy telco protocols and
 * traditionally complex, proprietary hardware systems.
 *
 * Copyright 2008, 2010 Free Software Foundation, Inc.
 * Copyright 2012, 2014 Range Networks, Inc.
 *
 * This software is distributed under the terms of the GNU Affero General
 * Public License version 3. See the COPYING and NOTICE files in the main
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#include <Logger.h>

#include "TRXManager.h"

#include "UMTSCommon.h"
#include "UMTSTransfer.h"
#include "UMTSLogicalChannel.h"
#include "UMTSConfig.h"
#include "UMTSL1FEC.h"

#include "Transceiver.h"
#include "RadioInterface.h"

#include <string>
#include <string.h>
#include <stdlib.h>

#undef WARNING


using namespace UMTS;
using namespace std;

/*
 * TransceiverManager initialization.
 * Clock is now driven directly by Transceiver::writeClockInterface()
 * which calls gNodeB.clock().setFN() in-process.
 */
void TransceiverManager::TransceiverManagerInit(int numARFCNs,
                Transceiver *wTransceiver, RadioInterface *wRadioInterface)
{
        mTransceiver = wTransceiver;
        mRadioInterface = wRadioInterface;
        for (int i = 0; i < numARFCNs; i++) {
                mARFCNs.push_back(new ::ARFCNManager(wTransceiver, wRadioInterface, *this, i));
        }
}

void TransceiverManager::trxStart()
{
        for (unsigned i = 0; i < mARFCNs.size(); i++) {
                mARFCNs[i]->arfcnManagerStart();
        }
}

::ARFCNManager::ARFCNManager(Transceiver *wTrx, RadioInterface *wRadioInterface,
                             TransceiverManager &wTransceiver, unsigned wCId)
        :mTransceiver(wTransceiver),
        mTrx(wTrx),
        mRadioInterface(wRadioInterface),
        mCId(wCId)
{
        // Connect RadioModem <-> Transceiver for direct data path
        mRadioModem.setTransceiver(wTrx);
        if (wTrx) {
                wTrx->setRadioModem(&mRadioModem);
        }
        mRadioModem.radioModemStart();
}


void ::ARFCNManager::arfcnManagerStart()
{
        mTxThread.start((void*(*)(void*))TransmitLoopAdapter,this);
}

void* TransmitLoopAdapter(::ARFCNManager* manager)
{
        manager->transmitLoop();
        // do not reach
        return NULL;    // pacify the compiler
}


void ::ARFCNManager::writeHighSide(TxBitsBurst* txBurst)
{
        bool underrun;
        Time updateTime;
        mRadioModem.addBurst(txBurst,underrun,updateTime);
        if (underrun) {
                delete txBurst;
        }
}

void ::ARFCNManager::transmitLoop(void)
{
        /*
         * Slot-level TX generation driven by the Transceiver's hardware deadline clock.
         *
         * Previously this generated full frames (15 slots at once) driven by
         * gNodeB.clock().FN() which included wall-time extrapolation, causing
         * mLastTransmitTime to run 8+ frames ahead of RX -- making it impossible
         * to inject AICH at the spec-required time (5 slots after preamble).
         *
         * Now we generate one slot at a time, tracking the Transceiver's actual
         * TX deadline (what has been pushed to hardware). This keeps mLastTransmitTime
         * within a few slots of the deadline, enabling on-time AICH insertion.
         */

        // Wait for Transceiver to start streaming (powerOn -> start()).
        // The Transceiver resets mTransmitDeadlineClock in start() when
        // hardware begins streaming. We sync to this, not gNodeB.clock()
        // which uses wall-time extrapolation and runs ahead.
        UMTS::Time currTime;
        bool synced = false;

        while (1) {
          usleep(200);  // 200us -- slot-level granularity (1 slot ~ 667us)

          if (!mTrx) { synced = false; continue; }

          // Re-sync to deadline whenever the Transceiver isn't running
          // (initial start, or after a stop/restart cycle).  Without this,
          // currTime keeps the pre-stop value while start() resets the
          // deadline, leaving currTime > target forever — transmitSlot
          // stops draining mTxQueue and BCH bursts pile up to the assert.
          if (!mTrx->isRunning()) { synced = false; continue; }
          if (!synced) {
            currTime = mTrx->getTransmitDeadline();
            synced = true;
          }

          // Generate TX data AHEAD of the hardware deadline so that
          // driveTransmitFIFO always finds data in the queue.
          // The deadline is the slot being pushed to hardware RIGHT NOW.
          // We must have already generated that slot's waveform.
          //
          // Lookahead of 2 slots (~1.3ms). With Transceiver latency of 4 slots,
          // total TX lead is ~6 slots. AICH target at offset 7 hasn't been
          // composited yet (7 > 6), so AICH goes through normal Path A.
          // Lookahead 2 provides enough buffer to avoid TX underruns.
          UMTS::Time deadline = mTrx->getTransmitDeadline();
          UMTS::Time target = deadline + UMTS::Time(0, 2);

          bool underrun = false;
          while (currTime < target) {
            mRadioModem.transmitSlot(currTime, underrun);
            currTime.incTN();
          }
        }
}

/*
 * Direct control interface methods.
 * These replace the old UDP sendCommand() path.
 * Each method calls Transceiver or RadioInterface directly.
 */

bool ::ARFCNManager::powerOff()
{
        if (mTrx) mTrx->powerOff();
        return true;
}

bool ::ARFCNManager::setPower(int dB)
{
        if (mTrx) mTrx->setPower(dB);
        return true;
}

bool ::ARFCNManager::setMaxDelay(unsigned km)
{
        // Delay spread is set during Transceiver::init()
        return true;
}

signed ::ARFCNManager::setRxGain(signed rxGain)
{
        if (mTrx) return mTrx->setRxGain(rxGain);
        return -1;
}

signed ::ARFCNManager::readTxPwr(void)
{
        // Not implemented for direct interface
        return 0;
}

signed ::ARFCNManager::readRxPwrCoarse(void)
{
        // Not implemented for direct interface
        return 0;
}

long long ::ARFCNManager::readRxPwrFine()
{
        // Not implemented for direct interface
        return 0;
}

signed ::ARFCNManager::setFreqOffset(signed offset)
{
        // Not supported in PCIeSDR direct mode
        return 0;
}

signed ::ARFCNManager::getTemperature(void)
{
        // Not implemented for direct interface
        return 0;
}

signed ::ARFCNManager::getNoiseLevel(void)
{
        // Not implemented for direct interface
        return 0;
}

bool ::ARFCNManager::radioPowerOff()
{
        if (mTrx) mTrx->powerOff();
        return true;
}

bool ::ARFCNManager::radioPowerOn(bool warn)
{
        if (mTrx) return mTrx->powerOn();
        return false;
}

bool ::ARFCNManager::tune(unsigned wUARFCN)
{
        // convert ARFCN number to a frequency
        unsigned txFreq = channelFreqKHz(gNodeB.band(),wUARFCN);
        unsigned rxFreq = txFreq-uplinkOffsetKHz(gNodeB.band());

        // tune tx
        if (mTrx && !mTrx->tuneTx(txFreq)) {
                LOG(ALERT) << "TXTUNE failed";
                return false;
        }

        // tune rx
        if (mTrx && !mTrx->tuneRx(rxFreq)) {
                LOG(ALERT) << "RXTUNE failed";
                return false;
        }

        // done
        mUARFCN=wUARFCN;
        return true;
}

bool ::ARFCNManager::tuneLoopback(int wARFCN)
{
        // convert ARFCN number to a frequency
        unsigned txFreq = channelFreqKHz(gNodeB.band(),wARFCN);

        // tune rx
        if (mTrx && !mTrx->tuneRx(txFreq)) {
                LOG(ALERT) << "RXTUNE failed";
                return false;
        }
        // tune tx
        if (mTrx && !mTrx->tuneTx(txFreq)) {
                LOG(ALERT) << "TXTUNE failed";
                return false;
        }
        // done
        mUARFCN=wARFCN;
        return true;
}

bool ::ARFCNManager::powerOn()
{
        if (mTrx) return mTrx->powerOn();
        return false;
}

bool ::ARFCNManager::resetFx3()
{
        // Not implemented for PCIeSDR direct interface
        return true;
}

bool ::ARFCNManager::txPowerOn(unsigned band)
{
        // Not implemented for PCIeSDR direct interface
        return true;
}

signed ::ARFCNManager::getFactoryCalibration(const char * param)
{
        // Not implemented for PCIeSDR direct interface
        return 0;
}


// vim: ts=4 sw=4
