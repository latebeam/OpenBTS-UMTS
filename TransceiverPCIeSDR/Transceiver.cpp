/*
 * Based on TransceiverUHD/Transceiver.cpp
 *
 * OpenBTS provides an open source alternative to legacy telco protocols and
 * traditionally complex, proprietary hardware systems.
 *
 * Copyright 2008, 2009, 2010 Free Software Foundation, Inc.
 * Copyright 2014 Range Networks, Inc.
 * Copyright 2014 Ettus Research LLC
 * Copyright 2026 Late Beam
 *
 * This software is distributed under the terms of the GNU General Public
 * License version 3. See the COPYING and NOTICE files in the current
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#include <stdio.h>
#include <Logger.h>
#include "Transceiver.h"
#include "UMTSRadioModem.h"
#include "UMTSConfig.h"

/* Clock indication reporting interval in frames */
#define CLK_IND_INTERVAL          1

/* Default attenuation value in dB */
#define DEFAULT_ATTEN             20

extern UMTS::UMTSConfig gNodeB;

Transceiver::Transceiver(UMTS::Time wTransmitLatency,
                         RadioInterface *wRadioInterface)
  : mTxServiceLoopThread(NULL), mRxServiceLoopThread(NULL),
    mOn(false), mPower(DEFAULT_ATTEN),
    mTransmitLatency(wTransmitLatency), mRadioInterface(wRadioInterface),
    mRadioModem(NULL)
{
  signalVector emptyVector(UMTS::gSlotLen);
  UMTS::Time emptyTime(0, 0);
  mEmptyTransmitBurst = new radioVector((const signalVector&) emptyVector,
                                        (UMTS::Time&) emptyTime);
}

Transceiver::~Transceiver()
{
  stop();
  delete mEmptyTransmitBurst;
}

/*
 * Initialize transceiver
 *
 * Randomize the central radio clock and set the downlink burst counters.
 */
void Transceiver::init(int wDelaySpread)
{
  if (wDelaySpread < 0)
    wDelaySpread = 0;

  stop();

  UMTS::Time time(random() % 1024, 0);
  mRadioInterface->getClock()->set(time);
  mTransmitDeadlineClock = time;
  mLastClockUpdateTime = time;
  mLatencyUpdateTime = time;

  mDelaySpread = wDelaySpread;
  mPower = mRadioInterface->setPowerAttenuation(mPower);
}

/*
 * Start the transceiver
 *
 * Submit command(s) to the radio device to commence streaming samples and
 * launch threads to handle sample I/O.
 */
bool Transceiver::start()
{
  ScopedLock lock(mLock);

  if (mOn) {
    LOG(ERR) << "Transceiver already running";
    return false;
  }

  LOG(NOTICE) << "Starting the transceiver";

  UMTS::Time time = mRadioInterface->getClock()->get();
  mTransmitDeadlineClock = time;
  mLastClockUpdateTime = time;
  mLatencyUpdateTime = time;

  if (!mRadioInterface->start()) {
    LOG(ALERT) << "Device failed to start";
    return false;
  }

  /* Device is running - launch I/O threads */
  mTxServiceLoopThread = new Thread(8 * 32768);
  mRxServiceLoopThread = new Thread(8 * 32768);

  mTxServiceLoopThread->start((void * (*)(void*))
                              TxServiceLoopAdapter, (void*) this);
  mRxServiceLoopThread->start((void * (*)(void*))
                              RxServiceLoopAdapter, (void*) this);
  writeClockInterface();

  mOn = true;
  LOG(NOTICE) << "Transceiver running";
  return true;
}

/*
 * Stop the transceiver
 *
 * Perform stopping by disabling receive streaming and issuing cancellation
 * requests to running threads.
 */
void Transceiver::stop()
{
  ScopedLock lock(mLock);

  if (!mOn)
    return;

  LOG(NOTICE) << "Stopping the transceiver";
  mTxServiceLoopThread->cancel();
  mRxServiceLoopThread->cancel();

  LOG(INFO) << "Stopping the device";
  mRadioInterface->stop();

  LOG(INFO) << "Terminating threads";
  mRxServiceLoopThread->join();
  mTxServiceLoopThread->join();

  delete mTxServiceLoopThread;
  delete mRxServiceLoopThread;

  mTransmitPriorityQueue.clear();

  mOn = false;
  LOG(NOTICE) << "Transceiver stopped";
}

void Transceiver::addRadioVector(signalVector &burst, UMTS::Time &wTime)
{
  // modulate and stick into queue
  radioVector *vec = new radioVector(burst, wTime);
  RN_MEMLOG(radioVector, vec);
  mTransmitPriorityQueue.write(vec);
}

void Transceiver::pushRadioVector(UMTS::Time &now)
{
  radioVector *stale, *next;

  // dump stale bursts, if any
  while ((stale = mTransmitPriorityQueue.getStaleBurst(now))) {
    LOG(NOTICE) << "dumping STALE burst in TRX interface burst:"
                << stale->time() << " now:" << now;
    writeClockInterface();
    delete stale;
  }

  // if queue contains data at the desired timestamp, stick it into FIFO
  if ((next = (radioVector*) mTransmitPriorityQueue.getCurrentBurst(now))) {
    // Check for AICH overlay: if a RACH preamble was detected after this
    // slot was composited by transmitSlot, the AICH waveform is waiting
    // here to be added to the burst before hardware push.
    {
      ScopedLock lock(mAICHOverlayLock);
      auto it = mAICHOverlays.find(now);
      if (it != mAICHOverlays.end()) {
        signalVector *overlay = it->second;
        signalVector::iterator src = overlay->begin();
        radioVector::iterator dst = next->begin();
        unsigned len = overlay->size();
        if (len > next->size()) len = next->size();
        // Measure overlay energy and base signal energy before applying
        float overlayEnergy = 0, baseEnergy = 0;
        {
          signalVector::iterator s = overlay->begin();
          radioVector::iterator d = next->begin();
          for (unsigned i = 0; i < len; i++) {
            overlayEnergy += s->real()*s->real() + s->imag()*s->imag();
            baseEnergy += d->real()*d->real() + d->imag()*d->imag();
            s++; d++;
          }
        }
        for (unsigned i = 0; i < len; i++) {
          *dst = complex(dst->real() + src->real(), dst->imag() + src->imag());
          dst++; src++;
        }
        // Measure combined energy after applying
        float combinedEnergy = 0;
        {
          radioVector::iterator d = next->begin();
          for (unsigned i = 0; i < len; i++) {
            combinedEnergy += d->real()*d->real() + d->imag()*d->imag();
            d++;
          }
        }
        LOG(INFO) << "Applied AICH overlay at " << now
                   << " overlayE=" << overlayEnergy
                   << " baseE=" << baseEnergy
                   << " combinedE=" << combinedEnergy
                   << " sample[0]=(" << next->begin()->real() << "," << next->begin()->imag() << ")";
        delete overlay;
        mAICHOverlays.erase(it);
      }
    }
    mRadioInterface->driveTransmitRadio(*(next), false);
    delete next;
    return;
  }

  // Check for missed overlays (slot had no radioVector to overlay onto)
  {
    ScopedLock lock(mAICHOverlayLock);
    auto it = mAICHOverlays.find(now);
    if (it != mAICHOverlays.end()) {
      LOG(INFO) << "AICH overlay MISSED (no burst) at " << now;
      delete it->second;
      mAICHOverlays.erase(it);
    }
  }

  // Extremely rare that we get here. We need to send a blank burst to the
  // radio interface to update the timestamp.
  LOG(INFO) << "Sending empty burst at " << now;
  mRadioInterface->driveTransmitRadio(*(mEmptyTransmitBurst), true);
}

void Transceiver::queueAICHOverlay(signalVector *overlay, UMTS::Time targetTime)
{
  ScopedLock lock(mAICHOverlayLock);
  // Replace any existing overlay for this slot (shouldn't happen normally)
  auto it = mAICHOverlays.find(targetTime);
  if (it != mAICHOverlays.end()) {
    delete it->second;
    mAICHOverlays.erase(it);
  }
  mAICHOverlays[targetTime] = overlay;
  LOG(INFO) << "Queued AICH overlay for " << targetTime;
}

void Transceiver::reset()
{
  mTransmitPriorityQueue.clear();
}

/*
 * Direct control interface methods (replaces UDP control socket)
 */
bool Transceiver::powerOn()
{
  return start();
}

void Transceiver::powerOff()
{
  stop();
}

int Transceiver::setRxGain(int dB)
{
  return (int) mRadioInterface->setRxGain((double) dB);
}

int Transceiver::setPower(int dB)
{
  mPower = mRadioInterface->setPowerAttenuation(dB);
  return mPower;
}

bool Transceiver::tuneTx(int freqKHz)
{
  return mRadioInterface->tuneTx(freqKHz * 1e3);
}

bool Transceiver::tuneRx(int freqKHz)
{
  return mRadioInterface->tuneRx(freqKHz * 1e3);
}

/*
 * Receive path: pull from RadioInterface, deliver directly to RadioModem
 *
 * This replaces the old UDP serialization path. Data flows:
 * RadioInterface::driveReceiveRadio -> mReceiveFIFO -> here -> RadioModem::receiveSlot
 */
void Transceiver::driveReceiveFIFO()
{
  radioVector *rxBurst = NULL;
  UMTS::Time burstTime;

  mRadioInterface->driveReceiveRadio(1024 + mDelaySpread);

  rxBurst = (radioVector *) mReceiveFIFO->get();
  if (!rxBurst)
    return;

  burstTime = rxBurst->time();

  // Pass received samples directly to RadioModem without transformation.
  // radioVector inherits from signalVector; copy to local so we can delete rxBurst.
  signalVector dataBurst(*rxBurst);

  delete rxBurst;

  if (!burstTime.TN() && !(burstTime.FN() % CLK_IND_INTERVAL))
    writeClockInterface();

  // Deliver directly to RadioModem (no UDP)
  if (mRadioModem)
    mRadioModem->receiveSlot(&dataBurst, burstTime);
}

/*
 * Features a carefully controlled latency mechanism, to
 * assure that transmit packets arrive at the radio
 * before they need to be transmitted.
 *
 * Deadline clock indicates the burst that needs to be
 * pushed into the FIFO right NOW.  If transmit queue does
 * not have a burst, stick in filler data.
 */
void Transceiver::driveTransmitFIFO()
{
  RadioClock *radioClock = mRadioInterface->getClock();

  if (!mOn)
    return;

  radioClock->wait();

  while (radioClock->get() + mTransmitLatency > mTransmitDeadlineClock) {
    // Dynamic latency adjustment for PCIeSDR.
    // Keep latency minimal (4-8 slots) for spec-compliant AICH timing.
    if (mRadioInterface->getWindowType() == RadioDevice::TX_WINDOW_PCIE) {
      if (mRadioInterface->isUnderrun()) {
        if (radioClock->get() > mLatencyUpdateTime + UMTS::Time(10, 0)) {
          mTransmitLatency = mTransmitLatency + UMTS::Time(0, 2);
          if (mTransmitLatency > UMTS::Time(0, 8))
            mTransmitLatency = UMTS::Time(0, 8);

          LOG(INFO) << "new latency: " << mTransmitLatency;
          mLatencyUpdateTime = radioClock->get();
        }
      } else {
        // if no underrun in 50 frames, reduce latency by 1 slot
        if (mTransmitLatency > UMTS::Time(0, 4)) {
          if (radioClock->get() > mLatencyUpdateTime + UMTS::Time(50, 0)) {
            mTransmitLatency.decTN();
            LOG(INFO) << "reduced latency: " << mTransmitLatency;
            mLatencyUpdateTime = radioClock->get();
          }
        }
      }
    }

    // time to push burst to transmit FIFO
    pushRadioVector(mTransmitDeadlineClock);
    mTransmitDeadlineClock.incTN();
  }
}

/*
 * Update gNodeB clock directly (replaces UDP clock socket).
 * Previously this serialized "IND CLOCK <FN>" over a UDP socket.
 * Now it updates gNodeB.clock() in-process.
 */
void Transceiver::writeClockInterface()
{
  unsigned FN = (unsigned)(mTransmitDeadlineClock.FN() + 1);

  gNodeB.clock().setFN(FN);
  mLastClockUpdateTime = mTransmitDeadlineClock;
}

void *RxServiceLoopAdapter(Transceiver *transceiver)
{
  while (1) {
    transceiver->driveReceiveFIFO();
    pthread_testcancel();
  }
  return NULL;
}

void *TxServiceLoopAdapter(Transceiver *transceiver)
{
  while (1) {
    transceiver->driveTransmitFIFO();
    pthread_testcancel();
  }
  return NULL;
}
