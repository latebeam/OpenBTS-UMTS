/*
 * Based on TransceiverUHD/Transceiver.h
 *
 * Copyright 2026 Late Beam
 *
 * This software is distributed under the terms of the GNU General Public
 * License version 3. See the COPYING and NOTICE files in the current
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#ifndef TRANSCEIVER_H
#define TRANSCEIVER_H

#include "RadioInterface.h"
#include "Interthread.h"
#include "UMTSCommon.h"

#include <sys/types.h>
#include <map>

namespace UMTS { class RadioModem; }

/** The Transceiver class, responsible for physical layer of basestation */
class Transceiver {
private:
  VectorQueue  mTransmitPriorityQueue;   ///< priority queue of transmit bursts received from UMTS core
  VectorFIFO*  mTransmitFIFO;      ///< radioInterface FIFO of transmit bursts
  VectorFIFO*  mReceiveFIFO;       ///< radioInterface FIFO of receive bursts

  RadioBurstFIFO  mT;
  RadioBurstFIFO  mR;

  Thread *mTxServiceLoopThread;    ///< thread to push/pull bursts into transmit/receive FIFO
  Thread *mRxServiceLoopThread;    ///< thread to push/pull bursts into transmit/receive FIFO

  bool mOn;                        ///< flag to indicate that transceiver is powered on
  int mPower;                      ///< the transmit power in dB
  int mDelaySpread;                ///< maximum expected delay spread, i.e. extend buffer when sending upstream

  UMTS::Time mTransmitLatency;     ///< latency between basestation clock and transmit deadline clock
  UMTS::Time mLatencyUpdateTime;   ///< last time latency was updated

  UMTS::Time mTransmitDeadlineClock;       ///< deadline for pushing bursts into transmit FIFO
  UMTS::Time mLastClockUpdateTime;         ///< last time clock update was sent up to core
  radioVector *mEmptyTransmitBurst;

  RadioInterface *mRadioInterface; ///< associated radioInterface object
  UMTS::RadioModem *mRadioModem;   ///< associated RadioModem for direct RX delivery

  /** AICH overlay map: signalVectors to add to TX bursts before hardware push.
      Used when RACH preamble is detected after transmitSlot has already composited
      the target slot. The overlay adds the AICH to the queued radioVector. */
  std::map<UMTS::Time, signalVector*> mAICHOverlays;
  Mutex mAICHOverlayLock;

  /** Push modulated burst into transmit FIFO corresponding to a particular timestamp */
  void pushRadioVector(UMTS::Time &nowTime);

  /** Update gNodeB clock directly (replaces UDP clock socket) */
  void writeClockInterface(void);

  /** Start and stop I/O threads */
  bool start();
  void stop();

  /** Protect destructor accessable stop call */
  Mutex mLock;

public:
  /** Transceiver constructor
      @param wTransmitLatency initial setting of transmit latency
      @param radioInterface associated radioInterface object
  */
  Transceiver(UMTS::Time wTransmitLatency,
        RadioInterface *wRadioInterface);

  /** Destructor */
  ~Transceiver();

  /** Set the RadioModem for direct RX delivery (call after RadioModem is created) */
  void setRadioModem(UMTS::RadioModem *rm) { mRadioModem = rm; }

  /** Start the control loop and I/O threads */
  void init(int wDelaySpread);

  /** attach the radioInterface receive FIFO */
  void receiveFIFO(VectorFIFO *wFIFO) { mReceiveFIFO = wFIFO; }

  /** attach the radioInterface transmit FIFO */
  void transmitFIFO(VectorFIFO *wFIFO) { mTransmitFIFO = wFIFO; }

  RadioBurstFIFO* highSideTransmitFIFO(void) { return &mT; }
  RadioBurstFIFO* highSideReceiveFIFO(void)  { return &mR; }

  /** modulate and add a burst to the transmit queue (public for direct RadioModem access) */
  void addRadioVector(signalVector &burst, UMTS::Time &wTime);

  /** Direct control interface (replaces UDP control socket) */
  bool powerOn();
  void powerOff();
  int setRxGain(int dB);
  int setPower(int dB);
  bool tuneTx(int freqKHz);
  bool tuneRx(int freqKHz);

  /** Get the radio interface */
  RadioInterface* getRadioInterface() { return mRadioInterface; }

  /** Get the current TX deadline clock -- used by transmitLoop for tight TX generation.
      Returns the furthest slot that has been pushed to hardware.
      transmitLoop should generate slots up to (but not beyond) this value. */
  UMTS::Time getTransmitDeadline() { return mTransmitDeadlineClock; }

  /** Check if transceiver is actively streaming */
  bool isRunning() const { return mOn; }

  /** Queue an AICH overlay to be added to a TX burst before hardware push.
      Called from the RX thread when RACH preamble is detected and the AICH
      target slot has already been composited by transmitSlot.
      @param overlay signalVector to add (takes ownership, will be deleted after use)
      @param targetTime the slot time to overlay onto */
  void queueAICHOverlay(signalVector *overlay, UMTS::Time targetTime);

protected:
  /** drive reception and demodulation of UMTS bursts */
  void driveReceiveFIFO();

  /** drive transmission of UMTS bursts */
  void driveTransmitFIFO();

  friend void *TxServiceLoopAdapter(Transceiver *);
  friend void *RxServiceLoopAdapter(Transceiver *);

  void reset();
};

/** FIFO thread loop */
void *TxServiceLoopAdapter(Transceiver *);
void *RxServiceLoopAdapter(Transceiver *);

#endif /* TRANSCEIVER_H */
