/*
 * OpenBTS provides an open source alternative to legacy telco protocols and
 * traditionally complex, proprietary hardware systems.
 *
 * Copyright 2008 Free Software Foundation, Inc.
 * Copyright 2011, 2014 Range Networks, Inc.
 *
 * This software is distributed under the terms of the GNU Affero General
 * Public License version 3. See the COPYING and NOTICE files in the main
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#ifndef TRXMANAGER_H
#define TRXMANAGER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Threads.h>
#include <Interthread.h>
#include "UMTSCommon.h"
#include "UMTSTransfer.h"
#include <list>

#include "UMTSRadioModem.h"

class Transceiver;
class RadioInterface;

/* Forward refs into the UMTS namespace. */
namespace UMTS {

class L1Decoder;

};

class ARFCNManager;


/**
	The TransceiverManager processes the complete transcevier interface.
	There is one of these for each access point.
 */
class TransceiverManager {

	private:

	/// the ARFCN manangers under this TRX
	std::vector<ARFCNManager*> mARFCNs;

	/// Transceiver and RadioInterface pointers for direct access
	Transceiver *mTransceiver;
	RadioInterface *mRadioInterface;

	public:

	/**
		Construct a TransceiverManager.
		@param numARFCNs Number of ARFCNs supported by the transceiver.
		@param wTransceiver Pointer to the in-process Transceiver.
		@param wRadioInterface Pointer to the RadioInterface.
	*/
	void TransceiverManagerInit(int numARFCNs,
		Transceiver *wTransceiver, RadioInterface *wRadioInterface);

	/**@name Accessors. */
	//@{
	ARFCNManager* ARFCN(unsigned i) { assert(i<mARFCNs.size()); return mARFCNs.at(i); }
	//@}

	unsigned numARFCNs() const { return mARFCNs.size(); }

	/** Start all ARFCN managers. Clock is now driven directly by Transceiver. */
	void trxStart();
};




/**
	The ARFCN Manager processes transceiver functions for a single ARFCN.
	When we do frequency hopping, this will manage a full rate radio channel.
*/
class ARFCNManager {

	private:

	TransceiverManager &mTransceiver;

	/// Direct pointers to transceiver and radio interface (replaces UDP sockets)
	Transceiver *mTrx;
	RadioInterface *mRadioInterface;

	Thread mTxThread;

	unsigned mUARFCN;			///< the current UARFCN

	UMTS::RadioModem mRadioModem;

	public:

	ARFCNManager(Transceiver *wTrx, RadioInterface *wRadioInterface,
	             TransceiverManager &wTRX, unsigned wCId);

	unsigned mCId;

	/** Start the transmit thread. */
	void arfcnManagerStart();
	const char *getId() { static char buf[8]; sprintf(buf,"C%d",mCId); return buf; }

	unsigned UARFCN() const { return mUARFCN; }

	void writeHighSide(UMTS::TxBitsBurst* burst);

	/**@name Transceiver controls (direct calls, no UDP). */
	//@{

	/**
		Tune to a given UARFCN.
		@param wUARFCN Target for tx/rx tuning.
		@return true on success.
	*/
	bool tune(unsigned wUARFCN);

	/**
		Tune to a given ARFCN, but with rx and tx on the same (downlink) frequency.
		@param wARFCN Target for tuning, using downlink frequeny.
		@return true on success.
	*/
	bool tuneLoopback(int wARFCN);

	/**
		Turn on the transceiver.
	*/
	bool powerOn();

	/** Turn off the transceiver. */
	bool powerOff();

        /***/
        bool resetFx3();

	/**
		Turn on the transceiver.
		@param warn Warn if the transceiver fails to start
	*/
	bool radioPowerOn(bool warn);

	/** Turn off the transceiver. */
	bool radioPowerOff();

	/** Turn on the transmitter. */
	bool txPowerOn(unsigned band);

	/** Just test if the transceiver is running without printing alarming messages. */
	bool trxRunning() {return false;}

        /**
		Set maximum expected delay spread.
		@param km Max network range in kilometers.
		@return true on success.
        */
        bool setMaxDelay(unsigned km);

        /**
                Set radio receive gain.
                @param new desired gain in dB.
                @return new gain in dB.
        */
        signed setRxGain(signed dB);

        /**
                Set radio frequency offset
                @param new desired freq. offset
                @return new freq. offset
        */
        signed setFreqOffset(signed offset);

        /**
                Get noise level as RSSI.
                @return current noise level.
        */
        signed getNoiseLevel(void);

        /***/
        signed getTemperature(void);

        /**
                Get the Tx Power.
                @return current Tx Power
        */
        signed readTxPwr(void);

        /**
                Get the Rx Power.
                @return current Rx Power
        */
        signed readRxPwrCoarse(void);

        /**
                Get the Rx Power.
                @return current Rx Power
        */
        long long readRxPwrFine();

	/**
		Set power wrt full scale.
		@param dB Power level wrt full power.
		@return true on success.
	*/
	bool setPower(int dB);

	/**
	Get factory calibration values.
	@return current eeprom values.
	*/
	signed getFactoryCalibration(const char * param);

	//@}

	/** Install a decoder on this ARFCN. */
	void installDecoder(UMTS::L1Decoder* wL1);


	private:

        /** Transmit loop; runs in the transmit thread. */
        void transmitLoop();

        /** Transmitter loop. */
        friend void* TransmitLoopAdapter(ARFCNManager*);

};

/** C interface for ARFCNManager threads. */
void* TransmitLoopAdapter(ARFCNManager*);

#endif // TRXMANAGER_H
// vim: ts=4 sw=4
