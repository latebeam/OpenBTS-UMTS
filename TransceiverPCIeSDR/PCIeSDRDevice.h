/*
 * Device support for PCIeSDR
 * Based on TransceiverUHD/UHDDevice.h
 *
 * Copyright 2026 Late Beam
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 *
 */

#ifndef PCIESDR_DEVICE_H
#define PCIESDR_DEVICE_H

#include "RadioDevice.h"
#include "SampleBuffer.h"

#define TX_AMPL          0.3

//PCIE
#define UMTSRATE 6250000
extern "C" {
#include "libsdr.h"
}	
#include <sys/time.h>
#include <math.h>
#include <limits.h>
#include <string>
#include <iostream>
#include <chrono>


typedef unsigned long long TIMESTAMP;

class PCIeSDRDevice : public RadioDevice {
public:
// PCIE
 	unsigned int tx_sps;
	unsigned int rx_sps;

	PCIeSDRDevice(double rate);
	~PCIeSDRDevice();

	bool open(const std::string &args, bool extref, const int transceiver_index);
	bool start();
	bool stop();
	bool restart();
	void setPriority();
	enum TxWindowType getWindowType() { return tx_window; }

	int readSamples(short *buf, int len, bool *overrun,
			long long timestamp, bool *underrun, unsigned *RSSI);

	int writeSamples(short *buf, int len,
			 bool *underrun, long long timestamp);

    bool setVCTCXO(unsigned int) { return true; };

	bool setTxFreq(double wFreq);
	bool setRxFreq(double wFreq);

	inline long long initialWriteTimestamp() { return ts_initial; }
	inline long long initialReadTimestamp() { return ts_initial; }

	inline double fullScaleInputValue() { return 32000 * TX_AMPL; }
	inline double fullScaleOutputValue() { return 32000; }

	double setRxGain(double db);
	double getRxGain(void) { return rx_gain; }
	double maxRxGain(void) { return rx_gain_max; }
	double minRxGain(void) { return rx_gain_min; }

	double setTxGain(double db);
	double maxTxGain(void) { return tx_gain_max; }
	double minTxGain(void) { return tx_gain_min; }

	double getTxFreq() { return tx_freq; }
	double getRxFreq() { return rx_freq; }

	inline double getSampleRate() { return tx_rate; }
	inline double numberRead() { return rx_pkt_cnt; }
	inline double numberWritten() { return 0; }

	/** Receive and process asynchronous message
	    @return true if message received or false on timeout or error
	*/
	bool recv_async_msg();

	enum err_code {
		ERROR_TIMING = -1,
		ERROR_UNRECOVERABLE = -2,
		ERROR_UNHANDLED = -3,
	};

private:
	int count = 0;
	long long initialTimestamp;

	///PCIESDR
	MultiSDRState* device;
	SDRStartParams StartParams;
	typedef struct {
		float re;
		float im;
	} sample_t;
	unsigned int dma_buffer_count;
	unsigned int dma_buffer_len;

	double actualSampleRate;	///< the actual sampling rate

	uint32_t samplesSentInOneSecond;
	struct timeval counterRealTime;

	bool started;
	bool skipRx;			///< set if device is transmit-only.


	TIMESTAMP ts_initial, ts_offset;

	std::vector<double> tx_gains, rx_gains;
	int64_t tx_underflow;
	int64_t rx_overflow;

	enum TxWindowType tx_window;

	double tx_gain, tx_gain_min, tx_gain_max;
	double rx_gain, rx_gain_min, rx_gain_max;

	double tx_rate, rx_rate;
	double tx_freq, rx_freq;

	bool aligned;

	size_t rx_pkt_cnt;
	size_t drop_cnt;
	long long prev_ts;

	SampleBuffer *rx_buffer;

	bool flush_recv(size_t num_pkts);

	double readTxGainFromFile(const std::string filename);

	Thread *async_event_thrd;
	
	double tx_gain_from_file;
};

#endif /* PCIESDR_DEVICE_H */
