/*
 * Device support for PCIeSDR
 * Based on TransceiverUHD/UHDDevice.cpp
 *
 * Copyright 2010-2011 Free Software Foundation, Inc.
 * Copyright 2014 Ettus Research LLC
 * Patched by FlUxIuS @ Penthertz SAS
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

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "Threads.h"
#include "Logger.h"
#include "PCIeSDRDevice.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define SAMPLE_BUF_SZ		(1 << 20)

#define WAIT_TX_GAIN_TIME_SEC 7
std::chrono::steady_clock::time_point global_start_time;
bool waiting_tx_gain = false;
#define PSAMPLES_NUM 5000

extern "C" {
	#include "convert.h"
}



extern "C" {
	#include "libsdr.h"
}


/* greatest common divisor */
static long gcd(long a, long b)
{
	if (a == 0)
		return b;
	else if (b == 0)
		return a;

	if (a < b)
		return gcd(a, b % a);
	else
		return gcd(b, a % b);
}


static void *async_event_loop(PCIeSDRDevice *dev)
{
	while (1) {
		dev->recv_async_msg();
		pthread_testcancel();
	}

	return NULL;
}

static void thread_enable_cancel(bool cancel)
{
	cancel ? pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL) :
		 pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
}

PCIeSDRDevice::PCIeSDRDevice(double rate)
	: tx_gain_min(0.0), tx_gain_max(89.0),
	  rx_gain_min(0.0), rx_gain_max(60.0),
	  tx_rate(rate), rx_rate(rate), tx_freq(0.0), rx_freq(0.0),
	  started(false), aligned(false), rx_pkt_cnt(0), drop_cnt(0),
	  prev_ts(0), ts_offset(0), rx_buffer(NULL)
{
      
	dma_buffer_count = 50;
	dma_buffer_len = 1250;

	initialTimestamp = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
}

PCIeSDRDevice::~PCIeSDRDevice()
{
	stop();
	delete rx_buffer;
}

double PCIeSDRDevice::setTxGain(double db)
{
	//LOG(DEBUG) << "setTxGain call";
	//msdr_set_tx_gain(device, 0, db);
	return db;
}

double PCIeSDRDevice::setRxGain(double db)
{
	StartParams.rx_gain[0] = db;
	
	LOG(INFO) << "setRxGain: " << db;
	if (started)
	{
	  int res = msdr_set_rx_gain(device, 0, db);
	  if (res)
	  {
	    LOG(ERR) << "setRxGain error: " << res;
	  }
	}
	
	return db;
}


#define  DEFAULT_TX_GAIN 0
double PCIeSDRDevice::readTxGainFromFile(const std::string filename) {
#if 0 // override gain from file
    return 50;
#endif

    std::ifstream inputFile(filename);
	
    if (!inputFile.is_open()) {
        LOG(ERR) << "Error: Could not open file " << filename;
        return DEFAULT_TX_GAIN;
    }

    double value;
    inputFile >> value;

    if (inputFile.fail()) {
        LOG(ERR) << "Error: Failed to read a double value from the file.";
        return DEFAULT_TX_GAIN;
    }

    LOG(INFO) << "Read value: " << value << std::endl;
	if (value > tx_gain_max)
	{
		LOG(WARNING) << "Too high tx gain value: " << value << " setting to max: " << tx_gain_max;
		value = 0;
	}

    inputFile.close();
    return value;
}


bool PCIeSDRDevice::open(const std::string &args, bool extref, const int transceiver_index)
{
	char pciesdr_name[500];
	char trx_gain_filename[256];

	LOG(INFO) << "Opening PCIESDR device.. index = " << transceiver_index;


	sprintf(pciesdr_name,"dev0=/dev/sdr%d", transceiver_index);
	started = false;

	device = msdr_open(pciesdr_name);
	if (device == NULL) {
		LOG(ALERT) << "PCIESDR creating failed, device " << pciesdr_name << "";
		return -1;
	}

	msdr_set_default_start_params(device, &StartParams, sizeof(StartParams), 1, 1, 1);

	StartParams.flags1 = 1;

	StartParams.sync_source = SDR_SYNC_INTERNAL;
	StartParams.clock_source = SDR_CLOCK_INTERNAL;

	sprintf(trx_gain_filename,"/OpenBTS/db%d/sdr_tx_gain.txt",transceiver_index);
	
	tx_gain_from_file = readTxGainFromFile(trx_gain_filename);

	double rate = 6.25e+06;

	StartParams.sample_rate_num[0] = rate;

	StartParams.sample_rate_den[0] = 1;
	StartParams.tx_bandwidth[0] = 3840000;
	StartParams.rx_bandwidth[0] = 3840000;

	StartParams.clock_source = SDR_CLOCK_INTERNAL;

	/* complex float32 */
	StartParams.rx_sample_fmt = SDR_SAMPLE_FMT_CF32;
	StartParams.tx_sample_fmt = SDR_SAMPLE_FMT_CF32;
	/* choose best format fitting the bandwidth */
	StartParams.rx_sample_hw_fmt = SDR_SAMPLE_HW_FMT_AUTO;
	StartParams.tx_sample_hw_fmt = SDR_SAMPLE_HW_FMT_AUTO;
	StartParams.rx_channel_count = 1;
	StartParams.tx_channel_count = 1;
	StartParams.rx_freq[0] = 19700e5;
	StartParams.tx_freq[0] = 21600e5;
	StartParams.rx_gain[0] = 20;
	StartParams.tx_gain[0] = -50;
	StartParams.rx_antenna[0] = SDR_RX_ANTENNA_RX;
	StartParams.rf_port_count = 1;
	StartParams.tx_port_channel_count[0] = 1;
	StartParams.rx_port_channel_count[0] = 1;

	/* if != 0, set a custom DMA buffer configuration. Otherwise the default is 150 buffers per 10 ms */
	StartParams.dma_buffer_count = dma_buffer_count;
	StartParams.dma_buffer_len = dma_buffer_len;


	size_t buf_len = SAMPLE_BUF_SZ / sizeof(uint32_t);
	rx_buffer = new SampleBuffer(buf_len, rx_rate);
	ts_offset = -10;


	started = false;
	LOG(INFO) << "PCIESDR open success";
	
	return true;

out_close:
	LOG(INFO) << "PCIESDR closing";
	msdr_close(device);
	device = NULL;
	return true;
}

bool PCIeSDRDevice::flush_recv(size_t num_pkts)
{
	
	unsigned int chan = 0;
	static sample_t samples[PSAMPLES_NUM];
	static sample_t *psamples;
	int64_t timestamp_tmp;
	int expect_smpls = sizeof(samples) / sizeof(samples[0]);
	int rc;
	MultiSDRReadMetadata md;
    LOG(INFO) << "PCIESDRDevice flush";

	psamples = &samples[0];

	while ((rc = msdr_read(device, &timestamp_tmp, (void**)&psamples, expect_smpls, chan, &md)) > 1) {
		if (rc < (int)expect_smpls)
			break;
	}
	if (rc < 0)
		return false;

	ts_initial = (TIMESTAMP)timestamp_tmp + rc;

	LOG(INFO) << "PCIESDRDevice flush done";

	return true;
}

bool PCIeSDRDevice::restart()
{
	LOG(INFO) << "restart";

	return true;
}

bool PCIeSDRDevice::start()
{
	LOG(INFO) << "start";
	global_start_time = std::chrono::steady_clock::now();
	waiting_tx_gain = true;
	
	SDRStats stats;
	int res;

	LOG(INFO) << "starting PCIESDR...";

	if (started) {
		LOG(ERR) << "Device already started";
		return false;
	}

	LOG(INFO) << "starting PCIESDR..., sample rate:" << actualSampleRate;
	LOG(INFO) << "***************************************" ;
	LOG(INFO) << "PCIESDR clock source:" << StartParams.clock_source;
	LOG(INFO) << "PCIESDR sync source:" << StartParams.sync_source;
	LOG(INFO) << "PCIESDR sample_rate_num:" << StartParams.sample_rate_num[0];
	LOG(INFO) << "PCIESDR sample_rate_den:" << StartParams.sample_rate_den[0];
	LOG(INFO) << "PCIESDR rx_sample_fmt:" << StartParams.rx_sample_fmt;
	LOG(INFO) << "PCIESDR tx_sample_fmt:" << StartParams.tx_sample_fmt;
	LOG(INFO) << "PCIESDR rx_sample_hw_fmt:" << StartParams.rx_sample_hw_fmt;
	LOG(INFO) << "PCIESDR tx_sample_hw_fmt:" << StartParams.tx_sample_hw_fmt;
	LOG(INFO) << "PCIESDR rx_channel_count:" << StartParams.rx_channel_count;
	LOG(INFO) << "PCIESDR tx_channel_count:" << StartParams.tx_channel_count;
	LOG(INFO) << "PCIESDR rx_freq:" << StartParams.rx_freq[0];
	LOG(INFO) << "PCIESDR tx_freq:" << StartParams.tx_freq[0];
	LOG(INFO) << "PCIESDR rx_gain:" << StartParams.rx_gain[0];
	LOG(INFO) << "PCIESDR tx_gain:" << StartParams.tx_gain[0];
	LOG(INFO) << "PCIESDR rx_antenna:" << StartParams.rx_antenna;
	LOG(INFO) << "PCIESDR rf_port_count:" << StartParams.rf_port_count;
	LOG(INFO) << "PCIESDR rx_bandwidth:" << StartParams.rx_bandwidth[0];
	LOG(INFO) << "PCIESDR tx_bandwidth:" << StartParams.tx_bandwidth[0];
	LOG(INFO) << "PCIESDR tx_port_channel_count:" << StartParams.tx_port_channel_count[0];
	LOG(INFO) << "PCIESDR rx_port_channel_count:" << StartParams.rx_port_channel_count[0];
	LOG(INFO) << "PCIESDR dma_buffer_count:" << StartParams.dma_buffer_count;
	LOG(INFO) << "PCIESDR dma_buffer_len:" << StartParams.dma_buffer_len;
	LOG(INFO) << "PCIESDR rx_latency:" << StartParams.rx_latency;
	LOG(INFO) << "PCIESDR config_script:" << StartParams.config_script;
	LOG(INFO) << "PCIESDR config_script_params:" << StartParams.config_script_params;
	LOG(INFO) << "PCIESDR tx_delay:" << StartParams.tx_delay;
	LOG(INFO) << "PCIESDR rx_delay:" << StartParams.rx_delay;
	LOG(INFO) << "PCIESDR sdr_count:" << StartParams.sdr_count;
	LOG(INFO) << "PCIESDR SpecialStartParams." << StartParams.spt;
	LOG(INFO) << "***************************************" ;


	res = msdr_set_start_params(device, &StartParams, sizeof(*&StartParams));
	if (res) {
		LOG(ERR) << "msdr_set_start_params failed:"<< res;
		return false;
	}

	res = msdr_start(device);
	if (res) {
		LOG(ERR) << "msdr_start failed:"<< res;
		return false;
	}
	
	
	msdr_set_tx_gain(device, 0, 0);


	LOG(INFO) << "After msdr_start(...)";
	res = msdr_get_stats(device, &stats);
	if (res != 0) {
		LOG(ERR) << "PCIESDRDevice start: get_stats failed:" << res;
	} else {
		tx_underflow = stats.tx_underflow_count;
		rx_overflow  = stats.rx_overflow_count;
	}
	flush_recv(10);
	LOG(INFO) << "After Flush Recv:";

	started = true;
	return true;
}

bool PCIeSDRDevice::stop()
{
	LOG(INFO) << "stop";
	if (device)
	{
	  msdr_close(device);
	  device = NULL;
	}
	return true;
}

void PCIeSDRDevice::setPriority()
{
	LOG(INFO) << "setPriority";
	return;
}

int PCIeSDRDevice::readSamples(short *buf, int len, bool *overrun,
			    long long timestamp, bool *underrun, unsigned *RSSI)
{
	int rc, num_smpls, expect_smpls;
	ssize_t avail_smpls;
	TIMESTAMP expect_timestamp;
	static sample_t samples[PSAMPLES_NUM];
	static sample_t *psamples;
	int64_t timestamp_tmp;
	MultiSDRReadMetadata md;
	md.timeout_ms = 1000;
	unsigned int chan = 0;

	if (waiting_tx_gain)
		memset(&samples, 0, sizeof(samples));

	if (!started)
	{
		LOG(ALERT) << "Device not started during readSamples:" ;
		return -1;

	}

	if (len > (int)(sizeof(samples) / sizeof(*samples))) {
		LOG(ERR) << "Sample buffer:" << (sizeof(samples) / sizeof(*samples))
			          << " is smaller than len:" << len;
		LOG(ALERT) << "Sample buffer:" << (sizeof(samples) / sizeof(*samples))
					  << " is smaller than len:" << len;
		return -1;
	}

	*overrun = false;
	*underrun = false;

	/* Align read time sample timing with respect to transmit clock */
	timestamp += ts_offset;
	/* Check that timestamp is valid */

	rc = rx_buffer->avail_smpls(timestamp);
	if (rc < 0) {
		LOG(ALERT) << "rc < 0 ";
		LOG(ERR) << rx_buffer->str_code(rc);
		LOG(ERR) << rx_buffer->str_status(timestamp);
		return 0;
	}

	/* Receive samples from HW until we have enough */
	while ((avail_smpls = rx_buffer->avail_smpls(timestamp)) < len) {
		thread_enable_cancel(false);
		expect_smpls = len - avail_smpls;
		expect_smpls = expect_smpls > (int)dma_buffer_len ? expect_smpls : (int)dma_buffer_len;
		expect_timestamp = timestamp + avail_smpls;
		timestamp_tmp = 0;
		psamples = &samples[0];

		num_smpls = msdr_read(device, &timestamp_tmp, (void**)&psamples, expect_smpls, chan, &md);
		thread_enable_cancel(true);

		rx_pkt_cnt++;
		if (num_smpls < 0) {
			LOG(ERR) << "PCIESDR readSamples msdr_read failed num_smpls " << num_smpls
					<< " device: " << device
					<< " expect_smpls: " << expect_smpls
					<< ", expTs:" << expect_timestamp << " got " << timestamp_tmp;
			LOG(ERR) << "Device receive timed out (" << rc
					<< " vs exp " << len << ").";
			return -1;
		}

		msdr_convert_cf32_to_ci16(buf, (float *)psamples, num_smpls);

		if (expect_smpls != num_smpls) {
			LOG(ALERT)<< "Unexpected recv buffer len: expect "
								<< expect_smpls << " got " << num_smpls
								<< ", diff=" << expect_smpls - num_smpls
								<< ", expTs:" << expect_timestamp << " got " << timestamp_tmp;
		}

		if (expect_timestamp != (TIMESTAMP)timestamp_tmp) {
			LOG(ERR) << "Unexpected recv buffer timestamp: expect "
								<< expect_timestamp << " got " << timestamp_tmp
								<< ", diff=" << timestamp_tmp - expect_timestamp;
		}
		rc = rx_buffer->write(buf, num_smpls, (TIMESTAMP)timestamp_tmp);
		if (rc < 0) {
			if (rc != SampleBuffer::ERROR_OVERFLOW) {
				return 0;
			}
		}
	}

	    /* We have enough samples */
		rc = rx_buffer->read(buf, len, timestamp);
		if ((rc < 0) || (rc != len)) {
			LOG(ERR) << rx_buffer->str_code(rc) << ". "
				                << rx_buffer->str_status(timestamp)
				                << ", (len=" << len << ")";
			return 0;
		}

	return len;
}

int PCIeSDRDevice::writeSamples(short *buf, int len,
			     bool *underrun, long long ts)
{
	int rc = 0;
	unsigned int i;
	static sample_t samples[PSAMPLES_NUM];
	static sample_t *psamples;
	int64_t hw_time;
	int64_t timestamp_tmp;
	SDRStats stats;
	int chan = 0;
	MultiSDRWriteMetadata md;

	*underrun = false;
	if (!started)
		return -1;
    
    int samplesToWrite = len;  
    while(samplesToWrite > PSAMPLES_NUM) {
        samplesToWrite -= 1250;
		LOG(ALERT) << "writeSamples overrun " << len;
    }
    
    if (waiting_tx_gain)
    {
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - global_start_time);
      if (elapsed.count() > WAIT_TX_GAIN_TIME_SEC)
      {
        LOG(INFO) << "MSDR set_tx_gain(...): " << tx_gain_from_file;
        waiting_tx_gain = false;
        msdr_set_tx_gain(device, 0, tx_gain_from_file);
      }
    }
    
    timestamp_tmp = ts; // - ts_offset; /* Shift Tx time by offset */
    
    psamples = &samples[0];

	for (int i=0; i< samplesToWrite*2; i++)
	{
		buf[i] = buf[i]*4;
	}

	msdr_convert_ci16_to_cf32((float*)psamples, buf, samplesToWrite);
	thread_enable_cancel(false);
	rc = msdr_write(device, timestamp_tmp, (const void**)&psamples, samplesToWrite, chan, &md);

	thread_enable_cancel(true);

	if (rc != (unsigned) samplesToWrite) {
		LOG(ALERT) << "Device send timed out";
	}

	if (rc != samplesToWrite) {
		LOG(ALERT) << "PCIESDR writeSamples: Device send timed out rc:" << rc
						<< " timestamp" << timestamp_tmp << " samplesToWrite:" << samplesToWrite << " hwtime:" << hw_time;
		LOG(ALERT) << "PCIESDR: Device Tx timed out (" << rc << " vs exp " << samplesToWrite << ").";
		return -1;
	}
	if (msdr_get_stats(device, &stats)) {
		LOG(ALERT) << "PCIESDR: get_stats failed:" << rc;
	} else if (stats.tx_underflow_count > tx_underflow) {
		tx_underflow = stats.tx_underflow_count;
		LOG(ALERT)<< "tx_underflow_count:" << stats.tx_underflow_count
						<< " rx_overflow_count:" << stats.rx_overflow_count;
		*underrun = true;
	}

	if (hw_time > timestamp_tmp) {
		LOG(ALERT) << "PCIESDR: tx underrun ts_tmp:" << timestamp_tmp << " ts:" << ts
						<< " hwts:" << hw_time;
		*underrun = true;
	}
	return rc;
}

bool PCIeSDRDevice::setTxFreq(double wFreq)
{
	LOG(INFO) << "setTxFreq";
    StartParams.tx_freq[0] = wFreq;

	return true;
}

bool PCIeSDRDevice::setRxFreq(double wFreq)
{
	LOG(ALERT) << "setRxFreq";
	StartParams.rx_freq[0] = wFreq;
	return true;
}

bool PCIeSDRDevice::recv_async_msg()
{
	LOG(ALERT) << "recv_async_msg";
	return true;
}
