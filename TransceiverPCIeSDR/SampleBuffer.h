/*
 * Based on TransceiverUHD/SampleBuffer.h
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

#ifndef UHD_BUFFER_H
#define UHD_BUFFER_H

#include <string>
#include <complex>

/*
 * Sample Buffer
 *
 * Allows reading and writing of timed samples using sample ticks or UHD
 * timespec values.
 */

class SampleBuffer {
public:
	SampleBuffer(int len, double rate);
	~SampleBuffer();

	/* Return number of samples available for a given ts */
	int avail_smpls(long long ts) const;
	int read(void *buf, int len, long long ts);
	int write(void *buf, int len, long long ts);

	/* Return formatted string describing internal buffer state */
	std::string str_status(long long ts) const;

	/* Formatted error code string */
	static std::string str_code(int code);

	enum err_code {
		ERROR_TIMESTAMP = -1,
		ERROR_READ = -2,
		ERROR_WRITE = -3,
		ERROR_OVERFLOW = -4
	};

private:
	std::complex<short> *data;
	int len;
	double clock_rate;

	long long time_start;
	long long time_end;
	long long data_start;
	long long data_end;
};

#endif /* UHD_BUFFER_H */
