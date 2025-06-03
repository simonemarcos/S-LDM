#include "asn_utils.h"
#include <chrono>
#include <cmath>

long compute_timestampIts () {
	/* To get millisec since  2004-01-01T00:00:00:000Z */
	auto time = std::chrono::system_clock::now(); // get the current time
	auto since_epoch = time.time_since_epoch(); // get the duration since epoch
	auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch); // convert it in millisecond since epoch

	long elapsed_since_2004 = millis.count() - TIME_SHIFT; // in TIME_SHIFT we saved the millisec from epoch to 2004-01-01
	return elapsed_since_2004;
}

double haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
	// 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
	return 12742000.0*asin(sqrt(sin(DEG_2_RAD_ASN_UTILS(lat_b-lat_a)/2)*sin(DEG_2_RAD_ASN_UTILS(lat_b-lat_a)/2)+cos(DEG_2_RAD_ASN_UTILS(lat_a))*cos(DEG_2_RAD_ASN_UTILS(lat_b))*sin(DEG_2_RAD_ASN_UTILS(lon_b-lon_a)/2)*sin(DEG_2_RAD_ASN_UTILS(lon_b-lon_a)/2)));
}

uint8_t
setByteMask(uint8_t mask) {
	uint8_t out = 0;

	for (int i = 0; i < 8; i++) {
		out <<= 1;
		if (mask & 1)
			out |= 1;
		mask >>= 1;
	}
	return out;
}

uint8_t
setByteMask(uint16_t mask, unsigned int i) {
	uint8_t  out = 0;
	uint8_t in = (mask >> (i*8)) & 0x000000FF;

	for (int j = 0; j < 8; j++) {
		out <<= 1;
		if (in & 1)
			out |= 1;
		in >>= 1;
	}
	return out;
}

uint8_t
setByteMask(uint32_t mask, unsigned int i) {
	uint8_t  out = 0;
	uint8_t in = (mask >> (i*8)) & 0x000000FF;

	for (int j = 0; j< 8; j++) {
		out <<= 1;
		if (in & 1)
			out |= 1;
		in >>= 1;
	}
	return out;
}

uint8_t
getFromMask(uint8_t mask) {
	uint8_t out = 0;

	for (int i = 0; i < 8; i++) {
		out <<= 1;
		if (mask & 1)
			out |= 1;
		mask >>= 1;
	}

	return out;
}
