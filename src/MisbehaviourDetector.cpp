#include "MisbehaviourDetector.h"

#include <iostream>

bool MisbehaviourDetector::processMessage(etsiDecoder::etsiDecodedData_t decoded_data, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id) {
	bool retval = false;
	int MB_CODE=0;
	uint64_t placeholderVehicle=1;

	if (decoded_data.type==etsiDecoder::ETSI_DECODED_CAM || decoded_data.type==etsiDecoder::ETSI_DECODED_CAM_NOGN) {
		std::cout <<"Detection on CAM" <<std::endl;
	} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_CPM || decoded_data.type==etsiDecoder::ETSI_DECODED_CPM_NOGN) {
		std::cout <<"Detection on CPM" <<std::endl;
	} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_VAM || decoded_data.type==etsiDecoder::ETSI_DECODED_VAM_NOGN) {
		std::cout <<"Detection on VAM" <<std::endl;
	} else {
		std::cerr << "Warning! Message type not supported!" << std::endl;
		return retval;
	}

	return retval;
};

void MisbehaviourDetector::notifyOpTermination(uint64_t stationID) {
	m_already_reported_mutex.lock();
	m_already_reported.remove(stationID);
	m_already_reported_mutex.unlock();
}