#include "MisbehaviourDetector.h"

#include <iostream>

uint64_t MisbehaviourDetector::processMessage(etsiDecoder::etsiDecodedData_t decoded_data,ldmmap::vehicleData_t vehdata, std::vector<ldmmap::vehicleData_t> PO_vec) {
	uint64_t MB_CODE=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	if (decoded_data.type==etsiDecoder::ETSI_DECODED_CAM || decoded_data.type==etsiDecoder::ETSI_DECODED_CAM_NOGN) {

		//checks to be made here with decoded data
		// MB_CODE=detectionFunction(...);
		MB_CODE=0;
		MB_CODE|=checkCAMFreq(vehdata);

		// Avoid reporting the same vehicle multiple times
		// in future the check should be on the cause of report
		// if different another report can and should be issued
		if (MB_CODE) {
			// report to be created here
			m_already_reported_mutex.lock();
			if(std::find(m_already_reported.begin(), m_already_reported.end(), vehdata.stationID) == m_already_reported.end()) {
				//only mark as reported without sending the actual report for now
				m_already_reported.push_back(vehdata.stationID);
			} else {
				
			}
			m_already_reported_mutex.unlock();
		}
		return MB_CODE;

	} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_CPM || decoded_data.type==etsiDecoder::ETSI_DECODED_CPM_NOGN) {

		//checks to be made here with decoded data
		// MB_CODE=detectionFunction(...);
		MB_CODE=0;

		// Avoid reporting the same vehicle multiple times
		// in future the check should be on the cause of report
		// if different another report can and should be issued
		if (MB_CODE) {
			// report to be created here
			m_already_reported_mutex.lock();
			//using first element (at least one present) to retrieve CPM sender StationID
			if(std::find(m_already_reported.begin(), m_already_reported.end(), PO_vec.front().perceivedBy) == m_already_reported.end()) {
				//only mark as reported without sending the actual report for now
				m_already_reported.push_back(PO_vec.front().perceivedBy);
			} else {

			}
			m_already_reported_mutex.unlock();
		}
		return MB_CODE;

	} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_VAM || decoded_data.type==etsiDecoder::ETSI_DECODED_VAM_NOGN) {
		
		//checks to be made here with decoded data
		// MB_CODE=detectionFunction(...);
		MB_CODE=0;

		// Avoid reporting the same vehicle multiple times
		// in future the check should be on the cause of report
		// if different another report can and should be issued
		if (MB_CODE) {
			// report to be created here
			m_already_reported_mutex.lock();
			if(std::find(m_already_reported.begin(), m_already_reported.end(), vehdata.stationID) == m_already_reported.end()) {
				//only mark as reported without sending the actual report for now
				m_already_reported.push_back(vehdata.stationID);
			} else {
				
			}
			m_already_reported_mutex.unlock();
		}
		return MB_CODE;

	}

	return MB_CODE;
};

void MisbehaviourDetector::notifyOpTermination(uint64_t stationID) {
	m_already_reported_mutex.lock();
	m_already_reported.remove(stationID);
	m_already_reported_mutex.unlock();
}