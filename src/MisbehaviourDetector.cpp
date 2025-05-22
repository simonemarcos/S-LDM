#include "MisbehaviourDetector.h"

#include <iostream>

uint64_t MisbehaviourDetector::processMessage(etsiDecoder::etsiDecodedData_t decoded_data, ldmmap::vehicleData_t vehdata, std::vector<ldmmap::vehicleData_t> PO_vec) {

	uint64_t MB_CODE=0, unavailables;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	if (decoded_data.type==etsiDecoder::ETSI_DECODED_CAM || decoded_data.type==etsiDecoder::ETSI_DECODED_CAM_NOGN) {

		//checks to be made here with decoded data
		// MB_CODE=detectionFunction(...);
		MB_CODE=0;
		MB_CODE=individualCAMchecks(vehdata,unavailables);
		
		// Esempio di check di classe 2
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

void MisbehaviourDetector::StatsInit() {
	int stationType;
	for (stationType=0;stationType<=11;stationType++) {
		averages[stationType].speed_ms=500;
		deviations[stationType].speed_ms=50;
	}
	averages[15].speed_ms=500;
	deviations[15].speed_ms=50;
	for (stationType=100;stationType<=108;stationType++) {
		averages[stationType].speed_ms=500;
		deviations[stationType].speed_ms=50;
	}
	averages[110].speed_ms=500;
	deviations[110].speed_ms=50;
	averages[115].speed_ms=500;
	deviations[115].speed_ms=50;
	averages[117].speed_ms=500;
	deviations[117].speed_ms=50;
	averages[120].speed_ms=500;
	deviations[120].speed_ms=50;

	maxSpeeds = {
		{ldmmap::StationType_LDM_bus,130},
		{ldmmap::StationType_LDM_cyclist,80},
		{ldmmap::StationType_LDM_detectedPassengerCar,180},
		{ldmmap::StationType_LDM_detectedPedestrian,20},
		{ldmmap::StationType_LDM_detectedTruck,130},
		{ldmmap::StationType_LDM_heavyTruck,110},
		{ldmmap::StationType_LDM_lightTruck,130},
		{ldmmap::StationType_LDM_moped,80},
		{ldmmap::StationType_LDM_motorcycle,200},
		{ldmmap::StationType_LDM_passengerCar,180},
		{ldmmap::StationType_LDM_pedestrian,20},
		{ldmmap::StationType_LDM_roadSideUnit,0},
		{ldmmap::StationType_LDM_specialVehicles,60},
		{ldmmap::StationType_LDM_trailer,110},
		{ldmmap::StationType_LDM_tram,60}
	};

	maxAccelerations = {
		{ldmmap::StationType_LDM_bus,130},
		{ldmmap::StationType_LDM_cyclist,80},
		{ldmmap::StationType_LDM_detectedPassengerCar,180},
		{ldmmap::StationType_LDM_detectedPedestrian,20},
		{ldmmap::StationType_LDM_detectedTruck,130},
		{ldmmap::StationType_LDM_heavyTruck,110},
		{ldmmap::StationType_LDM_lightTruck,130},
		{ldmmap::StationType_LDM_moped,80},
		{ldmmap::StationType_LDM_motorcycle,200},
		{ldmmap::StationType_LDM_passengerCar,180},
		{ldmmap::StationType_LDM_pedestrian,20},
		{ldmmap::StationType_LDM_roadSideUnit,0},
		{ldmmap::StationType_LDM_specialVehicles,60},
		{ldmmap::StationType_LDM_trailer,110},
		{ldmmap::StationType_LDM_tram,60}
	};

	maxCurvatures = {
		{ldmmap::StationType_LDM_bus,130},
		{ldmmap::StationType_LDM_cyclist,80},
		{ldmmap::StationType_LDM_detectedPassengerCar,180},
		{ldmmap::StationType_LDM_detectedPedestrian,20},
		{ldmmap::StationType_LDM_detectedTruck,130},
		{ldmmap::StationType_LDM_heavyTruck,110},
		{ldmmap::StationType_LDM_lightTruck,130},
		{ldmmap::StationType_LDM_moped,80},
		{ldmmap::StationType_LDM_motorcycle,200},
		{ldmmap::StationType_LDM_passengerCar,180},
		{ldmmap::StationType_LDM_pedestrian,20},
		{ldmmap::StationType_LDM_roadSideUnit,0},
		{ldmmap::StationType_LDM_specialVehicles,60},
		{ldmmap::StationType_LDM_trailer,110},
		{ldmmap::StationType_LDM_tram,60}
	};

	maxYawRates = {
		{ldmmap::StationType_LDM_bus,130},
		{ldmmap::StationType_LDM_cyclist,80},
		{ldmmap::StationType_LDM_detectedPassengerCar,180},
		{ldmmap::StationType_LDM_detectedPedestrian,20},
		{ldmmap::StationType_LDM_detectedTruck,130},
		{ldmmap::StationType_LDM_heavyTruck,110},
		{ldmmap::StationType_LDM_lightTruck,130},
		{ldmmap::StationType_LDM_moped,80},
		{ldmmap::StationType_LDM_motorcycle,200},
		{ldmmap::StationType_LDM_passengerCar,180},
		{ldmmap::StationType_LDM_pedestrian,20},
		{ldmmap::StationType_LDM_roadSideUnit,0},
		{ldmmap::StationType_LDM_specialVehicles,60},
		{ldmmap::StationType_LDM_trailer,110},
		{ldmmap::StationType_LDM_tram,60}
	};
}

uint64_t MisbehaviourDetector::individualCAMchecks(ldmmap::vehicleData_t vehdata, uint64_t &unavailables) {
	uint64_t MB_CODE=0;

	// to be used in dynamic road situation evaluation of average passing by vehicles speeds
	// Class 3 Topology checks
	// speed>(averages[stationType].speed_ms+2*deviations[stationType].speed_ms)

	// Plausible max speed check
	if (vehdata.speed_ms!=SpeedValue_unavailable) {
		if (vehdata.speed_ms*3.6>maxSpeeds[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_SPEED_VALUE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_SPEED_VALUE_IMP);
	}

	//if speed can be negative to mean retro
	// Implausible speed in reverse
	// if (additionalData.direction==1 && vehdata.speed*3.6>30) {

	// Direction inconsistent with signed speed
	if (vehdata.direction!=DriveDirection_unavailable) {
		if (vehdata.speed_ms!=SpeedValue_unavailable) {
			if ((vehdata.direction==0 && vehdata.speed_ms*3.6<0) || (vehdata.direction==1 && vehdata.speed_ms*3.6>0)) {
				MB_CODE|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
			}
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
	}

	// Plausible max acceleration check
	if (vehdata.longitudinalAcceleration!=LongitudinalAccelerationValue_unavailable) {
		if (vehdata.longitudinalAcceleration>maxAccelerations[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_VALUE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_ACCELERATION_VALUE_IMP);
	}

	// Plausible max curvature check
	if (vehdata.curvature!=CurvatureValue_unavailable) {
		if (vehdata.curvature>maxCurvatures[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_CURVATURE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_CURVATURE_IMP);
	}

	// Plausible max curvature check
	if (vehdata.yawRate!=YawRateValue_unavailable) {
		if (vehdata.yawRate>maxYawRates[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_YAW_RATE_IMP);
	}

	return MB_CODE;
}