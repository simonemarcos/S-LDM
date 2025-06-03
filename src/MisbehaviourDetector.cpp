#include "MisbehaviourDetector.h"

#include <iostream>
#include <cmath>

uint64_t MisbehaviourDetector::processMessage(etsiDecoder::etsiDecodedData_t decoded_data, ldmmap::vehicleData_t vehdata, std::vector<ldmmap::vehicleData_t> PO_vec) {

	uint64_t MB_CODE=0, unavailables;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	if (decoded_data.type==etsiDecoder::ETSI_DECODED_CAM || decoded_data.type==etsiDecoder::ETSI_DECODED_CAM_NOGN) {

		//checks to be made here with decoded data
		// MB_CODE=detectionFunction(...);
		MB_CODE=0;
		MB_CODE=individualCAMchecks(vehdata,unavailables);

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
		} else {
		}
		m_lastMessageCache.insert_or_assign(vehdata.stationID,vehdata);
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
	ldmmap::vehicleData_t lastMessage;
	bool lastMessagePresent=false;
	if (m_lastMessageCache.find(vehdata.stationID)!=m_lastMessageCache.end()) {
		lastMessagePresent=true;
		lastMessage=m_lastMessageCache[vehdata.stationID];
	}

	// to be used in dynamic road situation evaluation of average passing by vehicles speeds
	// Class 3 Topology checks
	// speed>(averages[stationType].speed_ms+2*deviations[stationType].speed_ms)

	// Class 1 checks
	{

	// Plausible max speed check
	if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
		if (vehdata.speed_ms*3.6>maxSpeeds[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_SPEED_VALUE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_SPEED_VALUE_IMP);
	}

	//if speed can be negative to mean retro
	// Implausible speed in reverse
	// if ((vehdata.direction==0 && vehdata.speed_ms*3.6<0) || (vehdata.direction==1 && vehdata.speed_ms*3.6>0)) {
	
	// Direction inconsistent with signed speed
	if (vehdata.driveDirection!=ldmmap::e_DataUnavailableValue::driveDirection) {
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			if (vehdata.driveDirection==DriveDirection_backward && vehdata.speed_ms*3.6>30) {
				MB_CODE|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
			}
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
	}

	// Plausible max acceleration check
	if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
		if (vehdata.longitudinalAcceleration>maxAccelerations[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_VALUE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_ACCELERATION_VALUE_IMP);
	}

	// Plausible max curvature check
	if (vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
		if (vehdata.curvature>maxCurvatures[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_CURVATURE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_CURVATURE_IMP);
	}

	// Plausible max curvature check
	if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
		if (vehdata.yawRate>maxYawRates[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_YAW_RATE_IMP);
	}

	}


	// Class 2 checks
	if (lastMessagePresent) {

		// in seconds?
		double deltaTime=(vehdata.gnTimestamp-lastMessage.gnTimestamp)/1000.0;
		std::cout <<"GN Timestamps: " <<vehdata.gnTimestamp <<" - " <<lastMessage.gnTimestamp <<std::endl;
		std::cout <<"Delta Time: " <<deltaTime <<std::endl;
		if (deltaTime<0) {
			deltaTime+=4294967296;
		}

		// Beacon frequency check
		if (deltaTime<0.1) {
			MB_CODE|=MB_CODE_CONV(MB_BREACON_FREQ_INC);
		}

		// Position change speed check

		const double radiansFactor=M_PI/180;
        double dLat = (vehdata.lat - lastMessage.lat) * radiansFactor;
        double dLon = (vehdata.lon - lastMessage.lon) * radiansFactor;
        double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lastMessage.lat*radiansFactor) * cos(vehdata.lat*radiansFactor);
        double rad = 6371;
        double c = 2 * asin(sqrt(a));
		double haversineDistance=rad*c*1000;
		
		
		double euclideanDistance=rad*sqrt(pow(dLat, 2) + pow(dLon*cos(lastMessage.lat*radiansFactor), 2))*1000;

		std::cout <<"Haversine: " <<haversineDistance <<std::endl;
		std::cout <<"Measured: " <<euclideanDistance <<std::endl;
		std::cout <<"Calculated: " <<lastMessage.speed_ms*deltaTime <<std::endl;

		// distance between messages too far from previous speed
		if (euclideanDistance>lastMessage.speed_ms*deltaTime*1.1 || euclideanDistance<lastMessage.speed_ms*deltaTime*0.9) {
			MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_INC);
		}

		// Position change heading check
		double heading=fmod((atan2(dLon,dLat)/radiansFactor)+360,360);

		std::cout <<"Previous: " <<lastMessage.heading <<std::endl;
		std::cout <<"Calculated: " <<heading <<std::endl;

		if (heading>lastMessage.heading*1.1 || heading<lastMessage.heading*0.9) {
			MB_CODE|=MB_CODE_CONV(MB_POSITION_HEADING_INC);
		}
	}
	
	return MB_CODE;
}