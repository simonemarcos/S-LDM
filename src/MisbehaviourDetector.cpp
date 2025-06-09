#include "MisbehaviourDetector.h"

#include <iostream>
#include <cmath>
#include "INIReader.h"

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

void MisbehaviourDetector::Init() {

	int stationTypes[]={
		ldmmap::StationType_LDM_bus,
		ldmmap::StationType_LDM_cyclist,
		ldmmap::StationType_LDM_detectedPassengerCar,
		ldmmap::StationType_LDM_detectedPedestrian,
		ldmmap::StationType_LDM_detectedTruck,
		ldmmap::StationType_LDM_heavyTruck,
		ldmmap::StationType_LDM_lightTruck,
		ldmmap::StationType_LDM_moped,
		ldmmap::StationType_LDM_motorcycle,
		ldmmap::StationType_LDM_passengerCar,
		ldmmap::StationType_LDM_pedestrian,
		ldmmap::StationType_LDM_roadSideUnit,
		ldmmap::StationType_LDM_specialVehicles,
		ldmmap::StationType_LDM_trailer,
		ldmmap::StationType_LDM_tram,
	};

	std::map<int,std::string> stationTypeNames={
		{ldmmap::StationType_LDM_bus,"BUS"},
		{ldmmap::StationType_LDM_cyclist,"CYCLIST"},
		{ldmmap::StationType_LDM_detectedPassengerCar,"PASSENGER_CAR"},
		{ldmmap::StationType_LDM_detectedPedestrian,"PEDESTRIAN"},
		{ldmmap::StationType_LDM_detectedTruck,"TRUCK"},
		{ldmmap::StationType_LDM_heavyTruck,"HEAVY_TRUCK"},
		{ldmmap::StationType_LDM_lightTruck,"LIGHT_TRUCK"},
		{ldmmap::StationType_LDM_moped,"MOPED"},
		{ldmmap::StationType_LDM_motorcycle,"MOTORCYCLE"},
		{ldmmap::StationType_LDM_passengerCar,"PASSENGER_CAR"},
		{ldmmap::StationType_LDM_pedestrian,"PEDESTRIAN"},
		{ldmmap::StationType_LDM_roadSideUnit,"RSU"},
		{ldmmap::StationType_LDM_specialVehicles,"SPECIAL_VEHICLES"},
		{ldmmap::StationType_LDM_trailer,"TRAILER"},
		{ldmmap::StationType_LDM_tram,"TRAM"},
	};
	
	INIReader reader("MBDConfig.ini");

	if (reader.ParseError()!=0) {
		std::cout <<"INI Error: " <<reader.ParseError() <<std::endl;
	}

	// MBD options if present
	m_opts.useHaversineDistance=reader.GetBoolean("OPTIONS","UseHaversine",false);
	m_opts.tolerance=reader.GetInteger("OPTIONS","Tolerance",10);
	
	// Default thresholds to be used if not specified in MBDConfig.ini
	double maxSpeedGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxSpeed",380);
	double maxAccelerationGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxAcceleration",20);
	double maxBrakingGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxBraking",50);
	double maxCurvatureGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxCurvature",50);
	double maxYawRateGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxYawRate",50);

	double maxJerkGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxJerk",6);

	// Initialization per stationType
	for (int stationType:stationTypes) {
		averages[stationType].speed_ms=500;
		deviations[stationType].speed_ms=50;

		std::string section=stationTypeNames[stationType]+"_THRESHOLDS";

		double maxSpeed=reader.GetInteger(section,"MaxSpeed",maxSpeedGeneral)/3.6; // in meters/second
		maxSpeeds.insert(std::pair<int,double>(stationType,maxSpeed));

		double maxAcceleration=reader.GetInteger(section,"MaxAcceleration",maxAccelerationGeneral);
		maxAccelerations.insert(std::pair<int,double>(stationType,maxAcceleration));
		
		double maxBraking=reader.GetInteger(section,"MaxBraking",maxBrakingGeneral);
		maxBrakings.insert(std::pair<int,double>(stationType,maxBraking));

		double maxCurvature=reader.GetInteger(section,"MaxCurvature",maxCurvatureGeneral);
		maxCurvatures.insert(std::pair<int,double>(stationType,maxCurvature));
		
		double maxYawRate=reader.GetInteger(section,"MaxYawRate",maxYawRateGeneral);
		maxYawRates.insert(std::pair<int,double>(stationType,maxYawRate));

		double maxJerk=reader.GetInteger(section,"MaxJerk",maxJerkGeneral);
		maxJerks.insert(std::pair<int,double>(stationType,maxJerk));
	}
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

	// ------- CLASS 1 CHECKS -------
	{
	// ------- PLAUSIBLE MAX SPEED CHECK -------

	if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
		if (vehdata.speed_ms*3.6>maxSpeeds[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_SPEED_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_SPEED_IMP);
	}
	
	// ------- DIRECTION INCONSISTENT WITH SIGNED SPEED -------

	if (vehdata.driveDirection!=ldmmap::e_DataUnavailableValue::driveDirection) {
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			if (vehdata.driveDirection==DriveDirection_backward && vehdata.speed_ms*3.6>30) {
				MB_CODE|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
			}
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
	}

	// ------- PLAUSIBLE MAX ACCELERATION CHECK -------

	if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
		if (vehdata.longitudinalAcceleration>maxAccelerations[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_ACCELERATION_IMP);
	}

	// ------- PLAUSIBLE MAX CURVATURE CHECK -------

	if (vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
		if (vehdata.curvature>maxCurvatures[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_CURVATURE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_CURVATURE_IMP);
	}

	// ------- PLAUSIBLE MAX YAW RATE CHECK -------

	if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
		if (vehdata.yawRate>maxYawRates[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(MB_YAW_RATE_IMP);
	}
	}

	// ------- CLASS 2 CHECKS -------
	if (lastMessagePresent) {

		// ------- BEACON FREQUENCY CHECK -------

		double messageDeltaTime=(vehdata.gnTimestamp-lastMessage.gnTimestamp)/1000.0; // in seconds
		if (messageDeltaTime<0) {
			messageDeltaTime+=429496.7296; // divided by 1000 to be in seconds
		}
		if (messageDeltaTime<0.1) {
			MB_CODE|=MB_CODE_CONV(MB_BREACON_FREQ_INC);
		}
		
		// ------- POSITION CHANGE SPEED CHECK -------

		const double radiansFactor=M_PI/180;
		const double dLat = (vehdata.lat - lastMessage.lat) * radiansFactor;
		const double dLon = (vehdata.lon - lastMessage.lon) * radiansFactor;
		const double earthRadius = 6371000; //in meters
		double messagePositionDistance;
		if (m_opts.useHaversineDistance) {
			double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lastMessage.lat*radiansFactor) * cos(vehdata.lat*radiansFactor);
			double c = 2 * asin(sqrt(a));
			messagePositionDistance=earthRadius*c;
		} else {
			messagePositionDistance=earthRadius*sqrt(pow(dLat, 2) + pow(dLon*cos(lastMessage.lat*radiansFactor), 2));
		}
		double messagePositionSpeed=messagePositionDistance/messageDeltaTime;
		// Average speed needed to travel the "message distance" is implausible
		if (messagePositionSpeed>maxSpeeds[vehdata.stationType]) {
			// MB_CODE TO BE DECIDED
		}
		double averageSpeed=(vehdata.speed_ms+lastMessage.speed_ms)/2.0;
		// Calculated average speed doesn't match with average speed of the CAMs
		if (messagePositionSpeed>(1+m_opts.tolerance)*averageSpeed || messagePositionSpeed<(1-m_opts.tolerance)*averageSpeed) {
			MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_INC);
		}

		// ------- POSITION CHANGE HEADING CHECK -------

		double messageHeading=fmod((atan2(dLon,dLat)/radiansFactor)+360,360);
		double averageHeading=(vehdata.heading+lastMessage.heading)/2.0;
		// adjust for "left side" map heading
		if (vehdata.heading>180 || lastMessage.heading>180) {
			averageHeading+=180;
		}
		// Calculated average heading doesn't match with average heading of the CAMs
		if (messageHeading>(1+m_opts.tolerance)*averageHeading || messageHeading<(1-m_opts.tolerance)*averageHeading) {
			MB_CODE|=MB_CODE_CONV(MB_POSITION_HEADING_INC);
		}

		// ------- HEADING CHANGE SPEED CHECK -------

		// ?

		// ------- HEADING CHANGE YAW RATE CHECK -------

		double messageHeadingYawRate=vehdata.heading-lastMessage.heading/messageDeltaTime; // in degrees/second
		if (messageHeadingYawRate>maxYawRates[vehdata.stationType]) {
			// MB_CODE TO BE DECIDED
		}
		double averageYawRate=(vehdata.yawRate+lastMessage.yawRate)/2.0;
		if (messageHeadingYawRate>(1+m_opts.tolerance)*averageYawRate || messageHeadingYawRate<(1-m_opts.tolerance)*averageYawRate) {
			MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_INC);
		}

		// ------- SPEED CHANGE ACCELERATION CHECK -------

		double messageSpeedAcceleration=(vehdata.speed_ms-lastMessage.speed_ms)/messageDeltaTime;
		// Average acceleration needed to reach message speed is implausible
		if (messageSpeedAcceleration>0) {
			if (messageSpeedAcceleration>maxAccelerations[vehdata.stationType]) {
				// MB_CODE TO BE DECIDED
			}
		} else {
			if (-messageSpeedAcceleration>maxBrakings[vehdata.stationType]) {
				// MB_CODE TO BE DECIDED
			}
		}
		double averageAcceleration=(vehdata.longitudinalAcceleration+lastMessage.longitudinalAcceleration)/2.0;
		// Calculated average acceleration doesn't match with average acceleration of the CAMs
		if (messageSpeedAcceleration>(1+m_opts.tolerance)*averageAcceleration || messageSpeedAcceleration<(1-m_opts.tolerance)*averageSpeed) {
			MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_INC);
		}

		// ------- POSITION + HEADING DRIVE DIRECTION CHECK -------

		// considering this as driving forward
		if (averageHeading-90<messageHeading<averageHeading+90) {
			if (lastMessage.driveDirection==DriveDirection_backward) {
				MB_CODE|=MB_CODE_CONV(MB_POS_AND_HEADING_DIRECTION_INC);
			}
		} else {
			if (lastMessage.driveDirection==DriveDirection_forward) {
				MB_CODE|=MB_CODE_CONV(MB_POS_AND_HEADING_DIRECTION_INC);
			}
		}

		// ------- LENGTH WIDTH CHANGE -------

		if (vehdata.vehicleLength.isAvailable() && vehdata.vehicleWidth.isAvailable()) {
			if (vehdata.vehicleLength.getData()!=lastMessage.vehicleLength.getData() || vehdata.vehicleWidth.getData()!=lastMessage.vehicleWidth.getData()) {
				MB_CODE|=MB_CODE_CONV(MB_LENGTH_WIDTH_INC);
			}
		} else {
			unavailables|=MB_CODE_CONV(MB_LENGTH_WIDTH_INC);
		}

		// ------- ACCELERATION CHECK -------

		double messageAccelerationChange=vehdata.longitudinalAcceleration-lastMessage.longitudinalAcceleration;
		if (messageAccelerationChange>maxJerks[vehdata.stationType] || -messageAccelerationChange<maxJerks[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_INC);
		}
	}
	
	return MB_CODE;
}