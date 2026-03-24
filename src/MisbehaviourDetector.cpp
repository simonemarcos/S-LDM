#include "MisbehaviourDetector.h"

#include <iostream>
#include <cmath>
#include "INIReader.h"
#include "utils.h"
#include "SequenceOf.hpp"

#include "curl/curl.h"
#include "pcap.h"

const std::map<int,std::string> misbehaviourStringsStation={
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_SPEED_IMP,"Speed value implausible"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_DIRECTION_SPEED_IMP,"Drive direction inconsistent with speed"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_ACCELERATION_IMP,"Acceleration value implausible"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_CURVATURE_IMP,"Curvature value implausible"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_YAW_RATE_IMP,"Yaw Rate value implausible"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_BEACON_FREQ_INC,"Beacon frequency inconsistency (too high)"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_POSITION_SPEED_INC,"Position change inconsistent with average speed"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_POSITION_HEADING_INC,"Position change inconsistent with average heading"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_HEADING_SPEED_INC,"Heading change inconsistent with speed"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_HEADING_YAW_RATE_INC,"Heading change inconsistent with average yaw rate"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_SPEED_ACCELERATION_INC,"Speed change inconsistent with average acceleration"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_POS_AND_HEADING_DIRECTION_INC,"Position and heading change inconsistent with drive direction"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_LENGTH_WIDTH_INC,"Length/Width inconsistency"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_ACCELERATION_INC,"Acceleration change implausible (inconsistent with vehicle type)"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_CURVATURE_SPEED_INC,"Curvature change inconsistent with speed"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_CURVATURE_HEADING_INC,"Curvature change inconsistent with heading change"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_CURVATURE_YAW_RATE_INC,"Curvature change inconsistent with yaw rate"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_YAW_RATE_SPEED_INC,"Yaw rate change inconsistent with speed"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_YAW_RATE_CURVATURE_INC,"Yaw rate change inconsistent with curvature"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_POSITION_SPEED_IMP,"Position change implausible (calculated speed inconsistent with vehicle type)"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_HEADING_YAW_RATE_IMP,"Heading change implausible (calculated yaw rate inconsistent with vehicle type)"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_SPEED_ACCELERATION_IMP,"Speed change implausible (calculated acceleration inconsistent with vehicle type)"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_NOT_ON_ROAD,"Position not on road"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_INSIDE_BUILDING,"Position overlaps with building"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_HEADING_NOT_FOLLOWING_ROAD,"Heading inconsistent with road heading"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_SPEED_OVER_ROAD_LIMIT,"Speed inconsistent with road plausible speed (over speed limit)"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_MOVEMENT_CONTROL,"Acceleration inconsistent with movement control field"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_ENVIRONMENT,"Environment field inconsistent with position"},
	{MisbehaviourDetector::mbdMisbehaviourCode_e::MB_BEACON_FREQ_LOW,"Beacon frequency too low (timestamps too far in time)"},
};

const std::map<int,std::string> misbehaviourStringsEvent={
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_DISTANCE_DENM_CAM,"Distance between DENM and CAM too high"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_ROAD_TYPE,"Road type in location container is not 'non-urban'"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_EVENT_HISTORY_INC,"Event history from last DENM doesn't match with new DENM"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_UNLIKELY_STATISTICS,"Unlikely event based on statistics"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_UNLIKELY_NEARBY_VEHICLES,"Unlikely event based on nearby vehicles (density and average speed)"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_REPORTER_SPEED,"Reporting station CAM speed too high"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_AHEAD_VEH_SPEED,"Speed of vehicles ahead of reporting station too high"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_PREDICTED_PATH_INC,"Predicted path"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_REPORTER_SLOW_DOWN,"Reporter not slowing down"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_SURROUNDING_VEH_SPEED_LDM,"LDM Vehicles speed exceeds threshold"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_SURROUNDING_VEH_SPEED_CAM,"CAM Vehicles speed exceeds threshold"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_CAM_SPEED,"Speed in CAM of stationary vehicle is not 0"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_POSITION_IMP,"Event position is not plausible (not close to road)"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_BUS_TRAM_STOP,"Event position doesn't match with bus/tram stop location"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED,"Surrounding vehicles behaviours check (have to slow down)"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY,"Surrounding vehicles behaviour check (heading and path history)"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_STATION_TYPE,"Station type in DENM is not 'specialVehicles'"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_STATION_TYPE_REPORTER_CAM,"Station type in reporters CAM is not 'specialVehicles'"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_LIGHTBAR_ACTIVE_CAM,"Lightbar is not activated in emergency container of reporters CAM"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_IRC_VEHICLE_MASS,"Vehicle mass is not plausible for station type"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_IRC_EVENT_SPEED_INC,"Event speed inconsistent with reporter CAM speed"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_IRC_EVENT_HEADING_INC,"Event position heading inconsistent with CAM heading"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_IRC_BEHAVIOUR_ACCELERATION,"Reporter behaviour check (has to slow down)"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_WEATHER_INFO_INC,"Inconsistency with weather info"},
	{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_WEATHER_UNLIKELY_STATISTICS,"Unlikely weather based on statistics"},
};

uint64_t MisbehaviourDetector::processCAM(proton::binary message_bin, ldmmap::vehicleData_t vehdata, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {

	uint64_t MB_CODE=0, unavailables=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	//if (!certificateData.digest.empty()) {m_certStore_ptr->insert_or_assign(certificateData.digest,certificateData);}
	// first check on security, for now the check is done but doesn't discard the packet if failed
	switch (sec_retval) {
		case Security::SECURITY_NO_SEC:
			// no security so do nothing here
			break;
		case Security::SECURITY_VALID_CERTIFICATE:
			m_certStore_ptr->insert_or_assign(certificateData.digest,certificateData);
			break;
		case Security::SECURITY_VERIFICATION_FAILED:
			// left here for possible future changes
			// at the moment this returnvalue makes geonet and then etsidecoder to return an error
			// meaning the message is discarded before reaching here
			break;
		case Security::SECURITY_INVALID_CERTIFICATE:
			// certificate verification failed
			break;
		case Security::SECURITY_DIGEST:
			switch(m_certStore_ptr->isValid(certificateData.digest)) {
				case e_DigestValid_retval::DIGEST_OK:
					break;
				case e_DigestValid_retval::DIGEST_EXPIRED:
					break;
				case e_DigestValid_retval::DIGEST_NOT_FOUND:
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}

	//checks to be made here with decoded data
	// MB_CODE=detectionFunction(...);
	MB_CODE=0;
	MB_CODE=individualCAMchecks(vehdata,unavailables);
	
	// if there are misbehaviours, update the logs
	if (MB_CODE) {
		if (msgNumber%10==1 || msgNumber%10==2) { // update logs every 10 misbehaviours
			fflush(log_csv);

			rewind(log_summary);
			ftruncate(fileno(log_summary),0);
			fprintf(log_summary,"CAM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				if (MB_CODE&(1<<p.first)) {
					misbehavioursCAM[p.first]++;
				}
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCAM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"VAM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursVAM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"CPM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCPM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"DENM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsEvent) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
			}
			fflush(log_summary);
		} else { // else only update the counters
			for (auto p:misbehaviourStringsStation) {
				if (MB_CODE&(1<<p.first)) {
					misbehavioursCAM[p.first]++;
				}
			}
		}

		pcap_dumper_t *pcap_log=pcap_dump_open_append(pcap_open_dead(DLT_EN10MB,1<<16),"./evidence.pcap");
		uint64_t timestamp;
		pcap_pkthdr pcap_hdr;
		uint8_t *message_bin_buf;
		
		u_char *binMessage=(u_char *) malloc(message_bin.size()+14);
		binMessage[0]=0xff;
		binMessage[1]=0xff;
		binMessage[2]=0xff;
		binMessage[3]=0xff;
		binMessage[4]=0xff;
		binMessage[5]=0xff;
		binMessage[6]=0x00;
		binMessage[7]=0x00;
		binMessage[8]=0x00;
		binMessage[9]=0x00;
		binMessage[10]=0x00;
		binMessage[11]=0x00;
		binMessage[12]=0x89;
		binMessage[13]=0x47;
		message_bin_buf=message_bin.data();
		pcap_hdr.caplen=message_bin.size()+14;
		pcap_hdr.len=pcap_hdr.caplen;
		timestamp=vehdata.gnTimestamp;
		pcap_hdr.ts.tv_sec=floor(timestamp/1000);
		pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;
		for (int i=0;i<message_bin.size();i++) {
			binMessage[i+14]=(u_char) message_bin_buf[i];
		}
		pcap_dump((u_char *) pcap_log, &pcap_hdr, &binMessage[0]);
		free(binMessage);
		msgNumber++;
		// for (int i=0;i<message_bin.size();i++) {
		// 	std::printf("%02X ",message_bin_buf[i]);
		// }

		if (MB_CODE >= 1<<mbdMisbehaviourCode_e::MB_BEACON_FREQ_INC) {
			proton::binary last_message_bin=m_lastBinMessageCache.at(vehdata.stationID);
			u_char *lastBinMessage=(u_char *) malloc(last_message_bin.size()+14);
			lastBinMessage[0]=0xff; lastBinMessage[1]=0xff;
			lastBinMessage[2]=0xff; lastBinMessage[3]=0xff;
			lastBinMessage[4]=0xff; lastBinMessage[5]=0xff;
			lastBinMessage[6]=0x00; lastBinMessage[7]=0x00;
			lastBinMessage[8]=0x00; lastBinMessage[9]=0x00;
			lastBinMessage[10]=0x00; lastBinMessage[11]=0x00;
			lastBinMessage[12]=0x89; lastBinMessage[13]=0x47;
			// class 2 misbehaviours detected, write last message in evidence
			message_bin_buf=last_message_bin.data();
			pcap_hdr.caplen=last_message_bin.size()+14;
			pcap_hdr.len=pcap_hdr.caplen;
			timestamp=m_lastMessageCache.at(vehdata.stationID).gnTimestamp;
			pcap_hdr.ts.tv_sec=floor(timestamp/1000);
			pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;

			for (int i=0;i<last_message_bin.size();i++) {
				lastBinMessage[i+14]=(u_char) message_bin_buf[i];
			}
			pcap_dump((u_char *) pcap_log, &pcap_hdr, &lastBinMessage[0]);
			free(lastBinMessage);
			msgNumber++;
		}

		pcap_dump_close(pcap_log);
	}

	// Avoid reporting the same vehicle multiple times
	// in future the check should be on the cause of report
	// if different another report can and should be issued
	if (MB_CODE) {
		// report to be created here
		m_already_reported_mutex.lock();
		if(std::find(m_already_reported.begin(), m_already_reported.end(), vehdata.stationID) == m_already_reported.end()) {
			//only mark as reported without sending the actual report for now
			m_already_reported.insert(vehdata.stationID);
		} else {
			
		}
		m_already_reported_mutex.unlock();
	} else {
	}

	m_lastMessageCache.insert_or_assign(vehdata.stationID,vehdata);
	m_lastBinMessageCache.insert_or_assign(vehdata.stationID,message_bin);
	return MB_CODE;
}

uint64_t MisbehaviourDetector::processVAM(proton::binary message_bin, ldmmap::vehicleData_t vehdata, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {

	uint64_t MB_CODE=0, unavailables=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	//checks to be made here with decoded data
	// MB_CODE=detectionFunction(...);
	MB_CODE=0;
	MB_CODE=individualVAMchecks(vehdata,unavailables);
	
	if (MB_CODE) {
		if (msgNumber%10==1 || msgNumber%10==2) { // update logs every 10 misbehaviours
			fflush(log_csv);

			rewind(log_summary);
			ftruncate(fileno(log_summary),0);
			fprintf(log_summary,"CAM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCAM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"VAM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				if (MB_CODE&(1<<p.first)) {
					misbehavioursVAM[p.first]++;
				}
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursVAM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"CPM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCPM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"DENM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsEvent) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
			}
			fflush(log_summary);
		} else { // else only update the counters
			for (auto p:misbehaviourStringsStation) {
				if (MB_CODE&(1<<p.first)) {
					misbehavioursVAM[p.first]++;
				}
			}
		}
	}

	// Avoid reporting the same vehicle multiple times
	// in future the check should be on the cause of report
	// if different another report can and should be issued
	if (MB_CODE) {
		// report to be created here
		m_already_reported_mutex.lock();
		if(std::find(m_already_reported.begin(), m_already_reported.end(), vehdata.stationID) == m_already_reported.end()) {
			//only mark as reported without sending the actual report for now
			m_already_reported.insert(vehdata.stationID);
		} else {
			
		}
		m_already_reported_mutex.unlock();
	} else {
	}
	m_lastMessageCache.insert_or_assign(vehdata.stationID,vehdata);
	return MB_CODE;
}

uint64_t MisbehaviourDetector::processCPM(proton::binary message_bin, std::vector<ldmmap::vehicleData_t> PO_vec, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {

	uint64_t MB_CODE=0, unavailables=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	//checks to be made here with decoded data
	// MB_CODE=detectionFunction(...);
	MB_CODE=0;
	MB_CODE=individualCPMchecks(PO_vec,unavailables);
	
	// Avoid reporting the same vehicle multiple times
	// in future the check should be on the cause of report
	// if different another report can and should be issued
	if (MB_CODE) {
		// report to be created here
		m_already_reported_mutex.lock();
		//using first element (at least one present) to retrieve CPM sender StationID
		if(std::find(m_already_reported.begin(), m_already_reported.end(), PO_vec.front().perceivedBy) == m_already_reported.end()) {
			//only mark as reported without sending the actual report for now
			m_already_reported.insert(PO_vec.front().perceivedBy);
		} else {
			
		}
		m_already_reported_mutex.unlock();
	} else {
	}
	for (auto PO:PO_vec) {
		m_lastMessageCache.insert_or_assign(PO.stationID,PO);
	}
	
	if (MB_CODE) {
		if (msgNumber%10==1 || msgNumber%10==2) { // update logs every 10 misbehaviours
			fflush(log_csv);

			rewind(log_summary);
			ftruncate(fileno(log_summary),0);
			fprintf(log_summary,"CAM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCAM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"VAM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursVAM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"CPM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsStation) {
				if (MB_CODE&(1<<p.first)) {
					misbehavioursCPM[p.first]++;
				}
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCPM[p.first],p.second.c_str());
			}
			fprintf(log_summary,"DENM MISBEHAVIOURS:\n");
			for (auto p:misbehaviourStringsEvent) {
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
			}
			fflush(log_summary);
		} else { // else only update the counters
			for (auto p:misbehaviourStringsStation) {
				if (MB_CODE&(1<<p.first)) {
					misbehavioursCPM[p.first]++;
				}
			}
		}
	}

	return MB_CODE;
}

void MisbehaviourDetector::notifyOpTermination(uint64_t stationID) {
	m_already_reported_mutex.lock();
	m_already_reported.erase(stationID);
	m_already_reported_mutex.unlock();
}

void sigintHandler(int sig_num) {
	fflush(NULL);
    printf("\nGraceful termination.\n");
    exit(0);
} 

void MisbehaviourDetector::Init(double minlat, double minlon, double maxlat, double maxlon) {
	
	log_csv=fopen("misbehaviours_log.csv","w");
	log_summary=fopen("misbehaviours_summary.txt","w");
	signal(SIGINT, sigintHandler);

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
		ldmmap::StationType_LDM_lightVruVehicle,
		ldmmap::StationType_LDM_animal,
		ldmmap::StationType_LDM_agricultural,
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
		{ldmmap::StationType_LDM_lightVruVehicle,"LIGHT_VRU_VEHICLE"},
		{ldmmap::StationType_LDM_animal,"ANIMAL"},
		{ldmmap::StationType_LDM_agricultural,"AGRICULTURAL"},
	};
	
	INIReader reader("MBDConfig.ini");

	if (reader.ParseError()!=0) {
		std::cout <<"INI Error: " <<reader.ParseError() <<std::endl;
	}

	int max_sector_size=reader.GetInteger("OPTIONS","MaxSectorSize",250);
	m_osmStore=new OSMStore(minlat, minlon, maxlat, maxlon, max_sector_size);

	// MBD options if present
	m_opts.useHaversineDistance=reader.GetBoolean("OPTIONS","UseHaversine",false);
	m_opts.tolerance=reader.GetInteger("OPTIONS","Tolerance",10);
	m_opts.maxTimeForConsecutive=reader.GetReal("OPTIONS","MaxTimeForConsecutive",1.0);
	m_opts.ignoreSecurity=reader.GetBoolean("OPTIONS","IgnoreSecurity",false);
	m_opts.cpmToleranceMultiplier=reader.GetReal("OPTIONS","ToleranceMultiplierCPM",1.5);
	m_opts.weatherAPIKey=reader.GetString("OPTIONS","weatherAPIKey","");
	weatherTimestamp=get_timestamp_s()-12600; // since there's no weather information at the start, the weatherTimestamp is set to 3.5 hours before server start allowing for a first bypass of the time constraint on next api call
	
	// Default thresholds to be used if not specified in MBDConfig.ini
	double maxSpeedGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxSpeed",380);
	double maxAccelerationGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxAcceleration",20);
	double maxBrakingGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxBraking",-50);
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

	misbehavioursCAM.resize(64);
	misbehavioursVAM.resize(64);
	misbehavioursCPM.resize(64);
	misbehavioursDENM.resize(64);
	msgNumber=1;
	pcap_dumper_t *dump=pcap_dump_open(pcap_open_dead(DLT_EN10MB,1<<16),"./evidence.pcap");
	pcap_dump_close(dump);
}

uint64_t MisbehaviourDetector::individualCAMchecks(ldmmap::vehicleData_t vehdata, uint64_t &unavailables, double tolMult) {
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
		if (vehdata.speed_ms>maxSpeeds[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",MB_SPEED_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,maxSpeeds[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_SPEED_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_SPEED);
	}
	
	// ------- DIRECTION INCONSISTENT WITH SIGNED SPEED -------

	if (vehdata.driveDirection!=ldmmap::e_DataUnavailableValue::driveDirection) {
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			if (vehdata.driveDirection==DriveDirection_backward && vehdata.speed_ms>(30/3.6)) {
				fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",MB_DIRECTION_SPEED_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,vehdata.driveDirection);
				MB_CODE|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
			}
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_DRIVE_DIRECTION);
	}

	// ------- PLAUSIBLE MAX ACCELERATION CHECK -------

	if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
		if (vehdata.longitudinalAcceleration>=0) {
			if (vehdata.longitudinalAcceleration>maxAccelerations[vehdata.stationType]) {
				fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",MB_ACCELERATION_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,vehdata.longitudinalAcceleration,maxAccelerations[vehdata.stationType]);
				MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
			}
		} else {
			if (vehdata.longitudinalAcceleration<maxBrakings[vehdata.stationType]) {
				fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",MB_ACCELERATION_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,vehdata.longitudinalAcceleration,maxBrakings[vehdata.stationType]);
				MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
			}
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_ACCELERATION);
	}

	// ------- PLAUSIBLE MAX CURVATURE CHECK -------

	if (vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
		if (vehdata.curvature>maxCurvatures[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",MB_CURVATURE_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,vehdata.curvature,maxCurvatures[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_CURVATURE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_CURVATURE);
	}

	// ------- PLAUSIBLE MAX YAW RATE CHECK -------

	if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
		if (vehdata.yawRate>maxYawRates[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",MB_YAW_RATE_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,vehdata.yawRate,maxYawRates[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_YAW_RATE);
	}
	}

	// ------- CLASS 2 CHECKS -------
	if (lastMessagePresent) {

		// ------- BEACON FREQUENCY CHECK -------

		double messageDeltaTime=(vehdata.camTimestamp-lastMessage.camTimestamp)/1000.0; // in seconds
		if (messageDeltaTime<0) {
			// messageDeltaTime+=429496.7296; // for gnTimestamp divided by 1000 to be in seconds
			messageDeltaTime+=65.536; // for camTimestamp divided by 1000 to be in seconds
		}
		if (messageDeltaTime>m_opts.maxTimeForConsecutive) {
			// messages too far in time
			MB_CODE|=MB_CODE_CONV(MB_BEACON_FREQ_LOW);
			return MB_CODE;
		}
		if (messageDeltaTime<0.095) {
			fprintf(log_csv,"%d,%d,%lu,%lu,%f,%li\n",MB_BEACON_FREQ_INC,msgNumber,vehdata.camTimestamp,vehdata.stationID,messageDeltaTime,lastMessage.camTimestamp);
			MB_CODE|=MB_CODE_CONV(MB_BEACON_FREQ_INC);
		}

		// ------- POSITION CHANGE SPEED CHECK -------

		const double radiansFactor=M_PI/180;
		const double dLat = (vehdata.lat - lastMessage.lat) * radiansFactor;
		const double dLon = (vehdata.lon - lastMessage.lon) * radiansFactor;
		const double earthRadius = 6371000; //in meters
		double averageSpeed;
		double messageHeadingYawRate;
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed && lastMessage.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			double messagePositionDistance;
			if (m_opts.useHaversineDistance) {
				double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lastMessage.lat*radiansFactor) * cos(vehdata.lat*radiansFactor);
				double c = 2 * asin(sqrt(a));
				messagePositionDistance=earthRadius*c;
			} else {
				messagePositionDistance=earthRadius*sqrt(pow(dLat, 2) + pow(dLon*cos(lastMessage.lat*radiansFactor), 2));
			}
			const double messagePositionSpeed=messagePositionDistance/messageDeltaTime;
			// Average speed needed to travel the "message distance" is implausible
			if (messagePositionSpeed>maxSpeeds[vehdata.stationType]) {
				fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f,%f\n",
					MB_POSITION_SPEED_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messagePositionDistance,messagePositionSpeed,maxSpeeds[vehdata.stationType]);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_IMP);
			}
			averageSpeed=(vehdata.speed_ms+lastMessage.speed_ms)/2.0;
			// Calculated average speed doesn't match with average speed of the CAMs
			if (messagePositionSpeed>averageSpeed*(1+m_opts.tolerance*tolMult) || messagePositionSpeed<averageSpeed*(1-m_opts.tolerance*tolMult)) {
				fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f,%f\n",
					MB_POSITION_SPEED_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messagePositionDistance,messagePositionSpeed,averageSpeed);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_INC);
			}

			// ------- SPEED CHANGE ACCELERATION CHECK -------
			if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration && lastMessage.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
				const double messageSpeedAcceleration=(vehdata.speed_ms-lastMessage.speed_ms)/messageDeltaTime;
				const double averageAcceleration=(vehdata.longitudinalAcceleration+lastMessage.longitudinalAcceleration)/2.0;
				if (messageSpeedAcceleration>0) {
					// Average acceleration needed to reach message speed is implausible
					if (messageSpeedAcceleration>maxAccelerations[vehdata.stationType]) {
						fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
							MB_SPEED_ACCELERATION_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,maxAccelerations[vehdata.stationType]);
						MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
					}
	
					// Calculated average acceleration doesn't match with average acceleration of the CAMs
					if (messageSpeedAcceleration>averageAcceleration*(1+m_opts.tolerance*tolMult) || messageSpeedAcceleration<averageSpeed*(1-m_opts.tolerance*tolMult)) {
						if (abs(messageSpeedAcceleration-averageAcceleration)>5) {
							fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
								MB_SPEED_ACCELERATION_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,averageAcceleration);
							MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_INC);
						}
					}
				} else {
					// Average acceleration needed to reach message speed is implausible
					if (messageSpeedAcceleration<maxBrakings[vehdata.stationType]) {
						MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
					}
	
					// Calculated average acceleration doesn't match with average acceleration of the CAMs
					if (messageSpeedAcceleration<averageAcceleration*(1+m_opts.tolerance*tolMult) || messageSpeedAcceleration>averageSpeed*(1-m_opts.tolerance*tolMult)) {
						if (abs(messageSpeedAcceleration-averageAcceleration)>5) {
							fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
								MB_SPEED_ACCELERATION_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,averageAcceleration);
							MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_INC);
						}
					}
				}
			}
		}

		// ------- POSITION CHANGE HEADING CHECK -------
		if (vehdata.heading!=ldmmap::e_DataUnavailableValue::heading && lastMessage.heading!=ldmmap::e_DataUnavailableValue::heading) {
			const double messageHeading=fmod((atan2(dLon,dLat)/radiansFactor)+360,360);
			double averageHeading=(vehdata.heading+lastMessage.heading)/2.0;
			// adjust for "left side" map heading
			if (vehdata.heading>180 || lastMessage.heading>180) {
				averageHeading+=180;
			}
			// Calculated average heading doesn't match with average heading of the CAMs
			if (messageHeading>averageHeading*(1+m_opts.tolerance*tolMult) || messageHeading<averageHeading*(1-m_opts.tolerance*tolMult)) {
				fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",
					MB_POSITION_HEADING_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageHeading,averageHeading);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_HEADING_INC);
			}

			// ------- HEADING CHANGE YAW RATE CHECK -------

			if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate && lastMessage.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
				double deltaHeading=vehdata.heading-lastMessage.heading;
				if (deltaHeading>180) {
					deltaHeading=360-deltaHeading;
				} else if (deltaHeading<-180) {
					deltaHeading=-360-deltaHeading;
				}
				messageHeadingYawRate=deltaHeading/messageDeltaTime;
				messageHeadingYawRate=(fmod(vehdata.heading-lastMessage.heading+180,360)-180)/messageDeltaTime; // in degrees/second
				if (messageHeadingYawRate>maxYawRates[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
						MB_HEADING_YAW_RATE_IMP,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,maxYawRates[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_IMP);
				}
				const double averageYawRate=(vehdata.yawRate+lastMessage.yawRate)/2.0;
				if (messageHeadingYawRate>averageYawRate*(1+m_opts.tolerance*tolMult) || messageHeadingYawRate<averageYawRate*(1-m_opts.tolerance*tolMult)) {
					if (abs(messageHeadingYawRate-averageYawRate)>25) {
						fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
							MB_HEADING_YAW_RATE_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,averageYawRate);
						MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_INC);
					}
				}
			}

			// ------- POSITION + HEADING DRIVE DIRECTION CHECK -------

			// considering this as driving forward
			if (averageHeading-90<messageHeading<averageHeading+90) {
				if (lastMessage.driveDirection==DriveDirection_backward) {
					fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",
						MB_POS_AND_HEADING_DIRECTION_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeading,lastMessage.driveDirection);
					MB_CODE|=MB_CODE_CONV(MB_POS_AND_HEADING_DIRECTION_INC);
				}
			} else {
				if (lastMessage.driveDirection==DriveDirection_forward) {
					fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f\n",
						MB_POS_AND_HEADING_DIRECTION_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeading,lastMessage.driveDirection);
					MB_CODE|=MB_CODE_CONV(MB_POS_AND_HEADING_DIRECTION_INC);
				}
			}
		}

		// ------- LENGTH WIDTH CHANGE -------

		if (vehdata.vehicleLength.isAvailable() && lastMessage.vehicleLength.isAvailable()) {
			if (vehdata.vehicleWidth.isAvailable() && lastMessage.vehicleWidth.isAvailable()) {
				// if tolMult is to default then we are checking a CAM and sizes have to be consistent
				// otherwise it's a perceived object so the perceived size can change
				if (tolMult==1) {
					if (vehdata.vehicleLength.getData()!=lastMessage.vehicleLength.getData() || vehdata.vehicleWidth.getData()!=lastMessage.vehicleWidth.getData()) {
						fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f,%f\n",
							MB_LENGTH_WIDTH_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.vehicleLength.getData(),vehdata.vehicleWidth.getData(),lastMessage.vehicleLength.getData(),lastMessage.vehicleWidth.getData());
						MB_CODE|=MB_CODE_CONV(MB_LENGTH_WIDTH_INC);
					}
				} else {
					if (vehdata.vehicleLength.getData()>lastMessage.vehicleLength.getData()*(1+m_opts.tolerance*tolMult)
						|| vehdata.vehicleLength.getData()<lastMessage.vehicleLength.getData()*(1-m_opts.tolerance*tolMult)
						|| vehdata.vehicleWidth.getData()>lastMessage.vehicleWidth.getData()*(1+m_opts.tolerance*tolMult)
						|| vehdata.vehicleWidth.getData()<lastMessage.vehicleWidth.getData()*(1-m_opts.tolerance*tolMult)) {

						fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f,%f\n",
							MB_LENGTH_WIDTH_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.vehicleLength.getData(),vehdata.vehicleWidth.getData(),lastMessage.vehicleLength.getData(),lastMessage.vehicleWidth.getData());
						MB_CODE|=MB_CODE_CONV(MB_LENGTH_WIDTH_INC);
					}
				}
			} else {
				unavailables|=MB_CODE_CONV(UNAV_WIDTH);
			}
		} else {
			unavailables|=MB_CODE_CONV(UNAV_LENGTH);
		}

		// ------- ACCELERATION CHECK -------

		if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration && lastMessage.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
			const double messageAccelerationChange=(vehdata.longitudinalAcceleration-lastMessage.longitudinalAcceleration)/messageDeltaTime;
			if (messageAccelerationChange>0) {
				if (messageAccelerationChange>maxJerks[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
						MB_ACCELERATION_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageAccelerationChange,maxJerks[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_INC);
				}
			} else {
				if (-messageAccelerationChange>maxJerks[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
						MB_ACCELERATION_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageAccelerationChange,maxJerks[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_INC);
				}
			}
		}
		
		double curvatureRatio=ldmmap::e_DataUnavailableValue::curvature;
		if (lastMessage.curvature!=0 && lastMessage.curvature!=ldmmap::e_DataUnavailableValue::curvature && vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
			curvatureRatio=vehdata.curvature/lastMessage.curvature;
		}
		double speedRatio=ldmmap::e_DataUnavailableValue::speed;
		if (lastMessage.speed_ms!=0 && lastMessage.speed_ms!=ldmmap::e_DataUnavailableValue::speed && vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			speedRatio=vehdata.speed_ms/lastMessage.speed_ms;
		}
		double yawRateRatio=ldmmap::e_DataUnavailableValue::yawRate;
		if (lastMessage.yawRate!=0 && lastMessage.yawRate!=ldmmap::e_DataUnavailableValue::yawRate && vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
			yawRateRatio=vehdata.yawRate/lastMessage.yawRate;
		}

		if (curvatureRatio!=ldmmap::e_DataUnavailableValue::curvature && speedRatio!=ldmmap::e_DataUnavailableValue::speed && yawRateRatio!=ldmmap::e_DataUnavailableValue::yawRate) {
			const double dCurvature=vehdata.curvature-lastMessage.curvature;
			// if approximatively constant (using tolerance for now but should be a separate parameter)
			if (abs(dCurvature)<lastMessage.curvature*(m_opts.tolerance/10)) {
				
				// ------- HEADING CHANGE SPEED CHECK -------

				const double averageHeadingYawRate=averageSpeed*(vehdata.curvature+lastMessage.curvature)/2.0/10000.0;
				if (messageHeadingYawRate>averageHeadingYawRate*(1+m_opts.tolerance*tolMult) || messageHeadingYawRate>averageHeadingYawRate*(1-m_opts.tolerance*tolMult)) {
					if (messageHeadingYawRate>averageHeadingYawRate+2 || messageHeadingYawRate<averageHeadingYawRate-2) {
						fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
							MB_HEADING_SPEED_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,averageHeadingYawRate);
						MB_CODE|=MB_CODE_CONV(MB_HEADING_SPEED_INC); // DA CAMBIARE
					}
				}
				
				// ------- YAW RATE CHANGE SPEED CHECK -------

				if (yawRateRatio>speedRatio*(1+m_opts.tolerance*tolMult) || yawRateRatio<speedRatio*(1-m_opts.tolerance*tolMult)) {
					fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
						MB_YAW_RATE_SPEED_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,yawRateRatio,speedRatio);
					MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_SPEED_INC);
				}
			}

			const double dSpeed=vehdata.speed_ms-lastMessage.speed_ms;
			if (abs(dSpeed)<lastMessage.speed_ms*(m_opts.tolerance/10)) {
				// pretty much the same checks

				// ------- CURVATURE CHANGE YAW RATE CHECK -------

				if (curvatureRatio>yawRateRatio*(1+m_opts.tolerance*tolMult) || curvatureRatio<yawRateRatio*(1-m_opts.tolerance*tolMult)) {
					if (abs(curvatureRatio-yawRateRatio)>2) {
						fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
							MB_CURVATURE_YAW_RATE_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,curvatureRatio,yawRateRatio);
						MB_CODE|=MB_CODE_CONV(MB_CURVATURE_YAW_RATE_INC); // DA CAMBIARE
					}
				}
		
				// ------- YAW RATE CHANGE CURVATURE CHECK -------

				if (yawRateRatio>curvatureRatio*(1+m_opts.tolerance*tolMult) || yawRateRatio<curvatureRatio*(1-m_opts.tolerance*tolMult)) {
					if (abs(yawRateRatio-curvatureRatio)>2) {
						fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
							MB_YAW_RATE_CURVATURE_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,yawRateRatio,curvatureRatio);
						MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_CURVATURE_INC); // DA CAMBIARE
					}
				}

			}
			
			// ------- CURVATURE CHANGE SPEED CHECK -------

			const double dYawRate=vehdata.yawRate-lastMessage.yawRate;
			if (abs(dYawRate)<lastMessage.yawRate*(m_opts.tolerance/10)) {
				// inverse proportional
				if (1/curvatureRatio>speedRatio*(1+m_opts.tolerance*tolMult) || 1/curvatureRatio<speedRatio*(1-m_opts.tolerance*tolMult)) {
					fprintf(log_csv,"%d,%d,%lu,%lu,%f,%f,%f\n",
						MB_CURVATURE_SPEED_INC,msgNumber,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,curvatureRatio,speedRatio);
					MB_CODE|=MB_CODE_CONV(MB_CURVATURE_SPEED_INC);
				}
			}
		}

		if (vehdata.gnLat==lastMessage.gnLat && vehdata.gnLon==lastMessage.gnLon) {
			std::cout <<"Geonet same source position coordinates\n";
		}
		if (vehdata.gnSpeed==lastMessage.gnSpeed) {
			std::cout <<"Geonet same source position speed\n";
		}
		if (vehdata.gnHeading==lastMessage.gnHeading) {
			std::cout <<"Geonet same source position heading\n";
		}
	}

	// ------- CLASS 3 CHECKS -------
	osmium::object_id_type closestWay=m_osmStore->checkIfPointOnRoad(vehdata.lat,vehdata.lon);
	if (closestWay==-1) {
		// not on road: misbehaviour

		// check if inside a building, can trigger another misbehaviour after "not on road", (maybe can justify "not on road" if in a "driveable" building)
		if (m_osmStore->checkIfPointInBuilding(vehdata.lat,vehdata.lon)) {
			// misbehaviour
		}
	} else {
		// on road: can check the rest

		if (vehdata.heading!=ldmmap::e_DataUnavailableValue::heading) {
			if (m_osmStore->checkHeadingMatchesRoad(vehdata.heading,vehdata.lat,vehdata.lon,closestWay)) {
				// misbehaviour
			}
		}

		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			if (m_osmStore->checkSpeedOverTypeLimit(vehdata.speed_ms,closestWay)) {
				// misbehaviour
			}
		}
	}



	// DENM related checks
	
	for (auto &ev:m_pendingEvents) {
		ldmmap::eventData_t evedata=ev.second.evedata;
		if (ev.second.evedata.originatingStationID==vehdata.stationID) {
			if (get_timestamp_ns()>ev.second.endOfChecks) {
				eventDecision(ev.second);
			} else {
				if (ev.second.eventHeading==720) {
					ev.second.eventHeading=vehdata.heading;
				}

				uint64_t eveCheck=ev.second.unavailables;
				if (MB_CODE_CHECK(eveCheck,EMB_DISTANCE_DENM_CAM)) {
					if (haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon)>100) {
						// misbehaviour: messages too far
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_DISTANCE_DENM_CAM);
						ev.second.unavailables&=MB_CODE_CLEAR(EMB_DISTANCE_DENM_CAM); // clear unavailable bit to avoid checking again in next CAMs
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_REPORTER_SPEED)) {
					if (vehdata.speed_ms>8.3) {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_REPORTER_SPEED);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_REPORTER_SLOW_DOWN)) {
					if (lastMessagePresent) {
						// has to slow down to 30km/h
						if (vehdata.speed_ms>8.3 && vehdata.speed_ms>lastMessage.speed_ms) {
							ev.second.EMB_CODE|=MB_CODE_CONV(EMB_REPORTER_SLOW_DOWN);
						}
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_CAM_SPEED)) {
					if (vehdata.speed_ms>0.1) {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_CAM_SPEED);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_STATION_TYPE_REPORTER_CAM)) {
					if (vehdata.stationType!=ldmmap::StationType_LDM_specialVehicles) {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_STATION_TYPE_REPORTER_CAM);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_LIGHTBAR_ACTIVE_CAM)) {
					if (vehdata.lightBarActivated.isAvailable()) {
						if (!vehdata.lightBarActivated.getData()) {
							ev.second.EMB_CODE|=MB_CODE_CONV(EMB_LIGHTBAR_ACTIVE_CAM);
						}
					} else {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_LIGHTBAR_ACTIVE_CAM);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_IRC_EVENT_SPEED_INC)) {
					if (abs(ev.second.evedata.eventSpeed.getData()-vehdata.speed_ms)>5) {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_EVENT_SPEED_INC);
					}
					ev.second.unavailables&=MB_CODE_CLEAR(EMB_IRC_EVENT_SPEED_INC);
				}

				if (MB_CODE_CHECK(eveCheck,EMB_IRC_EVENT_HEADING_INC)) {
					if (abs(ev.second.evedata.eventPositionHeading.getData()-vehdata.heading)>10) {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_EVENT_HEADING_INC);
					}
					ev.second.unavailables&=MB_CODE_CLEAR(EMB_IRC_EVENT_HEADING_INC);
				}

				if (MB_CODE_CHECK(eveCheck,EMB_IRC_BEHAVIOUR_ACCELERATION)) {
					if (lastMessagePresent) {
						if (vehdata.speed_ms>=lastMessage.speed_ms) {
							ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_BEHAVIOUR_ACCELERATION);
						}
					}
					if (vehdata.longitudinalAcceleration>0) {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_BEHAVIOUR_ACCELERATION);
					}
					// maybe after the first emergency brake the car stops braking so this also needs to be checked only once and not for the whole period of the event
					ev.second.unavailables&=MB_CODE_CLEAR(EMB_IRC_BEHAVIOUR_ACCELERATION);
				}
			}
		} else if (MB_CODE_CHECK(ev.second.unavailables,EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED)) {
			// nested ifs to avoid the heavier distance calculation
			if (lastMessagePresent && closestWay==ev.second.eventRoad && abs(vehdata.heading-ev.second.eventHeading)<45) {
				if (haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon)<100) {
					if (vehdata.speed_ms>8.3 && vehdata.speed_ms>lastMessage.speed_ms) {
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED);
					}
				}
			}
		}
	}

	if (tolMult!=1) {
		// clean the unavailables for cpms
		unavailables&=(MB_CODE_CONV(UNAV_LATITUDE)||MB_CODE_CONV(UNAV_LONGITUDE));
	}

	return MB_CODE;
}

uint64_t MisbehaviourDetector::individualVAMchecks(ldmmap::vehicleData_t vehdata, uint64_t &unavailables, double tolMult) {
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
		if (vehdata.speed_ms>maxSpeeds[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_SPEED_IMP);
		}
	} else {
			unavailables|=MB_CODE_CONV(UNAV_SPEED);
	}

	// ------- PLAUSIBLE MAX ACCELERATION CHECK -------

	if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
		if (vehdata.longitudinalAcceleration>=0) {
			if (vehdata.longitudinalAcceleration>maxAccelerations[vehdata.stationType]) {
				MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
			}
		} else {
			if (vehdata.longitudinalAcceleration<maxBrakings[vehdata.stationType]) {
				MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
			}
		}

	} else {
		unavailables|=MB_CODE_CONV(UNAV_ACCELERATION);
	}

	// ------- PLAUSIBLE MAX CURVATURE CHECK -------

	if (vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
		if (vehdata.curvature>maxCurvatures[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_CURVATURE_IMP);
		}
	} else {
		
	}

	// ------- PLAUSIBLE MAX YAW RATE CHECK -------

	if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
		if (vehdata.yawRate>maxYawRates[vehdata.stationType]) {
			MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_IMP);
		}
	} else {
		
	}
	}

	// ------- CLASS 2 CHECKS -------
	if (lastMessagePresent) {

		// ------- BEACON FREQUENCY CHECK -------

		double messageDeltaTime=(vehdata.gnTimestamp-lastMessage.gnTimestamp)/1000.0; // in seconds
		if (messageDeltaTime<0) {
			messageDeltaTime+=429496.7296; // divided by 1000 to be in seconds
		}
		if (messageDeltaTime>m_opts.maxTimeForConsecutive) {
			// messages too far in time
		}
		if (messageDeltaTime<0.1) {
			MB_CODE|=MB_CODE_CONV(MB_BEACON_FREQ_INC);
		}

		// ------- POSITION CHANGE SPEED CHECK -------

		const double radiansFactor=M_PI/180;
		const double dLat = (vehdata.lat - lastMessage.lat) * radiansFactor;
		const double dLon = (vehdata.lon - lastMessage.lon) * radiansFactor;
		const double earthRadius = 6371000; //in meters
		double averageSpeed;
		double messageHeadingYawRate;
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed && lastMessage.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			double messagePositionDistance;
			if (m_opts.useHaversineDistance) {
				double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lastMessage.lat*radiansFactor) * cos(vehdata.lat*radiansFactor);
				double c = 2 * asin(sqrt(a));
				messagePositionDistance=earthRadius*c;
			} else {
				messagePositionDistance=earthRadius*sqrt(pow(dLat, 2) + pow(dLon*cos(lastMessage.lat*radiansFactor), 2));
			}
			const double messagePositionSpeed=messagePositionDistance/messageDeltaTime;
			// Average speed needed to travel the "message distance" is implausible
			if (messagePositionSpeed>maxSpeeds[vehdata.stationType]) {
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_IMP);
			}
			averageSpeed=(vehdata.speed_ms+lastMessage.speed_ms)/2.0;
			// Calculated average speed doesn't match with average speed of the CAMs
			if (messagePositionSpeed>averageSpeed*(1+m_opts.tolerance*tolMult) || messagePositionSpeed<averageSpeed*(1-m_opts.tolerance*tolMult)) {
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_INC);
			}

			// ------- SPEED CHANGE ACCELERATION CHECK -------

			const double messageSpeedAcceleration=(vehdata.speed_ms-lastMessage.speed_ms)/messageDeltaTime;
			// Average acceleration needed to reach message speed is implausible
			if (messageSpeedAcceleration>0) {
				if (messageSpeedAcceleration>maxAccelerations[vehdata.stationType]) {
					MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
				}
			} else {
				if (messageSpeedAcceleration<maxBrakings[vehdata.stationType]) {
					MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
				}
			}
			const double averageAcceleration=(vehdata.longitudinalAcceleration+lastMessage.longitudinalAcceleration)/2.0;
			// Calculated average acceleration doesn't match with average acceleration of the CAMs
			if (messageSpeedAcceleration>averageAcceleration*(1+m_opts.tolerance*tolMult) || messageSpeedAcceleration<averageSpeed*(1-m_opts.tolerance*tolMult)) {
				MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_INC);
			}
		}

		// ------- POSITION CHANGE HEADING CHECK -------
		if (vehdata.heading!=ldmmap::e_DataUnavailableValue::heading && lastMessage.heading!=ldmmap::e_DataUnavailableValue::heading) {
			const double messageHeading=fmod((atan2(dLon,dLat)/radiansFactor)+360,360);
			double averageHeading=(vehdata.heading+lastMessage.heading)/2.0;
			// adjust for "left side" map heading
			if (vehdata.heading>180 || lastMessage.heading>180) {
				averageHeading+=180;
			}
			// Calculated average heading doesn't match with average heading of the CAMs
			if (messageHeading>averageHeading*(1+m_opts.tolerance*tolMult) || messageHeading<averageHeading*(1-m_opts.tolerance*tolMult)) {
				if (abs(messageHeading-averageHeading)>5) {
					MB_CODE|=MB_CODE_CONV(MB_POSITION_HEADING_INC);
				}
			}

			// ------- HEADING CHANGE YAW RATE CHECK -------

			if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate && lastMessage.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
				messageHeadingYawRate=(vehdata.heading-lastMessage.heading)/messageDeltaTime; // in degrees/second
				if (messageHeadingYawRate>maxYawRates[vehdata.stationType]) {
					MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_IMP);
				}
				const double averageYawRate=(vehdata.yawRate+lastMessage.yawRate)/2.0;
				if (messageHeadingYawRate>averageYawRate*(1+m_opts.tolerance*tolMult) || messageHeadingYawRate<averageYawRate*(1-m_opts.tolerance*tolMult)) {
					MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_INC);
				}
			}
		}

		// ------- LENGTH WIDTH CHANGE -------

		if (vehdata.vruSizeClass!=VruSizeClass_unavailable && lastMessage.vruSizeClass!=VruSizeClass_unavailable) {
			if (vehdata.vruSizeClass!=lastMessage.vruSizeClass) {
				MB_CODE|=MB_CODE_CONV(MB_LENGTH_WIDTH_INC);
			}
		} else {
			
		}

		// ------- ACCELERATION CHECK -------

		if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration && lastMessage.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
			const double messageAccelerationChange=vehdata.longitudinalAcceleration-lastMessage.longitudinalAcceleration;
			if (messageAccelerationChange>maxJerks[vehdata.stationType] || -messageAccelerationChange<maxJerks[vehdata.stationType]) {
				MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_INC);
			}
		}
		
		double curvatureRatio=ldmmap::e_DataUnavailableValue::curvature;
		if (lastMessage.curvature!=0 && lastMessage.curvature!=ldmmap::e_DataUnavailableValue::curvature && vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
			curvatureRatio=vehdata.curvature/lastMessage.curvature;
		}
		double speedRatio=ldmmap::e_DataUnavailableValue::speed;
		if (lastMessage.speed_ms!=0 && lastMessage.speed_ms!=ldmmap::e_DataUnavailableValue::speed && vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			speedRatio=vehdata.speed_ms/lastMessage.speed_ms;
		}
		double yawRateRatio=ldmmap::e_DataUnavailableValue::yawRate;
		if (lastMessage.yawRate!=0 && lastMessage.yawRate!=ldmmap::e_DataUnavailableValue::yawRate && vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
			yawRateRatio=vehdata.yawRate/lastMessage.yawRate;
		}

		if (curvatureRatio!=ldmmap::e_DataUnavailableValue::curvature && speedRatio!=ldmmap::e_DataUnavailableValue::speed && yawRateRatio!=ldmmap::e_DataUnavailableValue::yawRate) {
			const double dCurvature=vehdata.curvature-lastMessage.curvature;
			// if approximatively constant (using tolerance for now but should be a separate parameter)
			if (abs(dCurvature)<lastMessage.curvature*(m_opts.tolerance/10)) {
				
				// ------- HEADING CHANGE SPEED CHECK -------

				const double averageHeadingYawRate=averageSpeed*(vehdata.curvature+lastMessage.curvature)/2.0;
				if (messageHeadingYawRate>averageHeadingYawRate*(1+m_opts.tolerance*tolMult) || messageHeadingYawRate>averageHeadingYawRate*(1-m_opts.tolerance*tolMult)) {
					MB_CODE|=MB_CODE_CONV(MB_HEADING_SPEED_INC);	
				}
				
				// ------- YAW RATE CHANGE SPEED CHECK -------

				if (yawRateRatio>speedRatio*(1+m_opts.tolerance*tolMult) || yawRateRatio<speedRatio*(1-m_opts.tolerance*tolMult)) {
					MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_SPEED_INC);
				}
			}

			const double dSpeed=vehdata.speed_ms-lastMessage.speed_ms;
			if (abs(dSpeed)<lastMessage.speed_ms*(m_opts.tolerance/10)) {
				// pretty much the same checks

				// ------- CURVATURE CHANGE YAW RATE CHECK -------

				if (curvatureRatio>yawRateRatio*(1+m_opts.tolerance*tolMult) || curvatureRatio<yawRateRatio*(1-m_opts.tolerance*tolMult)) {
					MB_CODE|=MB_CODE_CONV(MB_CURVATURE_YAW_RATE_INC);
				}
		
				// ------- YAW RATE CHANGE CURVATURE CHECK -------

				if (yawRateRatio>curvatureRatio*(1+m_opts.tolerance*tolMult) || yawRateRatio<curvatureRatio*(1-m_opts.tolerance*tolMult)) {
					MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_CURVATURE_INC);
				}

			}
			
			// ------- CURVATURE CHANGE SPEED CHECK -------

			const double dYawRate=vehdata.yawRate-lastMessage.yawRate;
			if (abs(dYawRate)<lastMessage.yawRate*(m_opts.tolerance/10)) {
				if (curvatureRatio>speedRatio*(1+m_opts.tolerance*tolMult) || curvatureRatio<speedRatio*(1-m_opts.tolerance*tolMult)) {
					MB_CODE|=MB_CODE_CONV(MB_CURVATURE_SPEED_INC);
				}
			}
		}

		if (lastMessage.vruMovementControl!=VruMovementControl_unavailable) {
			if (lastMessage.vruMovementControl==VruMovementControl_braking || lastMessage.vruMovementControl==VruMovementControl_hardBraking ||
				lastMessage.vruMovementControl==VruMovementControl_brakingAndStopPedaling || lastMessage.vruMovementControl==VruMovementControl_hardBrakingAndStopPedaling) {
				
				if (vehdata.longitudinalAcceleration>=(1+m_opts.tolerance*tolMult)*lastMessage.longitudinalAcceleration) {
					MB_CODE|=MB_CODE_CONV(MB_MOVEMENT_CONTROL);
				}
			}
		}
	}

	// ------- CLASS 3 CHECKS -------
	osmium::object_id_type closestWay=m_osmStore->checkIfPointOnRoad(vehdata.lat,vehdata.lon);
	if (closestWay==-1) {
		// not on road
		MB_CODE|=MB_CODE_CONV(MB_NOT_ON_ROAD);
		if (m_osmStore->checkIfPointInBuilding(vehdata.lat,vehdata.lon)) {
			MB_CODE|=MB_CODE_CONV(MB_INSIDE_BUILDING);
		}
		if (vehdata.vruEnvironment==VruEnvironment_onVehicleRoad) {
			MB_CODE|=MB_CODE_CONV(MB_ENVIRONMENT);
		}
	} else {
		// on road
		if (m_osmStore->checkHeadingMatchesRoad(vehdata.heading,vehdata.lat,vehdata.lon,closestWay)) {
			MB_CODE|=MB_CODE_CONV(MB_HEADING_NOT_FOLLOWING_ROAD);
		}
		if (m_osmStore->checkSpeedOverTypeLimit(vehdata.speed_ms,closestWay)) {
			MB_CODE|=MB_CODE_CONV(MB_SPEED_OVER_ROAD_LIMIT);
		}
	}

	if (tolMult!=1) {
		// clean the unavailables for cpms
		unavailables&=(MB_CODE_CONV(UNAV_LATITUDE)||MB_CODE_CONV(UNAV_LONGITUDE));
	}

	return MB_CODE;	
}

uint64_t MisbehaviourDetector::individualCPMchecks(std::vector<ldmmap::vehicleData_t> PO_vec, uint64_t &unavailables) {
	uint64_t MB_CODE=0;
	for (auto vehdata:PO_vec) {
		if (vehdata.stationType==ldmmap::e_StationTypeLDM::StationType_LDM_cyclist
			|| vehdata.stationType==ldmmap::e_StationTypeLDM::StationType_LDM_pedestrian
			|| vehdata.stationType==ldmmap::e_StationTypeLDM::StationType_LDM_agricultural
			|| vehdata.stationType==ldmmap::e_StationTypeLDM::StationType_LDM_detectedPedestrian
			|| vehdata.stationType==ldmmap::e_StationTypeLDM::StationType_LDM_lightVruVehicle
			|| vehdata.stationType==ldmmap::e_StationTypeLDM::StationType_LDM_moped
			|| vehdata.stationType==ldmmap::e_StationTypeLDM::StationType_LDM_motorcycle) {
			// VAM controls for selected stationTypes
			MB_CODE|=individualVAMchecks(vehdata,unavailables,m_opts.cpmToleranceMultiplier);
		} else {
			// CAM controls for other stationTypes
			MB_CODE|=individualCAMchecks(vehdata,unavailables,m_opts.cpmToleranceMultiplier);
		}
	}
	return MB_CODE;
}

size_t WriteCallbackMBD(void* ptr, size_t size, size_t nmemb, void* stream) {
	((std::string *)stream)->append((char *)ptr, size * nmemb);
    return size*nmemb;
}


void MisbehaviourDetector::processDENM(proton::binary message_bin, ldmmap::eventData_t evedata, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {
	bool pending;
	pendingEvent_t currentEvent;

	currentEvent.keyEvent = m_db_ptr->KEY_EVENT(evedata.eventLatitude,evedata.eventLongitude,evedata.eventElevation,evedata.eventCauseCode);
	uint64_t expiration=get_timestamp_ns()+20e9; // 20 seconds from now

	//checks to be made here with decoded data
	// MB_CODE=detectionFunction(...);

	// if the event doesn't exist in the validation buffer add it and perform individualDENMchecks
	// otherwise add the new reporter to the reporters
	if (m_pendingEvents.find(currentEvent.keyEvent)==m_pendingEvents.end()) {

		currentEvent.endOfChecks=expiration;
		currentEvent.evedata=evedata;
		currentEvent.EMB_CODE=0;
		currentEvent.unavailables=0;
		currentEvent.reporters.insert(evedata.originatingStationID);
		
		ldmmap::eventData_t lastEvent;
		bool lastEventPresent;
		ldmmap::vehicleData_t vehdata;
		bool lastMessagePresent;
		pending=false;

		uint64_t keyEvent = m_db_ptr->KEY_EVENT(evedata.eventLatitude,evedata.eventLongitude,evedata.eventElevation,evedata.eventCauseCode);
		if (m_pendingEvents.find(keyEvent)!=m_pendingEvents.end()) {
			lastEvent=m_pendingEvents.at(keyEvent).evedata;
			lastEventPresent=true;
		} else {
			lastEventPresent=false;
		}

		if (m_lastMessageCache.find(evedata.originatingStationID)!=m_lastMessageCache.end()) {
			vehdata=m_lastMessageCache.at(evedata.originatingStationID);
			lastMessagePresent=true;
			currentEvent.eventHeading=vehdata.heading;
		} else {
			lastMessagePresent=false;
			currentEvent.eventHeading=720;
		}

		currentEvent.eventRoad=m_osmStore->checkIfPointOnRoad(evedata.eventLatitude,evedata.eventLongitude,5);
		currentEvent.eventRoad;
		

		// ------- GENERAL -------

		// DENM and CAM distance check 
		if (lastMessagePresent) {
			if (haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon)>100) {
				// misbehaviour: messages too far
				fprintf(log_csv,"%d,%d,%lu,%lu,%f\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon));
				currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_DISTANCE_DENM_CAM);
			}
		} else {
			pending=true;
			// unavailables: CAM/position to perform the check in the next CAM
			currentEvent.unavailables|=MB_CODE_CONV(EMB_DISTANCE_DENM_CAM);
		}

		// traffic jam
		if (evedata.eventCauseCode==CauseCodeType_dangerousEndOfQueue || evedata.eventCauseCode==CauseCodeType_trafficCondition) {
			if (evedata.roadType.isAvailable()) {
				if (evedata.roadType.getData()!=RoadType_nonUrban_NoStructuralSeparationToOppositeLanes
				&& evedata.roadType.getData()!=RoadType_nonUrban_WithStructuralSeparationToOppositeLanes) {
					// misbehaviour: roadType not 'non-urban'
					fprintf(log_csv,"%d,%d,%lu,%lu,%d\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,evedata.roadType.getData());
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_ROAD_TYPE);
				}
			} else {
				pending=true;
				// unavailables: roadType
				currentEvent.unavailables|=MB_CODE_CONV(EMB_ROAD_TYPE);
			}

			if (lastEventPresent) {
				if (evedata.eventHistory.isAvailable() && lastEvent.eventHistory.isAvailable()) {
					// simply compare eventHistory, maybe since eventHistory appends each new event to it it should compare after truncating last element
					if (evedata.eventHistory.getData().list.array!=lastEvent.eventHistory.getData().list.array) {
						// misbehaviour: eventHistory not matching
						currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_EVENT_HISTORY_INC);
					}
					//evedata.eventHistory.getData().list.array
				} else {
					// unavailables: eventHistory
					currentEvent.unavailables|=MB_CODE_CONV(EMB_EVENT_HISTORY_INC);
				}
			}

			currentEvent.unavailables|=MB_CODE_CONV(EMB_UNLIKELY_STATISTICS);
			
			// for dangerous end of queue set the heading limit to 45 to only get vehicles in front
			// for traffic jam ahead no heading limit (360)
			int headingLimit=360;
			int speedLimit=8.3;
			int code_setter=EMB_SURROUNDING_VEH_SPEED_LDM;

			// traffic jam, dangerous end of queue
			if (evedata.eventCauseCode==CauseCodeType_dangerousEndOfQueue) {
				headingLimit=45;
				speedLimit=2;
				code_setter=EMB_AHEAD_VEH_SPEED;
				if (lastMessagePresent) {
					if (vehdata.speed_ms>8.33) {
						// misbehaviour: speed to high
						fprintf(log_csv,"%d,%d,%lu,%lu,%f\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,vehdata.speed_ms);
						currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_REPORTER_SPEED);
					}
				} else {
					pending=true;
					// unavailables: no last CAM wait for next one
					currentEvent.unavailables|=MB_CODE_CONV(EMB_REPORTER_SPEED);
				}
			}
			
			std::vector<ldmmap::LDMMap::returnedVehicleData_t> selectedVehicles;
			int sumSpeed=0;
			
			// if there is a cam from reported use it for position otherwise use event position
			if (lastMessagePresent) {
				m_db_ptr->rangeSelectVehicle(100,vehdata.lat,vehdata.lon,selectedVehicles);
			} else {
				m_db_ptr->rangeSelectVehicle(100,evedata.eventLatitude,evedata.eventLongitude,selectedVehicles);
			}
			for (auto ve:selectedVehicles) {
				ldmmap::vehicleData_t vd=ve.vehData;
				
				// ------- SPEED OF VEHICLES AROUND/AHEAD -------
				// checks speeds of vehicles in front for dangerous end of queue
				// checks speeds of all vehicles for traffic jam ahead
				// difference in heading low enough so same direction to avoid other direction vehicles

				// what if vehdata is not present?
				if (abs(vd.heading-vehdata.heading)<45) {
					const double radiansFactor=M_PI/180;
					double dLat=vd.lat-vehdata.lat;
					double dLon=vd.lon-vehdata.lon;
					// check if it's in front by calculating the heading of the vector formed between the 2 cars
					// since the vector head is on the other car this has to be in front to have a similar heading to the driving direction
					if (abs(vehdata.heading-atan2(dLon,dLat)/radiansFactor)<headingLimit) {
						if (vd.speed_ms>speedLimit) {
							// misbehaviour: above value
							fprintf(log_csv,"%d,%d,%lu,%lu,%lu,%f\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,vd.stationID,vd.speed_ms); // additional data is stationId and speed of each ahead vehicle over the limit
							currentEvent.EMB_CODE|=MB_CODE_CONV(code_setter);
						}
					}
				}

				// ------- DENSITY/SPEED STAT -------
				sumSpeed+=vd.speed_ms;
			}
			// if nearby there are less than 15 vehicles (density) or their average speed is above 50km/h (speed)
			if (selectedVehicles.size()<15 || sumSpeed/selectedVehicles.size()>13.8) {
				fprintf(log_csv,"%d,%d,%lu,%lu,%d,%f\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,selectedVehicles.size(),sumSpeed/selectedVehicles.size()); // add data density (rather number of vehicles) and avg speed
				currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_UNLIKELY_NEARBY_VEHICLES);
			}

			currentEvent.unavailables|=MB_CODE_CONV(EMB_PREDICTED_PATH_INC);
			
			// traffic jam, traffic jam ahead
			if (evedata.eventCauseCode==CauseCodeType_trafficCondition) {
				// wait for next CAMs to check speed
				pending=true;
				currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_SPEED_CAM);
			}
		}
		// stationary vehicle
		if (evedata.eventCauseCode==CauseCodeType_stationaryVehicle) {
			pending=true;
			// stopped vehicle

			// now assuming that the event comes from the stationary vehicle and not another one
			if (lastMessagePresent) {
				// not using 0 in case of small errors
				if (vehdata.speed_ms>0.1) {
					// misbehaviour: vehicle not stationary
					fprintf(log_csv,"%d,%d,%lu,%lu,%f\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,vehdata.speed_ms);
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_CAM_SPEED);
				}
			} else {
				// unavailables
				currentEvent.unavailables|=MB_CODE_CONV(EMB_CAM_SPEED);
			}

			// assuming subcausecode 0 means unavailable
			if (evedata.eventSubCauseCode==0) {
				
				if (currentEvent.eventRoad==-1) {
					// misbehaviour: position not plausible
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_POSITION_IMP);
				}
				/*	BUS/TRAM STOP CHECK
					if (evedata.eventStationType==ldmmap::StationType_LDM_bus) {
						if (!m_osmStore->checkIfPointIsStop(evedata.eventLatitude,evedata.eventLongitude,true)) {
							// misbehaviour
							currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_BUS_TRAM_STOP);
						}
					} else if (evedata.eventStationType==ldmmap::StationType_LDM_tram) {
						if (!m_osmStore->checkIfPointIsStop(evedata.eventLatitude,evedata.eventLongitude,false)) {
							// misbehaviour
							currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_BUS_TRAM_STOP);
						}
					}
				*/
			}

			// subcause==breakdown or subcause==post-crash
			if (evedata.eventSubCauseCode==2 || evedata.eventSubCauseCode==3) {
				pending=true;
				// wait for cams
				currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED);
				currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY);
			}
		}
		// emergency vehicle
		if (evedata.eventCauseCode==CauseCodeType_emergencyVehicleApproaching) {
			pending=true;

			currentEvent.unavailables|=MB_CODE_CONV(EMB_CURVATURE_CHANGE_HEADING_INC);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_CURVATURE_CHANGE_YAW_RATE_INC);
			
			if (evedata.eventStationType!=ldmmap::StationType_LDM_specialVehicles) {
				// misbehaviour
				currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_STATION_TYPE);
			}

			currentEvent.unavailables|=MB_CODE_CONV(EMB_STATION_TYPE_REPORTER_CAM);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_LIGHTBAR_ACTIVE_CAM);

			if (lastEventPresent) {
				if (evedata.eventHistory.isAvailable() && lastEvent.eventHistory.isAvailable()) {
					// simply compare eventHistory, maybe since eventHistory appends each new event to it it should compare after truncating last element
					if (evedata.eventHistory.getData().list.array!=lastEvent.eventHistory.getData().list.array) {
						// misbehaviour: eventHistory not matching
						currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_EVENT_HISTORY_INC);
					}
				} else {
					// unavailables: eventHistory
					currentEvent.unavailables|=MB_CODE_CONV(EMB_EVENT_HISTORY_INC);
				}
			}

			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY);
		}
		// exchange of IRCs
		if (evedata.eventCauseCode==CauseCodeType_collisionRisk) {
			if (evedata.vehicleMass.getData()>vehicleMasses[evedata.eventStationType]) {
				currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_IRC_VEHICLE_MASS);
			}

			if (lastMessagePresent) {
				if (abs(evedata.eventSpeed.getData()-vehdata.speed_ms)>5) {
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_IRC_EVENT_SPEED_INC);	
				}
				if (abs(evedata.eventPositionHeading.getData()-vehdata.heading)>10) {
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_IRC_EVENT_HEADING_INC);
				}
			} else {
				currentEvent.unavailables|=MB_CODE_CONV(EMB_IRC_EVENT_SPEED_INC);
				currentEvent.unavailables|=MB_CODE_CONV(EMB_IRC_EVENT_HEADING_INC);
			}
			currentEvent.unavailables|=MB_CODE_CONV(EMB_IRC_BEHAVIOUR_ACCELERATION);
		}
		//dangerous situation
		if (evedata.eventCauseCode==CauseCodeType_dangerousSituation) {
			pending=true;
			currentEvent.unavailables|=MB_CODE_CONV(EMB_CAM_SPEED);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY);
		}
		//adverse weather condition
		if (evedata.eventCauseCode==CauseCodeType_adverseWeatherCondition_Adhesion
			|| evedata.eventCauseCode==CauseCodeType_adverseWeatherCondition_ExtremeWeatherCondition
			|| evedata.eventCauseCode==CauseCodeType_adverseWeatherCondition_Precipitation
			|| evedata.eventCauseCode==CauseCodeType_adverseWeatherCondition_Visibility) {

			if (m_opts.weatherAPIKey!="") {
				if (get_timestamp_s()-weatherTimestamp>12600) {
					std::string url="https://my.meteoblue.com/packages/current?apikey="+m_opts.weatherAPIKey+"&lat="+std::to_string(evedata.eventLatitude)+"&lon="+std::to_string(evedata.eventLongitude)+"&asl="+std::to_string(evedata.eventElevation)+"&format=json";
					CURL *curl=curl_easy_init();
					curl_easy_setopt(curl,CURLOPT_URL,url.c_str());
					
					std::string buffer;
					curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallbackMBD);
					curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);
					
					weatherTimestamp=get_timestamp_s();
					CURLcode res=curl_easy_perform(curl);
					if (res!=CURLE_OK) {
						std::cout <<"Curl error: " <<res <<std::endl;
					}
					curl_easy_cleanup(curl);
		
					sscanf(&buffer[buffer.find("pictocode_detailed")+20],"\n%d,\n\"pictocode\":\n%d",&picto_detailed,&picto);
				}
				// pictogram values:
				// https://docs.meteoblue.com/en/meteo/variables/pictograms
				if (picto_detailed!=30) {
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_WEATHER_INFO_INC);
				}
			}


			// currentEvent.unavailables|=MB_CODE_CONV(EMB_WEATHER_INFO_INC);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_WEATHER_UNLIKELY_STATISTICS);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY);
			
		}
		
		// if pending add to the internal store for future checks
		if (pending) {
			m_pendingEvents.insert_or_assign(currentEvent.keyEvent,currentEvent);
		} else {
			eventDecision(currentEvent);
		}

	} else {
		m_pendingEvents.at(currentEvent.keyEvent).reporters.insert(evedata.originatingStationID);

		// concept of consensus threshold verification, if enough stations report an event it can be considered true
		int threshold=10;
		if (m_pendingEvents.at(currentEvent.keyEvent).reporters.size()>threshold) {
			currentEvent.EMB_CODE=0;
			eventDecision(currentEvent);
		}
	}
}

void MisbehaviourDetector::eventDecision(pendingEvent_t currentEvent) {

	m_pendingEvents.erase(currentEvent.keyEvent);

	ldmmap::LDMMap::LDMMap_error_t db_retval;
	ldmmap::eventData_t evedata=currentEvent.evedata;
	if (currentEvent.EMB_CODE) {
		// report to be created here
		m_already_reported_mutex.lock();
		if(std::find(m_already_reported.begin(), m_already_reported.end(), evedata.originatingStationID) == m_already_reported.end()) {
			//only mark as reported without sending the actual report for now
			m_already_reported.insert(evedata.originatingStationID);
		} else {
			
		}
		m_already_reported_mutex.unlock();

		if (currentEvent.EMB_CODE) {
			if (msgNumber%10==1 || msgNumber%10==2) { // update logs every 10 misbehaviours
				fflush(log_csv);

				rewind(log_summary);
				ftruncate(fileno(log_summary),0);
				fprintf(log_summary,"CAM MISBEHAVIOURS:\n");
				for (auto p:misbehaviourStringsStation) {
					fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCAM[p.first],p.second.c_str());
				}
				fprintf(log_summary,"VAM MISBEHAVIOURS:\n");
				for (auto p:misbehaviourStringsStation) {
					fprintf(log_summary,"%d:\t\t%s\n",misbehavioursVAM[p.first],p.second.c_str());
				}
				fprintf(log_summary,"CPM MISBEHAVIOURS:\n");
				for (auto p:misbehaviourStringsStation) {
					fprintf(log_summary,"%d:\t\t%s\n",misbehavioursCPM[p.first],p.second.c_str());
				}
				fprintf(log_summary,"DENM MISBEHAVIOURS:\n");
				for (auto p:misbehaviourStringsEvent) {
					if (currentEvent.EMB_CODE&(1<<p.first)) {
						misbehavioursDENM[p.first]++;
					}
					fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
				}
				fflush(log_summary);
			} else { // else only update the counters
				for (auto p:misbehaviourStringsEvent) {
					if (currentEvent.EMB_CODE&(1<<p.first)) {
						misbehavioursDENM[p.first]++;
					}
				}
			}
		}
	} else {
		
		ldmmap::LDMMap::returnedEventData_t retEvent;
		ldmmap::LDMMap::event_LDMMap_error_t db_everetval;
		uint64_t nearUpdateEvent_key = 0;
		uint64_t keyEvent = m_db_ptr->KEY_EVENT(evedata.eventLatitude,evedata.eventLongitude,evedata.eventElevation,evedata.eventCauseCode);
		std::cout << "[DEBUG] Updating event with eventKey: " << keyEvent << std::endl;
		if (!evedata.eventTermination.isAvailable()) {
			db_everetval = m_db_ptr->lookupAndUpdateEvent(keyEvent,evedata.eventLatitude,evedata.eventLongitude,
			evedata.eventCauseCode,evedata,retEvent, nearUpdateEvent_key);
			if (db_everetval == 6) {
				//std::cout <<"EVENT NOT FOUND" << std::endl; //For test
			} else if (db_everetval == 2 || db_everetval == 3) {
				eventMapModified.store(true);
				//std::cout << "Updated Near Event Key: " << nearUpdateEvent_key << std::endl;
			}
			//std::cout <<"Result of lookupAndUpdate = " << db_everetval << std::endl; //For test
			if (db_everetval == ldmmap::LDMMap::event_LDMMAP_ITEM_NOT_FOUND) {
				db_everetval = m_db_ptr->insertEvent(evedata,keyEvent);
				eventMapModified.store(true);
				//std::cout <<"INSERT EVENT with KEY: " <<keyEvent << std::endl; //For test
			}
		} else {
			db_everetval = m_db_ptr->removeEvent(keyEvent);
			eventMapModified.store(true);
			//std::cout <<"REMOVE EVENT with KEY: " <<keyEvent << std::endl; //For test
		}
			
		if(db_everetval!=ldmmap::LDMMap::event_LDMMAP_OK && db_retval!=ldmmap::LDMMap::event_LDMMAP_UPDATED
			&& db_retval!=ldmmap::LDMMap::event_LDMMAP_NEAR_EVENT_UPDATED  && db_retval!=ldmmap::LDMMap::event_LDMMAP_REMOVED) {
			std::cerr << "[WARNING] Operation on the database for event " <<keyEvent << "failed!" << std::endl;
		}

	}
}

void MisbehaviourDetector::cleanupPendingEvents() {
	uint64_t now = get_timestamp_ns();
    for (auto it=m_pendingEvents.cbegin();it!=m_pendingEvents.cend();) {
		// first check if the event is too old to be relevant, then check the validation period
		// may not make sense since the endOfChecks will always come before we reach the other timestamp
		if (((now/1000.0)-it->second.evedata.detectionTime)>300e6) { // 300e6 -> 5 minutes
			it=m_pendingEvents.erase(it);
		} else if(now>=it->second.endOfChecks) {
			eventDecision(it->second);
            it=m_pendingEvents.erase(it);
		} else {
            ++it;
        }
    }
}