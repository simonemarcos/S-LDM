#include "MisbehaviourDetector.h"

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
	//{MisbehaviourDetector::mbdEventMisbehaviourCode_e::EMB_EVENT_HISTORY_INC,"Event history from last DENM doesn't match with new DENM"},
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

pcap_dumper_t *log_pcap;
FILE *log_summary;
FILE *log_csv;

uint64_t MisbehaviourDetector::processCAM(proton::binary message_bin, ldmmap::vehicleData_t &vehdata, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {
	std::lock_guard<std::mutex> mbd_lock(m_mbd_mutex);

	uint64_t MB_CODE=0, unavailables=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	// Check for security, currently proceeds with the rest of the checks, this behaviour may change according to the reporting service needs
	if (!m_opts.ignoreSecurity) {
		switch (sec_retval) {
			case Security::SECURITY_NO_SEC:
				MB_CODE|=MB_CODE_CONV(MB_NO_SECURITY);
				break;
			case Security::SECURITY_VALID_CERTIFICATE:
				if (!certificateData.digest.empty())
					m_certStore_ptr->insert_or_assign(certificateData.digest,certificateData);
				break;
			case Security::SECURITY_VERIFICATION_FAILED:
				// left here for possible future changes
				// at the moment this returnvalue makes geonet and then etsidecoder to return an error
				// meaning the message is discarded before reaching here
				break;
			case Security::SECURITY_INVALID_CERTIFICATE:
				MB_CODE|=MB_CODE_CONV(MB_INVALID_CERTIFICATE);
				break;
			case Security::SECURITY_DIGEST:
				switch(m_certStore_ptr->isValid(certificateData.digest)) {
					case e_DigestValid_retval::DIGEST_OK:
						break;
					case e_DigestValid_retval::DIGEST_EXPIRED:
						MB_CODE|=MB_CODE_CONV(MB_DIGEST_EXPIRED);
						break;
					case e_DigestValid_retval::DIGEST_NOT_FOUND:
						MB_CODE|=MB_CODE_CONV(MB_DIGEST_NOT_FOUND);
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
	}

	MB_CODE|=individualCAMchecks(vehdata,unavailables);
	
	// if there are misbehaviours, update the logs
	if (MB_CODE) {
		uint64_t timestamp;
		pcap_pkthdr pcap_hdr;
		
		static const u_char header[] = {
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x89, 0x47
		};
		std::vector<u_char> binMessage(sizeof(header)+message_bin.size());

		std::memcpy(binMessage.data(),header,sizeof(header));

		pcap_hdr.caplen=message_bin.size()+sizeof(header);
		pcap_hdr.len=pcap_hdr.caplen;
		timestamp=vehdata.gnTimestamp;
		pcap_hdr.ts.tv_sec=floor(timestamp/1000);
		pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;

		std::memcpy(binMessage.data()+sizeof(header), message_bin.data(), message_bin.size());

		pcap_dump((u_char *) log_pcap, &pcap_hdr, binMessage.data());
		msgNumber++;

		if (MB_CODE >= 1<<mbdMisbehaviourCode_e::MB_BEACON_FREQ_INC) {
			proton::binary last_message_bin=m_lastBinMessageCache.at(vehdata.stationID);
			std::vector<u_char> lastBinMessage(sizeof(header)+last_message_bin.size());
			
			std::memcpy(lastBinMessage.data(),header,sizeof(header));
			
			// class 2 misbehaviours detected, write last message in evidence
			pcap_hdr.caplen=last_message_bin.size()+sizeof(header);
			pcap_hdr.len=pcap_hdr.caplen;
			timestamp=m_lastMessageCache.at(vehdata.stationID).gnTimestamp;
			pcap_hdr.ts.tv_sec=floor(timestamp/1000);
			pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;

			std::memcpy(lastBinMessage.data()+sizeof(header), last_message_bin.data(), last_message_bin.size());

			pcap_dump((u_char *) log_pcap, &pcap_hdr, lastBinMessage.data());
			msgNumber++;
		}

		for (auto p:misbehaviourStringsStation) {
			if (MB_CODE&(1<<p.first)) {
				misbehavioursCAM[p.first]++;
			}
		}
		if (msgNumber%10==1 || msgNumber%10==2) { // update logs every 10 misbehaviours
			pcap_dump_flush(log_pcap);
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
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
			}
			fflush(log_summary);
		}
	}

	if (!m_opts.discardOnMisbehaviour || !MB_CODE) {
		m_lastMessageCache.insert_or_assign(vehdata.stationID,vehdata);
		m_lastBinMessageCache.insert_or_assign(vehdata.stationID,message_bin);
	}
	return MB_CODE;
}

uint64_t MisbehaviourDetector::processVAM(proton::binary message_bin, ldmmap::vehicleData_t &vehdata, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {
	std::lock_guard<std::mutex> mbd_lock(m_mbd_mutex);

	uint64_t MB_CODE=0, unavailables=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	// Check for security, currently proceeds with the rest of the checks, this behaviour may change according to the reporting service needs
	if (!m_opts.ignoreSecurity) {
		switch (sec_retval) {
			case Security::SECURITY_NO_SEC:
				MB_CODE|=MB_CODE_CONV(MB_NO_SECURITY);
				break;
			case Security::SECURITY_VALID_CERTIFICATE:
				if (!certificateData.digest.empty())
					m_certStore_ptr->insert_or_assign(certificateData.digest,certificateData);
				break;
			case Security::SECURITY_VERIFICATION_FAILED:
				// left here for possible future changes
				// at the moment this returnvalue makes geonet and then etsidecoder to return an error
				// meaning the message is discarded before reaching here
				break;
			case Security::SECURITY_INVALID_CERTIFICATE:
				MB_CODE|=MB_CODE_CONV(MB_INVALID_CERTIFICATE);
				break;
			case Security::SECURITY_DIGEST:
				switch(m_certStore_ptr->isValid(certificateData.digest)) {
					case e_DigestValid_retval::DIGEST_OK:
						break;
					case e_DigestValid_retval::DIGEST_EXPIRED:
						MB_CODE|=MB_CODE_CONV(MB_DIGEST_EXPIRED);
						break;
					case e_DigestValid_retval::DIGEST_NOT_FOUND:
						MB_CODE|=MB_CODE_CONV(MB_DIGEST_NOT_FOUND);
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
	}

	MB_CODE|=individualVAMchecks(vehdata,unavailables);
	
	if (MB_CODE) {
		uint64_t timestamp;
		pcap_pkthdr pcap_hdr;
		
		static const u_char header[] = {
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x89, 0x47
		};
		std::vector<u_char> binMessage(sizeof(header)+message_bin.size());

		std::memcpy(binMessage.data(),header,sizeof(header));

		pcap_hdr.caplen=message_bin.size()+sizeof(header);
		pcap_hdr.len=pcap_hdr.caplen;
		timestamp=vehdata.gnTimestamp;
		pcap_hdr.ts.tv_sec=floor(timestamp/1000);
		pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;

		std::memcpy(binMessage.data()+sizeof(header), message_bin.data(), message_bin.size());

		pcap_dump((u_char *) log_pcap, &pcap_hdr, binMessage.data());
		msgNumber++;

		if (MB_CODE >= 1<<mbdMisbehaviourCode_e::MB_BEACON_FREQ_INC) {
			proton::binary last_message_bin=m_lastBinMessageCache.at(vehdata.stationID);
			std::vector<u_char> lastBinMessage(sizeof(header)+last_message_bin.size());
			
			std::memcpy(lastBinMessage.data(),header,sizeof(header));
			
			// class 2 misbehaviours detected, write last message in evidence
			pcap_hdr.caplen=last_message_bin.size()+sizeof(header);
			pcap_hdr.len=pcap_hdr.caplen;
			timestamp=m_lastMessageCache.at(vehdata.stationID).gnTimestamp;
			pcap_hdr.ts.tv_sec=floor(timestamp/1000);
			pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;

			std::memcpy(lastBinMessage.data()+sizeof(header), last_message_bin.data(), last_message_bin.size());

			pcap_dump((u_char *) log_pcap, &pcap_hdr, lastBinMessage.data());
			msgNumber++;
		}

		for (auto p:misbehaviourStringsStation) {
			if (MB_CODE&(1<<p.first)) {
				misbehavioursVAM[p.first]++;
			}
		}
		if (msgNumber%10==1 || msgNumber%10==2) { // update logs every 10 misbehaviours
			pcap_dump_flush(log_pcap);
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
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
			}
			fflush(log_summary);
		}
	}

	if (!m_opts.discardOnMisbehaviour || !MB_CODE) {
		m_lastMessageCache.insert_or_assign(vehdata.stationID,vehdata);
	}
	return MB_CODE;
}

uint64_t MisbehaviourDetector::processCPM(proton::binary message_bin, std::vector<ldmmap::vehicleData_t> &PO_vec, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {
	std::lock_guard<std::mutex> mbd_lock(m_mbd_mutex);

	uint64_t MB_CODE=0, unavailables=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;

	// Check for security, currently proceeds with the rest of the checks, this behaviour may change according to the reporting service needs
	if (!m_opts.ignoreSecurity) {
		switch (sec_retval) {
			case Security::SECURITY_NO_SEC:
				MB_CODE|=MB_CODE_CONV(MB_NO_SECURITY);
				break;
			case Security::SECURITY_VALID_CERTIFICATE:
				if (!certificateData.digest.empty())
					m_certStore_ptr->insert_or_assign(certificateData.digest,certificateData);
				break;
			case Security::SECURITY_VERIFICATION_FAILED:
				// left here for possible future changes
				// at the moment this returnvalue makes geonet and then etsidecoder to return an error
				// meaning the message is discarded before reaching here
				break;
			case Security::SECURITY_INVALID_CERTIFICATE:
				MB_CODE|=MB_CODE_CONV(MB_INVALID_CERTIFICATE);
				break;
			case Security::SECURITY_DIGEST:
				switch(m_certStore_ptr->isValid(certificateData.digest)) {
					case e_DigestValid_retval::DIGEST_OK:
						break;
					case e_DigestValid_retval::DIGEST_EXPIRED:
						MB_CODE|=MB_CODE_CONV(MB_DIGEST_EXPIRED);
						break;
					case e_DigestValid_retval::DIGEST_NOT_FOUND:
						MB_CODE|=MB_CODE_CONV(MB_DIGEST_NOT_FOUND);
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
	}

	MB_CODE|=individualCPMchecks(PO_vec,unavailables);
	
	if (MB_CODE) {
		uint64_t timestamp;
		pcap_pkthdr pcap_hdr;
		
		static const u_char header[] = {
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x89, 0x47
		};
		std::vector<u_char> binMessage(sizeof(header)+message_bin.size());

		std::memcpy(binMessage.data(),header,sizeof(header));

		pcap_hdr.caplen=message_bin.size()+sizeof(header);
		pcap_hdr.len=pcap_hdr.caplen;
		timestamp=PO_vec.front().gnTimestamp;
		pcap_hdr.ts.tv_sec=floor(timestamp/1000);
		pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;

		std::memcpy(binMessage.data()+sizeof(header), message_bin.data(), message_bin.size());

		pcap_dump((u_char *) log_pcap, &pcap_hdr, binMessage.data());
		msgNumber++;

		if (MB_CODE >= 1<<mbdMisbehaviourCode_e::MB_BEACON_FREQ_INC) {
			proton::binary last_message_bin=m_lastBinMessageCache.at(certificateData.stationID);
			std::vector<u_char> lastBinMessage(sizeof(header)+last_message_bin.size());
			
			std::memcpy(lastBinMessage.data(),header,sizeof(header));
			
			// class 2 misbehaviours detected, write last message in evidence
			pcap_hdr.caplen=last_message_bin.size()+sizeof(header);
			pcap_hdr.len=pcap_hdr.caplen;
			timestamp=m_lastMessageCache.at(certificateData.stationID).gnTimestamp;
			pcap_hdr.ts.tv_sec=floor(timestamp/1000);
			pcap_hdr.ts.tv_usec=(timestamp-pcap_hdr.ts.tv_sec*1000)*1000;

			std::memcpy(lastBinMessage.data()+sizeof(header), last_message_bin.data(), last_message_bin.size());

			pcap_dump((u_char *) log_pcap, &pcap_hdr, lastBinMessage.data());
			msgNumber++;
		}

		for (auto p:misbehaviourStringsStation) {
			if (MB_CODE&(1<<p.first)) {
				misbehavioursCPM[p.first]++;
			}
		}
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
				fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
			}
			fflush(log_summary);
		}
	}
	
	if (!m_opts.discardOnMisbehaviour || !MB_CODE) {
		for (auto PO:PO_vec) {
			m_lastMessageCache.insert_or_assign(PO.stationID,PO);
		}
	}
	return MB_CODE;
}

void MisbehaviourDetector::notifyOpTermination(uint64_t stationID) {
	m_already_reported_mutex.lock();
	m_already_reported.erase(stationID);
	m_already_reported_mutex.unlock();
}

void sigIntHandler(int sig) {
	fflush(log_summary);
	fflush(log_csv);
	pcap_dump_flush(log_pcap);
	exit(0);
}

void MisbehaviourDetector::Init(double minlat, double minlon, double maxlat, double maxlon) {
	
	log_csv=fopen("misbehaviours_log.csv","w");
	log_summary=fopen("misbehaviours_summary.txt","w");
	signal(SIGINT, sigIntHandler);

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
	m_opts.discardOnMisbehaviour=reader.GetBoolean("OPTIONS","DiscardOnMisbehaviour",false);

	// Default thresholds to be used if not specified in MBDConfig.ini
	double maxSpeedGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxSpeed",380);
	double maxAccelerationGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MaxAcceleration",20);
	double minAccelerationGeneral=reader.GetInteger("GENERAL_THRESHOLDS","MinAcceleration",-50);
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
		
		double minAcceleration=reader.GetInteger(section,"MinAcceleration",minAccelerationGeneral);
		minAccelerations.insert(std::pair<int,double>(stationType,minAcceleration));

		double maxCurvature=reader.GetInteger(section,"MaxCurvature",maxCurvatureGeneral);
		maxLateralAcceleration.insert(std::pair<int,double>(stationType,maxCurvature));
		
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
	log_pcap=pcap_dump_open(pcap_open_dead(DLT_EN10MB,1<<16),"./evidence.pcap");
}

uint64_t MisbehaviourDetector::individualCAMchecks(ldmmap::vehicleData_t &vehdata, uint64_t &unavailables, int msgType) {
	uint64_t MB_CODE=0;
	ldmmap::vehicleData_t lastMessage;
	bool lastMessagePresent=false;
	double tolMult=msgType==CAM?1:m_opts.cpmToleranceMultiplier;

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
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f;maxSpeed=%f\"\n",msgNumber,msgType,MB_SPEED_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,maxSpeeds[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_SPEED_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_SPEED);
	}
	
	// ------- DIRECTION INCONSISTENT WITH SIGNED SPEED -------

	if (vehdata.driveDirection!=ldmmap::e_DataUnavailableValue::driveDirection) {
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			if (vehdata.driveDirection==DriveDirection_backward && vehdata.speed_ms>(30/3.6)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f;driveDirection=%f\"\n",msgNumber,msgType,MB_DIRECTION_SPEED_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,vehdata.driveDirection);
				MB_CODE|=MB_CODE_CONV(MB_DIRECTION_SPEED_IMP);
			}
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_DRIVE_DIRECTION);
	}

	// ------- PLAUSIBLE MAX ACCELERATION CHECK -------

	if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
		//accelerating
		if (vehdata.longitudinalAcceleration>maxAccelerations[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"acceleration=%f;maxAcceleration=%f\"\n",msgNumber,msgType,MB_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.longitudinalAcceleration,maxAccelerations[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
		}
		//braking
		if (vehdata.longitudinalAcceleration<minAccelerations[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"acceleration=%f;minAcceleration=%f\"\n",msgNumber,msgType,MB_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.longitudinalAcceleration,minAccelerations[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_ACCELERATION);
	}

	// ------- PLAUSIBLE MAX LATERAL ACCELERATION CHECK (FROM CURVATURE) -------
	const double curvaturelateralAcceleration=pow(vehdata.speed_ms,2)*vehdata.curvature;
	if (vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
		if (curvaturelateralAcceleration>maxLateralAcceleration[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"lateralAcceleration=%f;maxLateralAcceleration=%f\"\n",
				msgNumber,msgType,MB_CURVATURE_IMP,vehdata.gnTimestamp,vehdata.stationID,curvaturelateralAcceleration,maxLateralAcceleration[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_CURVATURE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_CURVATURE);
	}

	// ------- PLAUSIBLE MAX YAW RATE CHECK -------

	if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
		if (vehdata.yawRate>maxYawRates[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"yawRate=%f;maxYawRate=%f\"\n",msgNumber,msgType,MB_YAW_RATE_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.yawRate,maxYawRates[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_YAW_RATE);
	}
	}


	double messageDeltaTime; // seconds
	// ------- CLASS 2 CHECKS -------
	if (lastMessagePresent) {

		// ------- BEACON FREQUENCY CHECK -------
		messageDeltaTime=(vehdata.camTimestamp-lastMessage.camTimestamp)/1000.0;// in seconds
		if (messageDeltaTime<0) {
			// messageDeltaTime+=429496.7296; // for gnTimestamp divided by 1000 to be in seconds
			messageDeltaTime+=65.536; // for camTimestamp divided by 1000 to be in seconds
		}
		if (messageDeltaTime>m_opts.maxTimeForConsecutive) {
			// Messages are too far in time for the used configuration, avoid checking any class 2
			// reuse lastMessagePresent set to false to pretend there is no last message (potentially a second variable could be used but its not needed in the current scenario)
			MB_CODE|=MB_CODE_CONV(MB_BEACON_FREQ_LOW);
			lastMessagePresent=false;
		}
		if (messageDeltaTime<0.095) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;lastTimestamp=%li\"\n",msgNumber,msgType,MB_BEACON_FREQ_INC,vehdata.camTimestamp,vehdata.stationID,messageDeltaTime,lastMessage.camTimestamp);
			MB_CODE|=MB_CODE_CONV(MB_BEACON_FREQ_INC);
			if (messageDeltaTime<=0) {
				// identical timestamps: no time base for any rate calculation
				lastMessagePresent=false;
			}
		}

	}

	// Class 2 checks only done if there is a last message and it's recent enough
	if (lastMessagePresent) {
		
		const double radiansFactor=M_PI/180;
		const double dLat = (vehdata.lat - lastMessage.lat) * radiansFactor;
		const double dLon = (vehdata.lon - lastMessage.lon) * radiansFactor;
		const double earthRadius = 6371000; //in meters
		double averageSpeed; // average of the speeds of the two messages
		double messageHeadingYawRate; // yaw rate between the two messages (variation of heading over time)
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed && lastMessage.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			
			// ------- POSITION CHANGE SPEED CHECK -------

			double messagePositionDistance; // distance between the two messages
			if (m_opts.useHaversineDistance) {
				double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lastMessage.lat*radiansFactor) * cos(vehdata.lat*radiansFactor);
				double c = 2 * asin(sqrt(a));
				messagePositionDistance=earthRadius*c;
			} else {
				messagePositionDistance=earthRadius*sqrt(pow(dLat, 2) + pow(dLon*cos(lastMessage.lat*radiansFactor), 2));
			}
			const double messagePositionSpeed=messagePositionDistance/messageDeltaTime; //

			// Average speed needed to travel the "message distance" is implausible
			if (messagePositionSpeed>maxSpeeds[vehdata.stationType]) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;messageDistance=%f;calculatedSpeed=%f;maxSpeed=%f\"\n",
					msgNumber,msgType,MB_POSITION_SPEED_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messagePositionDistance,messagePositionSpeed,maxSpeeds[vehdata.stationType]);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_IMP);
			}
			averageSpeed=(vehdata.speed_ms+lastMessage.speed_ms)/2.0;
			// Calculated average speed doesn't match with average speed of the CAMs
			if (messagePositionSpeed>averageSpeed*(1+m_opts.tolerance*tolMult) || messagePositionSpeed<averageSpeed*(1-m_opts.tolerance*tolMult)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;messageDistance=%f;calculatedSpeed=%f;averageSpeed=%f\"\n",
					msgNumber,msgType,MB_POSITION_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messagePositionDistance,messagePositionSpeed,averageSpeed);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_INC);
			}

			// ------- SPEED CHANGE ACCELERATION CHECK -------

			if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration && lastMessage.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
				const double messageSpeedAcceleration=(vehdata.speed_ms-lastMessage.speed_ms)/messageDeltaTime;
				const double averageAcceleration=(vehdata.longitudinalAcceleration+lastMessage.longitudinalAcceleration)/2.0;
				if (messageSpeedAcceleration>=0) {
					// Average acceleration needed to reach message speed is implausible
					if (messageSpeedAcceleration>maxAccelerations[vehdata.stationType]) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedAcceleration=%f;maxAcceleration=%f\"\n",
							msgNumber,msgType,MB_SPEED_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,maxAccelerations[vehdata.stationType]);
						MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
					}
	
					// Calculated average acceleration doesn't match with average acceleration of the CAMs
					if (messageSpeedAcceleration>averageAcceleration*(1+m_opts.tolerance*tolMult) || messageSpeedAcceleration<averageAcceleration*(1-m_opts.tolerance*tolMult)) {
						if (fabs(messageSpeedAcceleration-averageAcceleration)>5) {
							fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedAcceleration=%f;averageAcceleration=%f\"\n",
								msgNumber,msgType,MB_SPEED_ACCELERATION_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,averageAcceleration);
							MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_INC);
						}
					}
				} else {
					// Average acceleration needed to reach message speed is implausible
					if (messageSpeedAcceleration<minAccelerations[vehdata.stationType]) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedAcceleration=%f;minAcceleration=%f\"\n",
							msgNumber,msgType,MB_SPEED_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,minAccelerations[vehdata.stationType]);
						MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
					}
	
					// Calculated average acceleration doesn't match with average acceleration of the CAMs
					if (messageSpeedAcceleration<averageAcceleration*(1+m_opts.tolerance*tolMult) || messageSpeedAcceleration>averageSpeed*(1-m_opts.tolerance*tolMult)) {
						if (fabs(messageSpeedAcceleration-averageAcceleration)>5) {
							fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedAcceleration=%f;averageAcceleration=%f\"\n",
								msgNumber,msgType,MB_SPEED_ACCELERATION_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,averageAcceleration);
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
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"calculatedHeading=%f;averageHeading=%f\"\n",
					msgNumber,msgType,MB_POSITION_HEADING_INC,vehdata.gnTimestamp,vehdata.stationID,messageHeading,averageHeading);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_HEADING_INC);
			}

			// ------- HEADING CHANGE YAW RATE CHECK -------

			if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate && lastMessage.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
				messageHeadingYawRate=(fmod(vehdata.heading-lastMessage.heading+540,360)-180)/messageDeltaTime; // in degrees/second
				if (fabs(messageHeadingYawRate)>maxYawRates[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calcualtedYawRate=%f;maxYawRate=%f\"\n",
						msgNumber,msgType,MB_HEADING_YAW_RATE_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,maxYawRates[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_IMP);
				}
				const double averageYawRate=(vehdata.yawRate+lastMessage.yawRate)/2.0;
				if (fabs(messageHeadingYawRate-averageYawRate)>25) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedYawRate=%f;averageYawRate=%f\"\n",
						msgNumber,msgType,MB_HEADING_YAW_RATE_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,averageYawRate);
					MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_INC);
				}
			}

			// ------- POSITION + HEADING DRIVE DIRECTION CHECK -------

			// considering this as driving forward
			if (averageHeading-90<messageHeading<averageHeading+90) {
				if (lastMessage.driveDirection==DriveDirection_backward) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"calculatedHeading=%f;oldDriveDirection=%f\"\n",
						msgNumber,msgType,MB_POS_AND_HEADING_DIRECTION_INC,vehdata.gnTimestamp,vehdata.stationID,messageHeading,lastMessage.driveDirection);
					MB_CODE|=MB_CODE_CONV(MB_POS_AND_HEADING_DIRECTION_INC);
				}
			} else {
				if (lastMessage.driveDirection==DriveDirection_forward) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"calculatedHeading=%f;oldDriveDirection=%f\"\n",
						msgNumber,msgType,MB_POS_AND_HEADING_DIRECTION_INC,vehdata.gnTimestamp,vehdata.stationID,messageHeading,lastMessage.driveDirection);
					MB_CODE|=MB_CODE_CONV(MB_POS_AND_HEADING_DIRECTION_INC);
				}
			}
		}

		// ------- LENGTH WIDTH CHANGE -------

		if (vehdata.vehicleLength.isAvailable() && lastMessage.vehicleLength.isAvailable()) {
			if (vehdata.vehicleWidth.isAvailable() && lastMessage.vehicleWidth.isAvailable()) {
				// if we are checking a CAM izes have to be consistent
				// otherwise it's a perceived object so the perceived size can change
				if (msgType==CAM) {
					if (vehdata.vehicleLength.getData()!=lastMessage.vehicleLength.getData() || vehdata.vehicleWidth.getData()!=lastMessage.vehicleWidth.getData()) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"newDimensions=%fx%f;oldDimensions=%fx%f\"\n",
							msgNumber,msgType,MB_LENGTH_WIDTH_INC,vehdata.gnTimestamp,vehdata.stationID,vehdata.vehicleLength.getData(),vehdata.vehicleWidth.getData(),lastMessage.vehicleLength.getData(),lastMessage.vehicleWidth.getData());
						MB_CODE|=MB_CODE_CONV(MB_LENGTH_WIDTH_INC);
					}
				} else {
					if (vehdata.vehicleLength.getData()>lastMessage.vehicleLength.getData()*(1+m_opts.tolerance*tolMult)
						|| vehdata.vehicleLength.getData()<lastMessage.vehicleLength.getData()*(1-m_opts.tolerance*tolMult)
						|| vehdata.vehicleWidth.getData()>lastMessage.vehicleWidth.getData()*(1+m_opts.tolerance*tolMult)
						|| vehdata.vehicleWidth.getData()<lastMessage.vehicleWidth.getData()*(1-m_opts.tolerance*tolMult)) {

						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"newDimensions=%fx%f;oldDimensions=%fx%f\"\n",
							msgNumber,msgType,MB_LENGTH_WIDTH_INC,vehdata.gnTimestamp,vehdata.stationID,vehdata.vehicleLength.getData(),vehdata.vehicleWidth.getData(),lastMessage.vehicleLength.getData(),lastMessage.vehicleWidth.getData());
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
			if (messageAccelerationChange>=0) {
				if (messageAccelerationChange>maxJerks[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedJerk=%f;maxJerk=%f\"\n",
						msgNumber,msgType,MB_ACCELERATION_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageAccelerationChange,maxJerks[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_INC);
				}
			} else {
				if (messageAccelerationChange<-maxJerks[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedJerk=%f;minJerk=%f\"\n",
						msgNumber,msgType,MB_ACCELERATION_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageAccelerationChange,-maxJerks[vehdata.stationType]);
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
			if (fabs(curvatureRatio-1)<0.05 || fabs(dCurvature)<0.01) {
				
				// ------- HEADING CHANGE SPEED CHECK -------

				// yaw rate calculated as average speed * average curvature where curvature is assumed ~constant
				const double averageSpeedYawRate=(fabs(averageSpeed)*(vehdata.curvature+lastMessage.curvature)/2.0)/radiansFactor;
				if (messageHeadingYawRate>averageSpeedYawRate*(1+m_opts.tolerance*tolMult) || messageHeadingYawRate<averageSpeedYawRate*(1-m_opts.tolerance*tolMult)) {
					if (fabs(messageHeadingYawRate-averageSpeedYawRate)>2) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedYawRate=%f;averageSpeedYawRate=%f\"\n",
							msgNumber,msgType,MB_HEADING_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,averageSpeedYawRate);
						MB_CODE|=MB_CODE_CONV(MB_HEADING_SPEED_INC); // DA CAMBIARE
					}
				}
				// const double expectedSpeed=vehdata.yawRate*lastMessage.speed_ms/lastMessage.yawRate;
				// const double error=fabs(expectedSpeed-vehdata.speed_ms);
				// const double errorRatio=fabs(expectedSpeed)>1e-9 ? vehdata.speed_ms/expectedSpeed : 2;
				// if (fabs(errorRatio-1)>0.05) {
				// 	if (error>0.01) {
				// 		fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f;calculatedSpeed=%f\"\n",
				// 			msgNumber,msgType,MB_HEADING_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.speed_ms,expectedSpeed);
				// 		MB_CODE|=MB_CODE_CONV(MB_HEADING_SPEED_INC);
				// 	}
				// }
				
				// ------- YAW RATE CHANGE SPEED CHECK -------

				const double expectedYawRate=vehdata.speed_ms*lastMessage.yawRate/lastMessage.speed_ms;
				const double error=fabs(expectedYawRate-vehdata.yawRate);
				const double errorRatio=fabs(expectedYawRate)>1e-9 ? vehdata.yawRate/expectedYawRate : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>2) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"yawRate=%f;calculatedYawRate=%f\"\n",
							msgNumber,msgType,MB_YAW_RATE_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.yawRate,expectedYawRate);
						MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_SPEED_INC);
					}
				}
			}

			const double dSpeed=vehdata.speed_ms-lastMessage.speed_ms;
			if (fabs(speedRatio-1)<0.05 || fabs(dSpeed)<1) {
				// pretty much the same checks

				// ------- CURVATURE CHANGE YAW RATE CHECK -------

				const double expectedCurvature=lastMessage.curvature*vehdata.yawRate/lastMessage.yawRate;
				double error=fabs(expectedCurvature-vehdata.curvature);
				double errorRatio=fabs(expectedCurvature)>1e-9 ? vehdata.curvature/expectedCurvature : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>0.01) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"curvature=%f;calculatedCurvature=%f\"\n",
							msgNumber,msgType,MB_CURVATURE_YAW_RATE_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.curvature,expectedCurvature);
						MB_CODE|=MB_CODE_CONV(MB_CURVATURE_YAW_RATE_INC);
					}
				}
		
				// ------- YAW RATE CHANGE CURVATURE CHECK -------

				const double expectedYawRate=vehdata.curvature*lastMessage.yawRate/lastMessage.curvature;
				error=fabs(expectedYawRate-vehdata.yawRate);
				errorRatio=fabs(expectedYawRate)>1e-9 ? vehdata.yawRate/expectedYawRate : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>2) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"yawRate=%f;calculatedYawRate=%f\"\n",
							msgNumber,msgType,MB_YAW_RATE_CURVATURE_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.yawRate,expectedYawRate);
						MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_CURVATURE_INC);
					}
				}

			}
			
			// ------- CURVATURE CHANGE SPEED CHECK -------

			const double dYawRate=vehdata.yawRate-lastMessage.yawRate;
			if ((fabs(yawRateRatio-1)<0.05 || fabs(dYawRate)<1) && vehdata.speed_ms!=0) {
				const double expectedCurvature=lastMessage.curvature*lastMessage.speed_ms/vehdata.speed_ms;
				const double error=fabs(expectedCurvature-vehdata.curvature);
				const double errorRatio=fabs(expectedCurvature)>1e-9 ? vehdata.curvature/expectedCurvature : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>0.01) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"curvature=%f;calculatedCurvature=%f\"\n",
							msgNumber,msgType,MB_CURVATURE_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.curvature,expectedCurvature);
						MB_CODE|=MB_CODE_CONV(MB_CURVATURE_SPEED_INC);
					}
				}
			}
		}
	}

	// ------- CLASS 3 CHECKS -------
	double distance=3;
	osmium::object_id_type closestWay=m_osmStore->checkIfPointOnRoad(vehdata.lat,vehdata.lon,distance,lastMessagePresent?lastMessage.wayId:-1);
	if (closestWay==-1) {
		fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"latitude=%f;longitude=%f;distance=%f\"\n",msgNumber,msgType,MB_NOT_ON_ROAD,vehdata.gnTimestamp,vehdata.stationID,vehdata.lat,vehdata.lon,distance);
		MB_CODE|=MB_CODE_CONV(MB_NOT_ON_ROAD);

		// check if inside a building, can trigger another misbehaviour after "not on road", (maybe can justify "not on road" if in a "driveable" building)
		if (m_osmStore->checkIfPointInBuilding(vehdata.lat,vehdata.lon)) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"latitude=%f;longitude=%f\"\n",msgNumber,msgType,MB_INSIDE_BUILDING,vehdata.gnTimestamp,vehdata.stationID,vehdata.lat,vehdata.lon);
			MB_CODE|=MB_CODE_CONV(MB_INSIDE_BUILDING);
		}
	} else {
		vehdata.wayId=closestWay;
		// on road: can check the rest

		if (vehdata.heading!=ldmmap::e_DataUnavailableValue::heading) {
			double roadHeading;
			if (m_osmStore->checkHeadingNotFollowingRoad(vehdata.heading,vehdata.lat,vehdata.lon,closestWay,roadHeading)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"heading=%f;roadHeading=%f\"\n",msgNumber,msgType,MB_HEADING_NOT_FOLLOWING_ROAD,vehdata.gnTimestamp,vehdata.stationID,vehdata.heading,roadHeading);
				MB_CODE|=MB_CODE_CONV(MB_HEADING_NOT_FOLLOWING_ROAD);
			}
		}

		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			double speedLimit;
			if (m_osmStore->checkSpeedOverTypeLimit(vehdata.speed_ms,closestWay,speedLimit)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f;speedLimit=%f\"\n",msgNumber,msgType,MB_SPEED_OVER_ROAD_LIMIT,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,speedLimit);
				MB_CODE|=MB_CODE_CONV(MB_SPEED_OVER_ROAD_LIMIT);
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
					double distance=haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon);
					if (distance>100) {
						// misbehaviour: messages too far
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"distance=%f\"\n",msgNumber,DENM,EMB_DISTANCE_DENM_CAM,vehdata.gnTimestamp,vehdata.stationID,distance);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_DISTANCE_DENM_CAM);
						ev.second.unavailables&=MB_CODE_CLEAR(EMB_DISTANCE_DENM_CAM); // clear unavailable bit to avoid checking again in next CAMs
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_REPORTER_SPEED)) {
					if (vehdata.speed_ms>8.3) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f\"\n",msgNumber,DENM,EMB_REPORTER_SPEED,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_REPORTER_SPEED);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_REPORTER_SLOW_DOWN)) {
					if (lastMessagePresent) {
						// has to slow down to 30km/h
						if (vehdata.speed_ms>8.3 && vehdata.speed_ms>lastMessage.speed_ms) {
							fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"newSpeed=%f;oldSpeed=%f\"\n",msgNumber,DENM,EMB_REPORTER_SLOW_DOWN,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,lastMessage.speed_ms);
							ev.second.EMB_CODE|=MB_CODE_CONV(EMB_REPORTER_SLOW_DOWN);
						}
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_CAM_SPEED)) {
					if (vehdata.speed_ms>0.1) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f\"\n",msgNumber,DENM,EMB_CAM_SPEED,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_CAM_SPEED);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_STATION_TYPE_REPORTER_CAM)) {
					if (vehdata.stationType!=ldmmap::StationType_LDM_specialVehicles) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"stationType=%f\"\n",msgNumber,DENM,EMB_STATION_TYPE_REPORTER_CAM,vehdata.gnTimestamp,vehdata.stationID,vehdata.stationType);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_STATION_TYPE_REPORTER_CAM);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_LIGHTBAR_ACTIVE_CAM)) {
					if (vehdata.lightBarActivated.isAvailable()) {
						if (!vehdata.lightBarActivated.getData()) {
							fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"lightBarActivated=%f\"\n",msgNumber,DENM,EMB_LIGHTBAR_ACTIVE_CAM,vehdata.gnTimestamp,vehdata.stationID,vehdata.lightBarActivated.getData());
							ev.second.EMB_CODE|=MB_CODE_CONV(EMB_LIGHTBAR_ACTIVE_CAM);
						}
					} else {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"lightBarActivated=%f\"\n",msgNumber,DENM,EMB_LIGHTBAR_ACTIVE_CAM,vehdata.gnTimestamp,vehdata.stationID,vehdata.lightBarActivated.getData());
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_LIGHTBAR_ACTIVE_CAM);
					}
				}

				if (MB_CODE_CHECK(eveCheck,EMB_IRC_EVENT_SPEED_INC)) {
					if (fabs(ev.second.evedata.eventSpeed.getData()-vehdata.speed_ms)>5) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"eventSpeed=%f;vehicleSpeed=%f\"\n",
							msgNumber,DENM,EMB_IRC_EVENT_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,ev.second.evedata.eventSpeed.getData(),vehdata.speed_ms);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_EVENT_SPEED_INC);
					}
					ev.second.unavailables&=MB_CODE_CLEAR(EMB_IRC_EVENT_SPEED_INC);
				}

				if (MB_CODE_CHECK(eveCheck,EMB_IRC_EVENT_HEADING_INC)) {
					if (fabs(ev.second.evedata.eventPositionHeading.getData()-vehdata.heading)>10) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"eventHeading=%f;vehicleHeading=%f\"\n",
							msgNumber,DENM,EMB_IRC_EVENT_HEADING_INC,vehdata.gnTimestamp,vehdata.stationID,ev.second.evedata.eventPositionHeading.getData(),vehdata.heading);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_EVENT_HEADING_INC);
					}
					ev.second.unavailables&=MB_CODE_CLEAR(EMB_IRC_EVENT_HEADING_INC);
				}

				if (MB_CODE_CHECK(eveCheck,EMB_IRC_BEHAVIOUR_ACCELERATION)) {
					if (lastMessagePresent) {
						if (vehdata.speed_ms>=lastMessage.speed_ms) {
							fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"newSpeed=%f;oldSpeed=%f\"\n",msgNumber,DENM,EMB_IRC_BEHAVIOUR_ACCELERATION,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,lastMessage.speed_ms);
							ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_BEHAVIOUR_ACCELERATION);
						}
					}
					if (vehdata.longitudinalAcceleration>0) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"acceleration=%f\"\n",msgNumber,DENM,EMB_IRC_BEHAVIOUR_ACCELERATION,vehdata.gnTimestamp,vehdata.stationID,vehdata.longitudinalAcceleration);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_IRC_BEHAVIOUR_ACCELERATION);
					}
					// maybe after the first emergency brake the car stops braking so this also needs to be checked only once and not for the whole period of the event
					ev.second.unavailables&=MB_CODE_CLEAR(EMB_IRC_BEHAVIOUR_ACCELERATION);
				}
			}
		} else if (MB_CODE_CHECK(ev.second.unavailables,EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED)) {
			// nested ifs to avoid the heavier distance calculation
			if (lastMessagePresent && closestWay==ev.second.eventRoad && fabs(vehdata.heading-ev.second.eventHeading)<45) {
				if (haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon)<100) {
					if (vehdata.speed_ms>8.3 && vehdata.speed_ms>lastMessage.speed_ms) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"newSpeed=%f;oldSpeed=%f\"\n",msgNumber,DENM,EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,lastMessage.speed_ms);
						ev.second.EMB_CODE|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED);
					}
				}
			}
		}
	}

	if (msgType==CPM) {
		// clean the unavailables for cpms
		unavailables&=(MB_CODE_CONV(UNAV_LATITUDE)||MB_CODE_CONV(UNAV_LONGITUDE));
	}

	return MB_CODE;
}

uint64_t MisbehaviourDetector::individualVAMchecks(ldmmap::vehicleData_t &vehdata, uint64_t &unavailables, int msgType) {
	uint64_t MB_CODE=0;
	ldmmap::vehicleData_t lastMessage;
	bool lastMessagePresent=false;
	double tolMult=msgType==VAM?1:m_opts.cpmToleranceMultiplier;

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
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f;maxSpeed=%f\"\n",msgNumber,msgType,MB_SPEED_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,maxSpeeds[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_SPEED_IMP);
		}
	} else {
			unavailables|=MB_CODE_CONV(UNAV_SPEED);
	}

	// ------- PLAUSIBLE MAX ACCELERATION CHECK -------

	if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
		//accelerating
		if (vehdata.longitudinalAcceleration>maxAccelerations[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"acceleration=%f;maxAcceleration=%f\"\n",msgNumber,msgType,MB_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.longitudinalAcceleration,maxAccelerations[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
		}
		//braking
		if (vehdata.longitudinalAcceleration<minAccelerations[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"acceleration=%f;minAcceleration=%f\"\n",msgNumber,msgType,MB_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.longitudinalAcceleration,minAccelerations[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_IMP);
		}
	} else {
		unavailables|=MB_CODE_CONV(UNAV_ACCELERATION);
	}

	// ------- PLAUSIBLE MAX LATERAL ACCELERATION CHECK (FROM CURVATURE) -------
	const double curvaturelateralAcceleration=pow(vehdata.speed_ms,2)*vehdata.curvature;
	if (vehdata.curvature!=ldmmap::e_DataUnavailableValue::curvature) {
		if (curvaturelateralAcceleration>maxLateralAcceleration[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"lateralAcceleration=%f;maxLateralAcceleration=%f\"\n",
				msgNumber,msgType,MB_CURVATURE_IMP,vehdata.gnTimestamp,vehdata.stationID,curvaturelateralAcceleration,maxLateralAcceleration[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_CURVATURE_IMP);
		}
	} else {
		
	}

	// ------- PLAUSIBLE MAX YAW RATE CHECK -------

	if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
		if (vehdata.yawRate>maxYawRates[vehdata.stationType]) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"yawRate=%f;maxYawRate=%f\"\n",msgNumber,msgType,MB_YAW_RATE_IMP,vehdata.gnTimestamp,vehdata.stationID,vehdata.yawRate,maxYawRates[vehdata.stationType]);
			MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_IMP);
		}
	} else {
		
	}
	}

	double messageDeltaTime; // seconds
	// ------- CLASS 2 CHECKS -------
	if (lastMessagePresent) {

		// ------- BEACON FREQUENCY CHECK -------
		messageDeltaTime=(vehdata.camTimestamp-lastMessage.camTimestamp)/1000.0;// in seconds
		if (messageDeltaTime<0) {
			// messageDeltaTime+=429496.7296; // for gnTimestamp divided by 1000 to be in seconds
			messageDeltaTime+=65.536; // for camTimestamp divided by 1000 to be in seconds
		}
		if (messageDeltaTime>m_opts.maxTimeForConsecutive) {
			// Messages are too far in time for the used configuration, avoid checking any class 2
			// reuse lastMessagePresent set to false to pretend there is no last message (potentially a second variable could be used but its not needed in the current scenario)
			MB_CODE|=MB_CODE_CONV(MB_BEACON_FREQ_LOW);
			lastMessagePresent=false;
		}
		if (messageDeltaTime<0.095) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;lastTimestamp=%li\"\n",msgNumber,msgType,MB_BEACON_FREQ_INC,vehdata.camTimestamp,vehdata.stationID,messageDeltaTime,lastMessage.camTimestamp);
			MB_CODE|=MB_CODE_CONV(MB_BEACON_FREQ_INC);
		}

	}

	// ------- CLASS 2 CHECKS -------
	if (lastMessagePresent) {

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
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;messageDistance=%f;calculatedSpeed=%f;maxSpeed=%f\"\n",
					msgNumber,msgType,MB_POSITION_SPEED_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messagePositionDistance,messagePositionSpeed,maxSpeeds[vehdata.stationType]);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_IMP);
			}
			averageSpeed=(vehdata.speed_ms+lastMessage.speed_ms)/2.0;
			// Calculated average speed doesn't match with average speed of the CAMs
			if (messagePositionSpeed>averageSpeed*(1+m_opts.tolerance*tolMult) || messagePositionSpeed<averageSpeed*(1-m_opts.tolerance*tolMult)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;messageDistance=%f;calculatedSpeed=%f;averageSpeed=%f\"\n",
					msgNumber,msgType,MB_POSITION_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messagePositionDistance,messagePositionSpeed,averageSpeed);
				MB_CODE|=MB_CODE_CONV(MB_POSITION_SPEED_INC);
			}

			// ------- SPEED CHANGE ACCELERATION CHECK -------

			const double messageSpeedAcceleration=(vehdata.speed_ms-lastMessage.speed_ms)/messageDeltaTime;
			// Average acceleration needed to reach message speed is implausible
			if (messageSpeedAcceleration>=0) {
				if (messageSpeedAcceleration>maxAccelerations[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedAcceleration=%f;maxAcceleration=%f\"\n",
							msgNumber,msgType,MB_SPEED_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,maxAccelerations[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
				}
			} else {
				if (messageSpeedAcceleration<minAccelerations[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedAcceleration=%f;minAcceleration=%f\"\n",
							msgNumber,msgType,MB_SPEED_ACCELERATION_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,minAccelerations[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_SPEED_ACCELERATION_IMP);
				}
			}
			const double averageAcceleration=(vehdata.longitudinalAcceleration+lastMessage.longitudinalAcceleration)/2.0;
			// Calculated average acceleration doesn't match with average acceleration of the CAMs
			if (messageSpeedAcceleration>averageAcceleration*(1+m_opts.tolerance*tolMult) || messageSpeedAcceleration<averageAcceleration*(1-m_opts.tolerance*tolMult)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedAcceleration=%f;averageAcceleration=%f\"\n",
								msgNumber,msgType,MB_SPEED_ACCELERATION_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageSpeedAcceleration,averageAcceleration);
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
				if (fabs(messageHeading-averageHeading)>5) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"calculatedHeading=%f;averageHeading=%f\"\n",
						msgNumber,msgType,MB_POSITION_HEADING_INC,vehdata.gnTimestamp,vehdata.stationID,messageHeading,averageHeading);
					MB_CODE|=MB_CODE_CONV(MB_POSITION_HEADING_INC);
				}
			}

			// ------- HEADING CHANGE YAW RATE CHECK -------

			if (vehdata.yawRate!=ldmmap::e_DataUnavailableValue::yawRate && lastMessage.yawRate!=ldmmap::e_DataUnavailableValue::yawRate) {
				messageHeadingYawRate=(fmod(vehdata.heading-lastMessage.heading+540,360)-180)/messageDeltaTime; // in degrees/second
				if (fabs(messageHeadingYawRate)>maxYawRates[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calcualtedYawRate=%f;maxYawRate=%f\"\n",
						msgNumber,msgType,MB_HEADING_YAW_RATE_IMP,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,maxYawRates[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_IMP);
				}
				const double averageYawRate=(vehdata.yawRate+lastMessage.yawRate)/2.0;
				if (fabs(messageHeadingYawRate-averageYawRate)>25) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedYawRate=%f;averageYawRate=%f\"\n",
						msgNumber,msgType,MB_HEADING_YAW_RATE_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,averageYawRate);
					MB_CODE|=MB_CODE_CONV(MB_HEADING_YAW_RATE_INC);
				}
			}
		}

		// ------- LENGTH WIDTH CHANGE -------

		if (vehdata.vruSizeClass!=VruSizeClass_unavailable && lastMessage.vruSizeClass!=VruSizeClass_unavailable) {
			if (vehdata.vruSizeClass!=lastMessage.vruSizeClass) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"newSizeClass=%f;oldSizeClass=%f\"\n",
						msgNumber,msgType,MB_HEADING_YAW_RATE_INC,vehdata.gnTimestamp,vehdata.stationID,vehdata.vruSizeClass,lastMessage.vruSizeClass);
				MB_CODE|=MB_CODE_CONV(MB_LENGTH_WIDTH_INC);
			}
		} else {
			
		}

		// ------- ACCELERATION CHECK -------

		if (vehdata.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration && lastMessage.longitudinalAcceleration!=ldmmap::e_DataUnavailableValue::longitudinalAcceleration) {
			const double messageAccelerationChange=vehdata.longitudinalAcceleration-lastMessage.longitudinalAcceleration;
			if (messageAccelerationChange>=0) {
				if (messageAccelerationChange>maxJerks[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedJerk=%f;maxJerk=%f\"\n",
							msgNumber,msgType,MB_ACCELERATION_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageAccelerationChange,maxJerks[vehdata.stationType]);
					MB_CODE|=MB_CODE_CONV(MB_ACCELERATION_INC);
				}
			} else {
				if (messageAccelerationChange<-maxJerks[vehdata.stationType]) {
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedJerk=%f;minJerk=%f\"\n",
							msgNumber,msgType,MB_ACCELERATION_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageAccelerationChange,-maxJerks[vehdata.stationType]);
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
			if (fabs(curvatureRatio-1)<0.05 || fabs(dCurvature)<0.01) {
				
				// ------- HEADING CHANGE SPEED CHECK -------

				// yaw rate calculated as average speed * average curvature where curvature is assumed ~constant
				const double averageSpeedYawRate=(fabs(averageSpeed)*(vehdata.curvature+lastMessage.curvature)/2.0)/radiansFactor;
				if (messageHeadingYawRate>averageSpeedYawRate*(1+m_opts.tolerance*tolMult) || messageHeadingYawRate<averageSpeedYawRate*(1-m_opts.tolerance*tolMult)) {
					if (fabs(messageHeadingYawRate-averageSpeedYawRate)>2) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"deltaTime=%f;calculatedYawRate=%f;averageSpeedYawRate=%f\"\n",
							msgNumber,msgType,MB_HEADING_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,messageHeadingYawRate,averageSpeedYawRate);
						MB_CODE|=MB_CODE_CONV(MB_HEADING_SPEED_INC); // DA CAMBIARE
					}
				}
				// const double expectedSpeed=vehdata.yawRate*lastMessage.speed_ms/lastMessage.yawRate;
				// const double error=fabs(expectedSpeed-vehdata.speed_ms);
				// const double errorRatio=fabs(expectedSpeed)>1e-9 ? vehdata.speed_ms/expectedSpeed : 2;
				// if (fabs(errorRatio-1)>0.05) {
				// 	if (error>0.01) {
				// 		fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f;calculatedSpeed=%f\"\n",
				// 			msgNumber,msgType,MB_HEADING_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.speed_ms,expectedSpeed);
				// 		MB_CODE|=MB_CODE_CONV(MB_HEADING_SPEED_INC);
				// 	}
				// }
				
				// ------- YAW RATE CHANGE SPEED CHECK -------

				const double expectedYawRate=vehdata.speed_ms*lastMessage.yawRate/lastMessage.speed_ms;
				const double error=fabs(expectedYawRate-vehdata.yawRate);
				const double errorRatio=fabs(expectedYawRate)>1e-9 ? vehdata.yawRate/expectedYawRate : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>2) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"yawRate=%f;calculatedYawRate=%f\"\n",
							msgNumber,msgType,MB_YAW_RATE_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.yawRate,expectedYawRate);
						MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_SPEED_INC);
					}
				}
			}

			const double dSpeed=vehdata.speed_ms-lastMessage.speed_ms;
			if (fabs(speedRatio-1)<0.05 || fabs(dSpeed)<1) {
				// pretty much the same checks

				// ------- CURVATURE CHANGE YAW RATE CHECK -------

				const double expectedCurvature=lastMessage.curvature*vehdata.yawRate/lastMessage.yawRate;
				double error=fabs(expectedCurvature-vehdata.curvature);
				double errorRatio=fabs(expectedCurvature)>1e-9 ? vehdata.curvature/expectedCurvature : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>0.01) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"curvature=%f;calculatedCurvature=%f\"\n",
							msgNumber,msgType,MB_CURVATURE_YAW_RATE_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.curvature,expectedCurvature);
						MB_CODE|=MB_CODE_CONV(MB_CURVATURE_YAW_RATE_INC);
					}
				}
		
				// ------- YAW RATE CHANGE CURVATURE CHECK -------

				const double expectedYawRate=vehdata.curvature*lastMessage.yawRate/lastMessage.curvature;
				error=fabs(expectedYawRate-vehdata.yawRate);
				errorRatio=fabs(expectedYawRate)>1e-9 ? vehdata.yawRate/expectedYawRate : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>2) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"yawRate=%f;calculatedYawRate=%f\"\n",
							msgNumber,msgType,MB_YAW_RATE_CURVATURE_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.yawRate,expectedYawRate);
						MB_CODE|=MB_CODE_CONV(MB_YAW_RATE_CURVATURE_INC);
					}
				}

			}
			
			// ------- CURVATURE CHANGE SPEED CHECK -------

			const double dYawRate=vehdata.yawRate-lastMessage.yawRate;
			if ((fabs(yawRateRatio-1)<0.05 || fabs(dYawRate)<1) && vehdata.speed_ms!=0) {
				const double expectedCurvature=lastMessage.curvature*lastMessage.speed_ms/vehdata.speed_ms;
				const double error=fabs(expectedCurvature-vehdata.curvature);
				const double errorRatio=fabs(expectedCurvature)>1e-9 ? vehdata.curvature/expectedCurvature : 2;
				if (fabs(errorRatio-1)>0.05) {
					if (error>0.01) {
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"curvature=%f;calculatedCurvature=%f\"\n",
							msgNumber,msgType,MB_CURVATURE_SPEED_INC,vehdata.gnTimestamp,vehdata.stationID,messageDeltaTime,vehdata.curvature,expectedCurvature);
						MB_CODE|=MB_CODE_CONV(MB_CURVATURE_SPEED_INC);
					}
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
	double distance=3;
	osmium::object_id_type closestWay=m_osmStore->checkIfPointOnRoad(vehdata.lat,vehdata.lon,distance,lastMessagePresent?lastMessage.wayId:-1);
	if (closestWay==-1) {
		// not on road
		fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"latitude=%f;longitude=%f;distance=%f\"\n",msgNumber,msgType,MB_NOT_ON_ROAD,vehdata.gnTimestamp,vehdata.stationID,vehdata.lat,vehdata.lon,distance);
		MB_CODE|=MB_CODE_CONV(MB_NOT_ON_ROAD);
		if (m_osmStore->checkIfPointInBuilding(vehdata.lat,vehdata.lon)) {
			fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"latitude=%f;longitude=%f\"\n",msgNumber,msgType,MB_INSIDE_BUILDING,vehdata.gnTimestamp,vehdata.stationID,vehdata.lat,vehdata.lon);
			MB_CODE|=MB_CODE_CONV(MB_INSIDE_BUILDING);
		}
		if (vehdata.vruEnvironment==VruEnvironment_onVehicleRoad) {
			MB_CODE|=MB_CODE_CONV(MB_ENVIRONMENT);
		}
	} else {
		vehdata.wayId=closestWay;
		// on road
		if (vehdata.heading!=ldmmap::e_DataUnavailableValue::heading) {
			double roadHeading;
			if (m_osmStore->checkHeadingNotFollowingRoad(vehdata.heading,vehdata.lat,vehdata.lon,closestWay,roadHeading)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"heading=%f;roadHeading=%f\"\n",msgNumber,msgType,MB_HEADING_NOT_FOLLOWING_ROAD,vehdata.gnTimestamp,vehdata.stationID,vehdata.heading,roadHeading);
				MB_CODE|=MB_CODE_CONV(MB_HEADING_NOT_FOLLOWING_ROAD);
			}
		}
		if (vehdata.speed_ms!=ldmmap::e_DataUnavailableValue::speed) {
			double speedLimit;
			if (m_osmStore->checkSpeedOverTypeLimit(vehdata.speed_ms,closestWay,speedLimit)) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,\"speed=%f;speedLimit=%f\"\n",msgNumber,msgType,MB_SPEED_OVER_ROAD_LIMIT,vehdata.gnTimestamp,vehdata.stationID,vehdata.speed_ms,speedLimit);
				MB_CODE|=MB_CODE_CONV(MB_SPEED_OVER_ROAD_LIMIT);
			}
		}
	}

	if (msgType==CPM) {
		// clean the unavailables for cpms
		unavailables&=(MB_CODE_CONV(UNAV_LATITUDE)||MB_CODE_CONV(UNAV_LONGITUDE));
	}

	return MB_CODE;	
}

uint64_t MisbehaviourDetector::individualCPMchecks(std::vector<ldmmap::vehicleData_t> &PO_vec, uint64_t &unavailables) {
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
			MB_CODE|=individualVAMchecks(vehdata,unavailables,CPM);
		} else {
			// CAM controls for other stationTypes
			MB_CODE|=individualCAMchecks(vehdata,unavailables,CPM);
		}
	}
	return MB_CODE;
}

size_t WriteCallbackMBD(void* ptr, size_t size, size_t nmemb, void* stream) {
	((std::string *)stream)->append((char *)ptr, size * nmemb);
    return size*nmemb;
}


void MisbehaviourDetector::processDENM(proton::binary message_bin, ldmmap::eventData_t &evedata, Security::Security_error_t sec_retval, storedCertificate_t certificateData) {
	std::lock_guard<std::mutex> mbd_lock(m_mbd_mutex);

	bool pending;
	pendingEvent_t currentEvent;

	currentEvent.keyEvent = m_db_ptr->KEY_EVENT(evedata.eventLatitude,evedata.eventLongitude,evedata.eventElevation,evedata.eventCauseCode);
	uint64_t expiration=get_timestamp_ns()+20e9; // 20 seconds from now

	// Check for security, currently proceeds with the rest of the checks, this behaviour may change according to the reporting service needs
	// In particular for DENMs management there is no return code here so the current state is basically a stub
	if (!m_opts.ignoreSecurity) {
		switch (sec_retval) {
			case Security::SECURITY_NO_SEC:
				break;
			case Security::SECURITY_VALID_CERTIFICATE:
				if (!certificateData.digest.empty())
					m_certStore_ptr->insert_or_assign(certificateData.digest,certificateData);
				break;
			case Security::SECURITY_VERIFICATION_FAILED:
				// left here for possible future changes
				// at the moment this returnvalue makes geonet and then etsidecoder to return an error
				// meaning the message is discarded before reaching here
				break;
			case Security::SECURITY_INVALID_CERTIFICATE:
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
	}

	// if the event doesn't exist in the validation buffer add it and perform individualDENMchecks
	// otherwise add the new reporter to the reporters
	if (m_pendingEvents.find(currentEvent.keyEvent)==m_pendingEvents.end()) {

		currentEvent.endOfChecks=expiration;
		currentEvent.evedata=evedata;
		currentEvent.EMB_CODE=0;
		currentEvent.unavailables=0;
		currentEvent.reporters.insert(evedata.originatingStationID);
		
		ldmmap::vehicleData_t vehdata;
		bool lastMessagePresent;
		pending=false;

		uint64_t keyEvent = m_db_ptr->KEY_EVENT(evedata.eventLatitude,evedata.eventLongitude,evedata.eventElevation,evedata.eventCauseCode);

		if (m_lastMessageCache.find(evedata.originatingStationID)!=m_lastMessageCache.end()) {
			vehdata=m_lastMessageCache.at(evedata.originatingStationID);
			lastMessagePresent=true;
			currentEvent.eventHeading=vehdata.heading;
		} else {
			lastMessagePresent=false;
			currentEvent.eventHeading=720;
		}

		double distance=5;
		currentEvent.eventRoad=m_osmStore->checkIfPointOnRoad(evedata.eventLatitude,evedata.eventLongitude,distance);
		currentEvent.eventRoad;
		

		// ------- GENERAL -------

		// DENM and CAM distance check 
		if (lastMessagePresent) {
			if (haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon)>100) {
				// misbehaviour: messages too far
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,%f\n",msgNumber,DENM,EMB_DISTANCE_DENM_CAM,evedata.gnTimestampDENM,evedata.originatingStationID,haversineDist(evedata.eventLatitude,evedata.eventLongitude,vehdata.lat,vehdata.lon));
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
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,%d\n",msgNumber,DENM,EMB_ROAD_TYPE,evedata.gnTimestampDENM,evedata.originatingStationID,evedata.roadType.getData());
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_ROAD_TYPE);
				}
			} else {
				pending=true;
				// unavailables: roadType
				currentEvent.unavailables|=MB_CODE_CONV(EMB_ROAD_TYPE);
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
						fprintf(log_csv,"%d,%d,%d,%lu,%lu,%f\n",msgNumber,DENM,EMB_REPORTER_SPEED,evedata.gnTimestampDENM,evedata.originatingStationID,vehdata.speed_ms);
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
				if (fabs(vd.heading-vehdata.heading)<45) {
					const double radiansFactor=M_PI/180;
					double dLat=vd.lat-vehdata.lat;
					double dLon=vd.lon-vehdata.lon;
					// check if it's in front by calculating the heading of the vector formed between the 2 cars
					// since the vector head is on the other car this has to be in front to have a similar heading to the driving direction
					if (fabs(vehdata.heading-atan2(dLon,dLat)/radiansFactor)<headingLimit) {
						if (vd.speed_ms>speedLimit) {
							// misbehaviour: above value
							fprintf(log_csv,"%d,%d,%d,%lu,%lu,%lu,%f\n",msgNumber,DENM,code_setter,evedata.gnTimestampDENM,evedata.originatingStationID,vd.stationID,vd.speed_ms); // additional data is stationId and speed of each ahead vehicle over the limit
							currentEvent.EMB_CODE|=MB_CODE_CONV(code_setter);
						}
					}
				}

				// ------- DENSITY/SPEED STAT -------
				sumSpeed+=vd.speed_ms;
			}
			// if nearby there are less than 15 vehicles (density) or their average speed is above 50km/h (speed)
			if (selectedVehicles.size()<15 || sumSpeed/selectedVehicles.size()>13.8) {
				fprintf(log_csv,"%d,%d,%d,%lu,%lu,%d,%f\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,selectedVehicles.size(),sumSpeed/selectedVehicles.size()); // add data density (rather number of vehicles) and avg speed
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
					fprintf(log_csv,"%d,%d,%d,%lu,%lu,%f\n",EMB_DISTANCE_DENM_CAM,msgNumber,evedata.gnTimestampDENM,evedata.originatingStationID,vehdata.speed_ms);
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

			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED);
			currentEvent.unavailables|=MB_CODE_CONV(EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY);
		}
		// exchange of IRCs
		if (evedata.eventCauseCode==CauseCodeType_collisionRisk) {
			if (evedata.vehicleMass.getData()>vehicleMasses[evedata.eventStationType]) {
				currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_IRC_VEHICLE_MASS);
			}

			if (lastMessagePresent) {
				if (fabs(evedata.eventSpeed.getData()-vehdata.speed_ms)>5) {
					currentEvent.EMB_CODE|=MB_CODE_CONV(EMB_IRC_EVENT_SPEED_INC);	
				}
				if (fabs(evedata.eventPositionHeading.getData()-vehdata.heading)>10) {
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
			for (auto p:misbehaviourStringsEvent) {
				if (currentEvent.EMB_CODE&(1<<p.first)) {
					misbehavioursDENM[p.first]++;
				}
			}
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
					fprintf(log_summary,"%d:\t\t%s\n",misbehavioursDENM[p.first],p.second.c_str());
				}
				fflush(log_summary);
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
	std::lock_guard<std::mutex> mbd_lock(m_mbd_mutex);

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