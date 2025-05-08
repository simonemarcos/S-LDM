#include "MisbehaviourDetector.h"
#include "algorithm"
#include "utils.h"

#include "utmuts.h"

#include <iostream>

#include <set>


extern "C" {
	#include "CAM.h"
	#include "CPM.h"
	#include "VAM.h"
}

bool MisbehaviourDetector::processMessage(etsiDecoder::etsiDecodedData_t decoded_data, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id) {
	bool retval = false;
	uint64_t MB_CODE=0;
	ldmmap::LDMMap::LDMMap_error_t db_retval;
	ldmmap::vehicleData_t vehdata;
	vehicleDataVector_t(PO_vec);

	if(!m_db_ptr || !m_opts_ptr) {
		return retval;
	}

	if(m_detection_enabled==true) {
		if (decoded_data.type==etsiDecoder::ETSI_DECODED_CAM || decoded_data.type==etsiDecoder::ETSI_DECODED_CAM_NOGN) {
			if (decodeCAM(decoded_data,msg,on_msg_timestamp_us,main_bf,m_client_id,vehdata)==false) {
				return retval;
			}
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
			} else {
				//no Misbehaviour detected, insert into ldm
				db_retval=m_db_ptr->insert(vehdata);
				updateLastMessage(vehdata);
			}

		} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_CPM || decoded_data.type==etsiDecoder::ETSI_DECODED_CPM_NOGN) {
			if (decodeCPM(decoded_data,msg,on_msg_timestamp_us,main_bf,m_client_id,PO_vec)==false) {
				return retval;
			}
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
			} else {
				//no Misbehaviour detected, insert into ldm
				for (ldmmap::vehicleData_t PO_data:PO_vec) {
					db_retval=m_db_ptr->insert(PO_data);
	
					if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
						std::cerr << "Warning! Insert on the database for Perceived Object " << (int) PO_data.stationID << "failed!" << std::endl;
					}
				}
			}
		} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_VAM || decoded_data.type==etsiDecoder::ETSI_DECODED_VAM_NOGN) {
			if (decodeVAM(decoded_data,msg,on_msg_timestamp_us,main_bf,m_client_id,vehdata)==false) {
				return retval;
			}
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
			} else {
				//no Misbehaviour detected, insert into ldm
				db_retval=m_db_ptr->insert(vehdata);
			}

		} else {
			std::cerr << "Warning! Message type not supported!" << std::endl;
			return retval;
		}
	} else {
        //if detection is disabled just decode and insert
		if (decoded_data.type==etsiDecoder::ETSI_DECODED_CAM || decoded_data.type==etsiDecoder::ETSI_DECODED_CAM_NOGN) {			
			if (decodeCAM(decoded_data,msg,on_msg_timestamp_us,main_bf,m_client_id,vehdata)==false) {
				return retval;
			}
			std::cout << "[DEBUG] Updating vehicle with stationID: " << vehdata.stationID << std::endl;
			db_retval=m_db_ptr->insert(vehdata);
			
			if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
				std::cerr << "Warning! Insert on the database for vehicle " << (int) vehdata.stationID << "failed!" << std::endl;
			}

		} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_CPM || decoded_data.type==etsiDecoder::ETSI_DECODED_CPM_NOGN) {
			if (decodeCPM(decoded_data,msg,on_msg_timestamp_us,main_bf,m_client_id,PO_vec)==false) {
				return retval;
			}

			for (ldmmap::vehicleData_t PO_data:PO_vec) {
				db_retval=m_db_ptr->insert(PO_data);

				if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
					std::cerr << "Warning! Insert on the database for Perceived Object " << (int) PO_data.stationID << "failed!" << std::endl;
				}
			}

		} else if (decoded_data.type==etsiDecoder::ETSI_DECODED_VAM || decoded_data.type==etsiDecoder::ETSI_DECODED_VAM_NOGN) {
			if (decodeVAM(decoded_data,msg,on_msg_timestamp_us,main_bf,m_client_id,vehdata)==false) {
				return retval;
			}

			db_retval=m_db_ptr->insert(vehdata);

			if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
				std::cerr << "Warning! Insert on the database for VRU " << (int) vehdata.stationID << "failed!" << std::endl;
			}
		} else {
			std::cerr << "Warning! Message type not supported!" << std::endl;
			return retval;
		}
    }

	return retval;
};

void MisbehaviourDetector::notifyOpTermination(uint64_t stationID) {
	m_already_reported_mutex.lock();
	m_already_reported.remove(stationID);
	m_already_reported_mutex.unlock();
}

// If this is a full ITS message manage the low frequency container data
// Check if this CAM contains the low frequency container
// and if the ext. lights hack for older versions CAMs is disable
// If yes, store the exterior lights status
// If not, check if an older information about the exterior lights of the current vehicle already exist in the database (using m_db_ptr->lookup()),
// if this data exists, use this data, if not, just set the exterior lights information as unavailable

inline ldmmap::OptionalDataItem<uint8_t>
MisbehaviourDetector::manage_LowfreqContainer(CAM_t *decoded_cam,uint32_t stationID){

          if(decoded_cam->cam.camParameters.lowFrequencyContainer!=NULL) {
                  // In any normal, uncorrupted CAM, buf should never be NULL and it should contain at least one element (i.e. buf[0] always exists)
                  if(decoded_cam->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency.exteriorLights.buf!=NULL) {
                          return ldmmap::OptionalDataItem<uint8_t>(decoded_cam->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency.exteriorLights.buf[0]);
                  } else {
                          // Data from a corrupted decoded CAM is considered as unavailable, for the time being
                          return ldmmap::OptionalDataItem<uint8_t>(false);
                  }
          } else {
                  ldmmap::LDMMap::returnedVehicleData_t retveh;

                  if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
                          return retveh.vehData.exteriorLights;
                  } else {
                          return ldmmap::OptionalDataItem<uint8_t>(false);
                  }
          }
}

bool MisbehaviourDetector::decodeCAM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id,ldmmap::vehicleData_t &vehdata) {

	uint64_t bf = 0.0,af = 0.0;
	uint64_t main_af = 0.0;

	CAM_t *decoded_cam = (CAM_t *) decodedData.decoded_msg;
	double lat = decoded_cam->cam.camParameters.basicContainer.referencePosition.latitude/10000000.0;
	double lon = decoded_cam->cam.camParameters.basicContainer.referencePosition.longitude/10000000.0;
	uint32_t stationID = decoded_cam->header.stationID;
	double l_inst_period=0.0;

	uint32_t stationTypeID = decoded_cam->cam.camParameters.basicContainer.stationType;

	// After getting the lat and lon values from the CAM, check if it is inside the S-LDM full coverage area,
	// using the areaFilter module (which can access the command line options, thus also the coverage area
	// specified by the user)
	if(m_logfile_name!="") {
		bf=get_timestamp_ns();
	}

	if(m_areaFilter.isInside(lat,lon)==false) {
		return false;
	}

	if(m_logfile_name!="") {
		af=get_timestamp_ns();

		//fprintf(m_logfile_file,"[LOG - AREA FILTER (Client %s)] ProcTimeMilliseconds=%.6lf\n",m_client_id.c_str(),(af-bf)/1000000.0);
	}

	if(m_logfile_name!="") {
		bf=get_timestamp_ns();
	}

	// Update the database
	ldmmap::LDMMap::LDMMap_error_t db_retval;
	
	uint64_t gn_timestamp;
	if(decodedData.type == etsiDecoder::ETSI_DECODED_CAM) {
		gn_timestamp = decodedData.gnTimestamp;
		vehdata.exteriorLights = manage_LowfreqContainer (decoded_cam,stationID);

		// Since on 5G-CARMEN only some specific vehicles use the complete GN+BTP+CAM messages
		if(m_opts_ptr->interop_hijack_enable){
		// when the enable-interop-hijack option is true, we save all passengerCars=5 as specificCategoryVehicle1=100
		if(static_cast<ldmmap::e_StationTypeLDM>(decoded_cam->cam.camParameters.basicContainer.stationType)==ldmmap::StationType_LDM_passengerCar){
			vehdata.stationType = ldmmap::StationType_LDM_specificCategoryVehicle1;
			}
		// when the enable-interop-hijack option is true, we save all StationType_LDM_unknown=0 without changes to not interfere with the map_render
		else{
			vehdata.stationType = static_cast<ldmmap::e_StationTypeLDM>(decoded_cam->cam.camParameters.basicContainer.stationType);
		}
			}
		else{
		//If enable-interop-hijack option is false, store stationType as usual
		vehdata.stationType = static_cast<ldmmap::e_StationTypeLDM>(decoded_cam->cam.camParameters.basicContainer.stationType);
			}


	// There is no need for an else if(), as we can enter here only if the decoded message type is either ETSI_DECODED_CAM or ETSI_DECODED_CAM_NOGN
	} else {
		if(msg.properties().size()>0) {
			proton::scalar gn_timestamp_prop = msg.properties().get(options_string_pop(m_opts_ptr->gn_timestamp_property));

						// If the gn_timestamp property is available, check if its type is correct
						if(gn_timestamp_prop.type() == proton::LONG) {
								gn_timestamp = static_cast<uint64_t>(proton::get<long>(gn_timestamp_prop));

								// If the gn_timestamp property is there and the ext_lights_hijack is enabled we know this is a bmw message so we check for
								// the hijacked highFreqContainer to extract the exterior lights information instead of the lowfreqContainer
								// The agreed encoding for the hijack of the driveDirection is:
								/*                            DriveDirection = 0 -----> No exterior lights on.
																DriveDirection = 1 -----> Right turn signal on.
																DriveDirection = 2 -----> Left turn signal on.
									*/
								if(m_opts_ptr->ext_lights_hijack_enable){
									uint8_t ext_lights = 0;
									// To emulate the encoding found in the lowfreqContainer we place 1<<(1+3=4) for leftTurnSignalOn, and 1<<(2+3=5) for rightTurnSignalOn
									if(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection != 0)
										ext_lights |= 1 << (decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection + 3);

									vehdata.exteriorLights = ldmmap::OptionalDataItem<uint8_t>(ext_lights);

									} else {
									vehdata.exteriorLights = manage_LowfreqContainer (decoded_cam,stationID);
									}
						} else {
								gn_timestamp=UINT64_MAX; // Set to an impossible value, to understand it is not specified (not set to zero beacuse is a possible correct value).
								fprintf(stdout,"[WARNING] Current message contains no GN and a not supported gn_timestamp property, ageCheck disabled\n");
						}
				} else {
						gn_timestamp=UINT64_MAX;
						fprintf(stdout,"[WARNING] Current message contains no GN and no gn_timestamp property, ageCheck disabled\n");
				}

				vehdata.stationType = static_cast<ldmmap::e_StationTypeLDM>(decoded_cam->cam.camParameters.basicContainer.stationType);
			}



	// Check the age of the data store inside the database (if the age check is enabled / -g option not specified)
	// before updating it with the new receive data
	if(m_opts_ptr->ageCheck_enabled == true && gn_timestamp != UINT64_MAX) {
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
			// According to the standard: GNTimestamp = (TAI timestamp since 2004-01-01 00:00:00) % 4294967296
			// Due to the modulo operation, it is not enough to consider the difference between the received GNTimestamp and the one
			// stored in the database, as this may cause issues when receiving data and the GNTimestamp values are cyclically reset
			// at the same time
			// We thus check the "gap" between the received numbers. Let's consider for instance: stored=4294967291, rx=3
			// In this case the "rx" data is the most up-to-date, but a cyclical reset occurred
			// We can then compute gap = 3 - 4294967291 = -429467288 < -300000 (-5 minutes) - ok! We keep this data even if 3 < 4294967291
			// Let's consider instead:
			// stored=3, rx=4294967291
			// In this case 'rx' is not the most up to date data (it is impossible to have '3' stored in the database and then receive
			// '4294967291', unless clock jumps occur in the car, bacause after all that time the data corresponding to '3' would have already
			// been garbage cleaned from the database)
			// We can then compute gap = 4294967291 - 3 = 429467288 > 300000 (5 minutes) - no! We should dicard the data we just received
			// Let's consider now a "normal" scenario:
			// stored=3, rx=114
			// gap = 114 - 3 = 111 < 300000 - ok! The data is kept (it would be discarded only if gap > 300000)
			// Finally, let's briefly analyze a final scenario:
			// stored=4294967292, rx=4294967291
			// It is evident how the rx data should be discarded because older than the stored one
			// gap = rx - stored = 4294967291 - 4294967292 = -1 > -300000 (-5 minutes) - The data is correctly discarded due to the second
			// condition in the if() clause
			long long int gap = static_cast<long long int>(gn_timestamp)-static_cast<long long int>(retveh.vehData.gnTimestamp);

			if((gn_timestamp>retveh.vehData.gnTimestamp && gap>300000) ||
				(gn_timestamp<retveh.vehData.gnTimestamp && gap>-300000)) {
				if(m_logfile_name!="") {
					//fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] Message discarded (data is too old). Rx = %lu, Stored = %lu, Gap = %lld\n",
					//	m_client_id.c_str(),
					//	gn_timestamp,retveh.vehData.gnTimestamp,gap);
					return false;
				}
			}
		}
	}

	vehdata.lon = lon;
	vehdata.lat = lat;
	vehdata.timestamp_us = get_timestamp_us();
	vehdata.on_msg_timestamp_us = on_msg_timestamp_us;
	vehdata.elevation = decoded_cam->cam.camParameters.basicContainer.referencePosition.altitude.altitudeValue/100.0;
	vehdata.heading = decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue/10.0;
	vehdata.speed_ms = decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue/100.0;
	vehdata.gnTimestamp = gn_timestamp;
	vehdata.stationID = stationID; // It is very important to save also the stationID
	vehdata.camTimestamp = static_cast<long>(decoded_cam->cam.generationDeltaTime);
	vehdata.stationType = static_cast<ldmmap::e_StationTypeLDM>(decoded_cam->cam.camParameters.basicContainer.stationType);

	// Save also the source vehicle quadkey
	if(msg.properties().size()>0) {
		proton::scalar quadkey_prop = msg.properties().get("quadkeys");

		if(quadkey_prop.type() == proton::STRING) {
			vehdata.sourceQuadkey = proton::get<std::string>(quadkey_prop);
		} else {
			vehdata.sourceQuadkey="";
		}
	} else {
		vehdata.sourceQuadkey="";
	}

	if(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth != VehicleWidth_unavailable) {
		vehdata.vehicleWidth = ldmmap::OptionalDataItem<long>(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth*100);
	} else {
		vehdata.vehicleWidth = ldmmap::OptionalDataItem<long>(false);
	}

	if(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue != VehicleLengthValue_unavailable) {
		vehdata.vehicleLength = ldmmap::OptionalDataItem<long>(decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue*100);
	} else {
		vehdata.vehicleLength = ldmmap::OptionalDataItem<long>(false);
	}

	/*
	// Manage the low frequency container data
	// Check if this CAM contains the low frequency container
	// and if the ext. lights hack for older versions CAMs is disable
	// If yes, store the exterior lights status
	// If not, check if an older information about the exterior lights of the current vehicle already exist in the database (using m_db_ptr->lookup()),
	// if this data exists, use this data, if not, just set the exterior lights information as unavailable
	if(decoded_cam->cam.camParameters.lowFrequencyContainer!=NULL) {
		// In any normal, uncorrupted CAM, buf should never be NULL and it should contain at least one element (i.e. buf[0] always exists)
		if(decoded_cam->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency.exteriorLights.buf!=NULL) {
			vehdata.exteriorLights = ldmmap::OptionalDataItem<uint8_t>(decoded_cam->cam.camParameters.lowFrequencyContainer->choice.basicVehicleContainerLowFrequency.exteriorLights.buf[0]);
		} else {
			// Data from a corrupted decoded CAM is considered as unavailable, for the time being
			vehdata.exteriorLights = ldmmap::OptionalDataItem<uint8_t>(false);
		}
	} else {
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
			vehdata.exteriorLights = retveh.vehData.exteriorLights;
		} else {
			vehdata.exteriorLights = ldmmap::OptionalDataItem<uint8_t>(false);
		}
	}
	*/

	// If logging is enabled, compute also an "instantaneous update period" metric (i.e., how much time has passed between two consecutive vehicle updates)
	if(m_logfile_name!="") {
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
			l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
		} else {
			l_inst_period=-1.0;
		}
		
	}

	// Old insert without any check
	//std::cout << "[DEBUG] Updating vehicle with stationID: " << vehdata.stationID << std::endl;
	//
	//db_retval=m_db_ptr->insert(vehdata);
	//
	//if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
	//	std::cerr << "Warning! Insert on the database for vehicle " << (int) stationID << "failed!" << std::endl;
	//}

	if(m_logfile_name!="") {
		af=get_timestamp_ns();

		fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] LowFrequencyContainerAvail=%d InsertReturnValue=%d ProcTimeMilliseconds=%.6lf\n",
			m_client_id.c_str(),
			decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.vehicleWidth != VehicleWidth_unavailable,
			db_retval,
			(af-bf)/1000000.0);
	}

	if(m_logfile_name!="") {
		bf=get_timestamp_ns();
	}

	// If a trigger manager has been enabled, check if any triggering condition has occurred (for the time being, only a simple trigger manager based on turn indicators has been developed)
	if(m_indicatorTrgMan_enabled == true && vehdata.exteriorLights.isAvailable()) {
		// Trigger either if the cross-border trigger mode is enabled or if the triggering vehicle is located inside the internal area of this S-LDM instance
		if(m_opts_ptr->cross_border_trigger==true || m_areaFilter.isInsideInternal(lat,lon)==true) {
			// if the interop-hijack is enabled, check the stationType before triggering
			if(m_opts_ptr->interop_hijack_enable){
			if(vehdata.stationType == ldmmap::StationType_LDM_passengerCar){
				if(m_indicatorTrgMan_ptr->checkAndTrigger(lat,lon,stationID,vehdata.exteriorLights.getData()) == true) {
					std::cout << "[TRIGGER] Triggering condition detected!" << std::endl;
					}
				}
				}
			else{
				// if the interop-hijack is NOT enabled, trigger no matter the vehicleType
				if(m_indicatorTrgMan_ptr->checkAndTrigger(lat,lon,stationID,vehdata.exteriorLights.getData()) == true) {
					std::cout << "[TRIGGER] Triggering condition detected!" << std::endl;
				}
				}
		}
	}

	if(m_logfile_name!="") {
		af=get_timestamp_ns();

		fprintf(m_logfile_file,"[LOG - TRIGGER CHECK (Client %s)] TriggerEnabled=%d ExteriorLightsAvail=%d CrossBrdTriggerMode=%d IsInsideInternalArea=%d ProcTimeMilliseconds=%.6lf\n",
			m_client_id.c_str(),
			m_indicatorTrgMan_enabled,
			vehdata.exteriorLights.isAvailable(),
			m_opts_ptr->cross_border_trigger,
			m_areaFilter.isInsideInternal(lat,lon),
			(af-bf)/1000000.0);
	}

	ASN_STRUCT_FREE(asn_DEF_CAM,decoded_cam);

	if(m_logfile_name!="") {
		main_af=get_timestamp_ns();

		logfprintf(m_logfile_file,std::string("FULL CAM PROCESSING (Client") + m_client_id + std::string(")"),"StationID=%u StationTypeID=%d Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf"
			" CAMTimestamp=%ld GNTimestamp=%lu CAMTimestampDiff=%ld GNTimestampDiff=%ld"
			" ProcTimeMilliseconds=%.6lf Cardinality=%d\n",
			stationID,static_cast<int>(vehdata.stationType),lat,lon,
			vehdata.heading,
			l_inst_period,
			vehdata.camTimestamp,vehdata.gnTimestamp,get_timestamp_ms_cam()-vehdata.camTimestamp,get_timestamp_ms_gn()-vehdata.gnTimestamp,
			(main_af-main_bf)/1000000.0,m_db_ptr->getCardinality ());
		
		// fprintf(m_logfile_file,"[LOG - FULL CAM PROCESSING] StationID=%u Coordinates=%.7lf:%.7lf InstUpdatePeriod=%.3lf"
		// 	" CAMTimestamp=%ld GNTimestamp=%lu CAMTimestampDiff=%ld GNTimestampDiff=%ld"
		// 	" ProcTimeMilliseconds=%.6lf\n",
		// 	stationID,lat,lon,
		// 	l_inst_period,
		// 	vehdata.camTimestamp,vehdata.gnTimestamp,get_timestamp_ms_cam()-vehdata.camTimestamp,get_timestamp_ms_gn()-vehdata.gnTimestamp,
		// 	(main_af-main_bf)/1000000.0);	
	}

	return true;
}

bool MisbehaviourDetector::decodeCPM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id,std::vector<ldmmap::vehicleData_t> &PO_vec) {

	uint64_t bf = 0.0,af = 0.0;
	uint64_t main_af = 0.0;

	uint64_t fromStationID;

	CPM_t *decoded_cpm = (CPM_t *) decodedData.decoded_msg;
	double fromLat = (double) (decoded_cpm->payload.managementContainer.referencePosition.latitude / 10000000.0);
	double fromLon = (double) (decoded_cpm->payload.managementContainer.referencePosition.longitude / 10000000.0);

	fromStationID = (uint64_t) decoded_cpm->header.stationID;

	double l_inst_period=0.0;

	if(m_logfile_name!="") {
	    bf=get_timestamp_ns();
	}

	int wrappedContainer_size = decoded_cpm->payload.cpmContainers.list.count;
	for (int i=0; i<wrappedContainer_size; i++)
	{
	    WrappedCpmContainer_t wrappedContainer = (WrappedCpmContainer_t) *decoded_cpm->payload.cpmContainers.list.array[i];
	    WrappedCpmContainer__containerData_PR present = (WrappedCpmContainer__containerData_PR) wrappedContainer.containerData.present;
	    if(present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
	    {
	        PerceivedObjectContainer_t POcontainer = (PerceivedObjectContainer_t) wrappedContainer.containerData.choice.PerceivedObjectContainer;
	        int PObjects_size = POcontainer.perceivedObjects.list.count;
	        for(int j=0; j<PObjects_size;j++)
	        {
	            ldmmap::LDMMap::returnedVehicleData_t PO_ret_data;
	            PerceivedObject_t *PO_seq = (PerceivedObject_t *) POcontainer.perceivedObjects.list.array[j];

	            //Translate to ego vehicle coordinates
	            ldmmap::vehicleData_t PO_data;
	            PO_data.detected = true;
	            PO_data.vehicleLength = (long) PO_seq->objectDimensionX->value;
	            PO_data.vehicleWidth = (long) PO_seq->objectDimensionY->value;
	            PO_data.heading = (double) PO_seq->angles->zAngle.value / 10; // DECI constant was used
	            PO_data.xSpeed = (long) PO_seq->velocity->choice.cartesianVelocity.xVelocity.value;
	            PO_data.xSpeed = (long) PO_seq->velocity->choice.cartesianVelocity.yVelocity.value;
	            PO_data.speed_ms = (sqrt (pow(PO_data.xSpeed,2) +
	                                     pow(PO_data.ySpeed,2)))/ 100; // CENTI constant was used

	            double lonPO, latPO, from_x,from_y,xDistance, yDistance;
	            double gammar=0;
	            double kr=0;
	            xDistance = (double) PO_seq->position.xCoordinate.value/100;
	            yDistance = (double) PO_seq->position.yCoordinate.value/100;
	            transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
	            TransverseMercator_Forward(&tmerc, fromLon, fromLat, fromLon, &from_x, &from_y, &gammar, &kr);
	            from_x += xDistance;
	            from_y += yDistance;
	            TransverseMercator_Reverse(&tmerc, fromLon, from_x, from_y, &latPO, &lonPO, &gammar, &kr);
	            PO_data.lat = latPO;
	            PO_data.lon = lonPO;

				//asn_INTEGER2long conversion helper used for referenceTime TimestampIts_t to long
				long referenceTime;
				asn_INTEGER2long(&decoded_cpm->payload.managementContainer.referenceTime,&referenceTime);
	            
				PO_data.camTimestamp = referenceTime - (long) PO_seq->measurementDeltaTime;

	            PO_data.perceivedBy = (long) decoded_cpm->header.stationID;
	            PO_data.stationType = ldmmap::StationType_LDM_detectedPassengerCar;

	            if(m_recvCPMmap[fromStationID].find((long)PO_seq->objectId) == m_recvCPMmap[fromStationID].end()){
	                // First time we have received this object from this vehicle
	                //If PO id is already in local copy of LDM
	                if(m_db_ptr->lookup( (long)PO_seq->objectId, PO_ret_data) == ldmmap::LDMMap::LDMMAP_OK)
	                {
	                    // We need a new ID for object
	                    std::set<uint64_t> IDs;
	                    m_db_ptr->getAllIDs (IDs);
	                    int newID = 1;
	                    for (int num : IDs) {
	                        if (num == newID) {
	                            ++newID;
	                        } else if (num > newID) {
	                            break;
	                        }
	                    }
	                    //Update recvCPMmap
	                    m_recvCPMmap[fromStationID][ (long) PO_seq->objectId] = newID;
	                    PO_data.stationID = newID;
	                }
	                else
	                {
	                    //Update recvCPMmap
	                    m_recvCPMmap[fromStationID][ (long) PO_seq->objectId] =  (long) PO_seq->objectId;
	                    PO_data.stationID =  (long) PO_seq->objectId;
	                }
	            }
	            else
	            {
	                PO_data.stationID = m_recvCPMmap[fromStationID][ (long) PO_seq->objectId];
	            }

	            ldmmap::LDMMap::LDMMap_error_t db_retval;

	            uint64_t gn_timestamp;
	            if(decodedData.type == etsiDecoder::ETSI_DECODED_CPM) {
	                gn_timestamp = decodedData.gnTimestamp;
	                // There is no need for an else if(), as we can enter here only if the decoded message type is either ETSI_DECODED_CAM or ETSI_DECODED_CAM_NOGN
	            } else {
	                gn_timestamp=UINT64_MAX;
	                fprintf(stdout,"[WARNING] Current message contains no GN timestamp, ageCheck disabled.\n");
	            }

	            // Check the age of the data store inside the database (if the age check is enabled / -g option not specified)
	            // before updating it with the new receive data
	            if(gn_timestamp != UINT64_MAX) {
	                ldmmap::LDMMap::returnedVehicleData_t retveh;

	                if(m_db_ptr->lookup(PO_data.stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
	                    /* According to the standard: GNTimestamp = (TAI timestamp since 2004-01-01 00:00:00) % 4294967296
	                    // Due to the modulo operation, it is not enough to consider the difference between the received GNTimestamp and the one
	                    // stored in the database, as this may cause issues when receiving data and the GNTimestamp values are cyclically reset
	                    // at the same time
	                    // We thus check the "gap" between the received numbers. Let's consider for instance: stored=4294967291, rx=3
	                    // In this case the "rx" data is the most up-to-date, but a cyclical reset occurred
	                    // We can then compute gap = 3 - 4294967291 = -429467288 < -300000 (-5 minutes) - ok! We keep this data even if 3 < 4294967291
	                    // Let's consider instead:
	                    // stored=3, rx=4294967291
	                    // In this case 'rx' is not the most up to date data (it is impossible to have '3' stored in the database and then receive
	                    // '4294967291', unless clock jumps occur in the car, bacause after all that time the data corresponding to '3' would have already
	                    // been garbage cleaned from the database)
	                    // We can then compute gap = 4294967291 - 3 = 429467288 > 300000 (5 minutes) - no! We should dicard the data we just received
	                    // Let's consider now a "normal" scenario:
	                    // stored=3, rx=114
	                    // gap = 114 - 3 = 111 < 300000 - ok! The data is kept (it would be discarded only if gap > 300000)
	                    // Finally, let's briefly analyze a final scenario:
	                    // stored=4294967292, rx=4294967291
	                    // It is evident how the rx data should be discarded because older than the stored one
	                    // gap = rx - stored = 4294967291 - 4294967292 = -1 > -300000 (-5 minutes) - The data is correctly discarded due to the second
	                    // condition in the if() clause
	                     */
	                    long long int gap = static_cast<long long int>(gn_timestamp)-static_cast<long long int>(retveh.vehData.gnTimestamp);

	                    if((gn_timestamp>retveh.vehData.gnTimestamp && gap>300000) ||
	                       (gn_timestamp<retveh.vehData.gnTimestamp && gap>-300000)) {
	                        if(m_logfile_name!="") {
	                            fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] Message discarded (data is too old). Rx = %lu, Stored = %lu, Gap = %lld\n",
	                                    m_client_id.c_str(),
	                                    gn_timestamp,retveh.vehData.gnTimestamp,gap);
	                            return false;
	                        }
	                    }
	                }
	            }

	            PO_data.timestamp_us = get_timestamp_us();
	            memcpy(PO_data.macaddr,&(decodedData.GNaddress[0])+2,6); // Save the vehicle MAC address into the database (the MAC address is stored in the last 6 Bytes of the GN Address)

	            // Retrieve, if available, the information on the RSSI for the vehicle corresponding to the MAC address of the sender
	            // This is the RSSI on the CPM dissemination interface
	            // Uncomment this to retrieve the RSSI via iw, instead of using nl80211 (requires iw to be available in the system)
	            // PO_data.rssi_dBm=get_rssi_from_iw(PO_data.macaddr,std::string(m_opts_ptr->dissemination_device.c_str()));
	            
				// DISATTIVATO RSSI
				//if(m_nl_sock_info.sock_valid==true) {
	            //    PO_data.rssi_dBm = get_rssi_from_netlink(PO_data.macaddr, m_nl_sock_info);
	            //} else {
	            //    PO_data.rssi_dBm = RSSI_UNAVAILABLE;
	            //}

	            if(m_logfile_name!="") {
	                ldmmap::LDMMap::returnedVehicleData_t retveh;

	                if(m_db_ptr->lookup(PO_data.stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
	                    l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
	                } else {
	                    l_inst_period=-1.0;
	                }

	                // If a pointer to a VDPGPSClient has been specified, log also the current position of the receiver
	                //DISATTIVATO VDPGS
					//std::pair<double,double> latlon;
	                //if(m_gpsc_ptr!=nullptr) {
	                //    latlon = m_gpsc_ptr->getCurrentPositionDbl();
	                //}

	                if(m_logfile_name!="") {
	                    logfprintf(m_logfile_file,std::string("FULL CPM Perceived Object PROCESSING (Client ") + m_client_id + std::string(")"),"StationID=%u Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf"
	                                                                                                                           " MAC_Addr=%02X:%02X:%02X:%02X:%02X:%02X"
	                                                                                                                           " RSSI=%.2lf",
	                               PO_data.stationID,PO_data.lat,PO_data.lon,
	                               PO_data.heading,
	                               l_inst_period,
	                               PO_data.macaddr[0],PO_data.macaddr[1],PO_data.macaddr[2],PO_data.macaddr[3],PO_data.macaddr[4],PO_data.macaddr[5],
	                               PO_data.rssi_dBm);
	                }

					// DISATTIVATO VDPGS
	                //if(m_gpsc_ptr!=nullptr) {
	                //    fprintf(m_logfile_file," ReceiverCoordinates=%.7lf:%.7lf\n",latlon.first,latlon.second);
	                //} else {
	                //    fprintf(m_logfile_file,"\n");
	                //}
	            }
				PO_vec.emplace_back(PO_data);
	        }
	    }
	}
	return true;
}

bool MisbehaviourDetector::decodeVAM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id, ldmmap::vehicleData_t &vehdata) {
	
	uint64_t bf = 0.0,af = 0.0;
	uint64_t main_af = 0.0;
	VAM_t *decoded_vam = (VAM_t *) decodedData.decoded_msg;

	double lat, lon;
	uint64_t stationID;
	
	lat = decoded_vam->vam.vamParameters.basicContainer.referencePosition.latitude/10000000.0;
	lon = decoded_vam->vam.vamParameters.basicContainer.referencePosition.longitude/10000000.0;
	stationID = decoded_vam->header.stationID;
	
	double l_inst_period=0.0;

	if(m_logfile_name!="") {
		bf=get_timestamp_ns();
	}

	// Update the database
	ldmmap::LDMMap::LDMMap_error_t db_retval;
	
	uint64_t gn_timestamp;
	if(decodedData.type == etsiDecoder::ETSI_DECODED_VAM) {
		gn_timestamp = decodedData.gnTimestamp;
		// There is no need for an else if(), as we can enter here only if the decoded message type is either ETSI_DECODED_VAM or ETSI_DECODED_VAM_NOGN
	} else {
		gn_timestamp=UINT64_MAX;
		fprintf(stdout,"[WARNING] Current message contains no GN timestamp, ageCheck disabled.\n");
	}
	
	// Check the age of the data store inside the database (if the age check is enabled / -g option not specified)
	// before updating it with the new receive data
	if(gn_timestamp != UINT64_MAX){
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
			long long int gap = static_cast<long long int>(gn_timestamp)-static_cast<long long int>(retveh.vehData.gnTimestamp);

			if((gn_timestamp>retveh.vehData.gnTimestamp && gap>300000) ||
				(gn_timestamp<retveh.vehData.gnTimestamp && gap>-300000)) {
				if(m_logfile_name!="") {
					fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] Message discarded (data is too old). Rx = %lu, Stored = %lu, Gap = %lld\n",
						m_client_id.c_str(),
						gn_timestamp,retveh.vehData.gnTimestamp,gap);
					return false;
				}
			}
		}
	}
	
	vehdata.lon = lon;
	vehdata.lat = lat;
	vehdata.timestamp_us = get_timestamp_us();
	
	vehdata.elevation = decoded_vam->vam.vamParameters.basicContainer.referencePosition.altitude.altitudeValue/100.0;

	if(decoded_vam->vam.vamParameters.vruHighFrequencyContainer.heading.value==Wgs84AngleValue_unavailable) {
		vehdata.heading = LDM_HEADING_UNAVAILABLE;
	} else {
		vehdata.heading = decoded_vam->vam.vamParameters.vruHighFrequencyContainer.heading.value/10.0;
	}
	

	vehdata.speed_ms = decoded_vam->vam.vamParameters.vruHighFrequencyContainer.speed.speedValue/100.0;
	vehdata.camTimestamp = static_cast<long>(decoded_vam->vam.generationDeltaTime);
	vehdata.stationType = static_cast<ldmmap::e_StationTypeLDM>(decoded_vam->vam.vamParameters.basicContainer.stationType);
	
	vehdata.gnTimestamp = gn_timestamp;
	vehdata.stationID = stationID; // It is very important to save also the stationID
	memcpy(vehdata.macaddr,&(decodedData.GNaddress[0])+2,6); // Save the vehicle MAC address into the database (the MAC address is stored in the last 6 Bytes of the GN Address)
	
	// Retrieve, if available, the information on the RSSI for the vehicle corresponding to the MAC address of the sender
	// This is the RSSI on the VAM dissemination interface
	// Uncomment this to retrieve the RSSI via iw, instead of using nl80211 (requires iw to be available in the system)
	// vehdata.rssi_dBm=get_rssi_from_iw(vehdata.macaddr,std::string(m_opts_ptr->dissemination_device.c_str()));
	
	// DISATTIVATO RSSI
	//if(m_nl_sock_info.sock_valid==true) {
	//	vehdata.rssi_dBm = get_rssi_from_netlink(vehdata.macaddr, m_nl_sock_info);
	//} else {
	//	vehdata.rssi_dBm = RSSI_UNAVAILABLE;
	//}
	
	vehdata.vehicleWidth = ldmmap::OptionalDataItem<long>(false);
	vehdata.vehicleLength = ldmmap::OptionalDataItem<long>(false);
	
	// If logging is enabled, compute also an "instantaneous update period" metric (i.e., how much time has passed between two consecutive vehicle updates)
	if(m_logfile_name!="") {
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookup(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
			l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
		} else {
			l_inst_period=-1.0;
		}

		// If a pointer to a VDPGPSClient has been specified, log also the current position of the receiver
		// DISATTIVATO VDPGS
		//std::pair<double,double> latlon;
		//if(m_gpsc_ptr!=nullptr) {
		//	latlon = m_gpsc_ptr->getCurrentPositionDbl();
		//}

		if(m_logfile_name!="") {
			logfprintf(m_logfile_file,std::string("FULL VAM PROCESSING (Client ") + m_client_id + std::string(")"),"StationID=%u Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf"
				" MAC_Addr=%02X:%02X:%02X:%02X:%02X:%02X"
				" RSSI=%.2lf",
				stationID,lat,lon,
				vehdata.heading,
				l_inst_period,
				vehdata.macaddr[0],vehdata.macaddr[1],vehdata.macaddr[2],vehdata.macaddr[3],vehdata.macaddr[4],vehdata.macaddr[5],
				vehdata.rssi_dBm);
		}

		// DISATTIVATO VDPGS
		//if(m_gpsc_ptr!=nullptr) {
		//	fprintf(m_logfile_file," ReceiverCoordinates=%.7lf:%.7lf\n",latlon.first,latlon.second);
		//} else {
		//	fprintf(m_logfile_file,"\n");
		//}
	}
	
	ASN_STRUCT_FREE(asn_DEF_VAM,decoded_vam);

	return true;
}

void MisbehaviourDetector::updateLastMessage(ldmmap::vehicleData_t vehdata) {
	if (m_lastMessageCache.count(vehdata.stationID)==1) {
		m_lastMessageCache[vehdata.stationID]=vehdata;
	} else {
		m_lastMessageCache.emplace(vehdata.stationID,vehdata);
	}
}

uint64_t MisbehaviourDetector::checkCAMFreq(ldmmap::vehicleData_t vehdata) {
	uint64_t MB_CODE=0;
	//if present otherwise no way to check frequency
	if (m_lastMessageCache.count(vehdata.stationID)==1) {
		//checks if difference in timestamp is less than 100ms then freq>10Hz not allowed
		if (vehdata.camTimestamp-m_lastMessageCache[vehdata.stationID].camTimestamp<100) {
			MB_CODE=0x1;
		}
	}
	return MB_CODE;
}