#include "AMQPclient.h"
#include "QuadKeyTS.h"
#include "utils.h"
#include <fstream>
#include <iomanip>
#include <proton/reconnect_options.hpp>
#include <time.h>

#include "Seq.hpp"
#include "SequenceOf.hpp"
#include "asn_utils.h"
#include "utmuts.h"

extern "C" {
	#include "CAM.h"
	#include "DENM.h"
	#include "CPM.h"
	#include "VAM.h"
}

std::atomic<bool> eventMapModified(false);
namespace {
	// Example custom function to configure an AMQP filter,
	// specifically an APACHE.ORG:SELECTOR
	// (http://www.amqp.org/specification/1.0/filters)

	void set_filter(proton::source_options &opts, const std::string& selector_str) {

		proton::source::filter_map map;
		proton::symbol filter_key("selector");
		proton::value filter_value;


		// The value is a specific AMQP "described type": binary string with symbolic descriptor
		proton::codec::encoder enc(filter_value);
		enc << proton::codec::start::described()
			<< proton::symbol("apache.org:selector-filter:string")
			<< selector_str
			<< proton::codec::finish();

		// In our case, the map has this one element
		map.put(filter_key, filter_value);
		opts.filters(map);
	}
}

// If this is a full ITS message manage the low frequency container data
// Check if this CAM contains the low frequency container
// and if the ext. lights hack for older versions CAMs is disable
// If yes, store the exterior lights status
// If not, check if an older information about the exterior lights of the current vehicle already exist in the database (using m_db_ptr->lookupVehicle()),
// if this data exists, use this data, if not, just set the exterior lights information as unavailable

inline ldmmap::OptionalDataItem<uint8_t>
AMQPClient::manage_LowfreqContainer(CAM_t *decoded_cam,uint32_t stationID){

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

                  if(m_db_ptr->lookupVehicle(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
                          return retveh.vehData.exteriorLights;
                  } else {
                          return ldmmap::OptionalDataItem<uint8_t>(false);
                  }
          }
}

void 
AMQPClient::on_connection_open(proton::connection &conn) {
	if(m_logfile_name!="" && m_logfile_file!=nullptr) {
		fprintf(m_logfile_file,"[LOG - AMQPClient %s] Connection successfully established.\n",m_client_id.c_str());
		fflush(m_logfile_file);
	}
}

void 
AMQPClient::on_connection_close(proton::connection &conn) {
	if(m_logfile_name!="" && m_logfile_file!=nullptr) {
		fprintf(m_logfile_file,"[LOG - AMQPClient %s] Connection closed.\n",m_client_id.c_str());
		fflush(m_logfile_file);
	}
}

void 
AMQPClient::on_container_start(proton::container &c) {
	m_cont.store(&c);

	proton::source_options opts;
	proton::connection_options co;
	bool co_set = false;

	// std::vector<std::string> quadKeys;
	// std::vector<std::string> fromfile;
	// QuadKeys::QuadKeyTS tilesys;

	// Set the connection options
	if(!m_username.empty()) {
		co.user(m_username);
		co_set = true;

		std::cout<<"[AMQPClient " << m_client_id.c_str() << "] AMQP username successfully set: "<<m_username<<std::endl;
	}

	if(!m_password.empty()) {
		co.password(m_password);
		co_set = true;

		std::cout<<"[AMQPClient " << m_client_id.c_str() << "] AMQP password successfully set."<<std::endl;
	}

	if(m_reconnect == true) {
		co.reconnect(proton::reconnect_options());
		co_set = true;

		std::cout<<"[AMQPClient " << m_client_id.c_str() << "] AMQP automatic reconnection enabled."<<std::endl;
	}

	if(m_allow_sasl == true) {
		co.sasl_enabled(true);
		co_set = true;

		std::cout<<"[AMQPClient " << m_client_id.c_str() << "] AMQP SASL enabled."<<std::endl;
	}

	if(m_allow_insecure == true) {
		co.sasl_allow_insecure_mechs(true);
		co_set = true;

		std::cout<<"[AMQPClient " << m_client_id.c_str() << "] Warning: clear-text passwords are enabled."<<std::endl;
	}

	if(m_idle_timeout_ms>=0) {
		if(m_idle_timeout_ms==0) {
			co.idle_timeout(proton::duration::FOREVER);

			std::cout<<"[AMQPClient " << m_client_id.c_str() << "] Idle timeout set to FOREVER."<<std::endl;
		} else {
			co.idle_timeout(proton::duration(m_idle_timeout_ms));

			std::cout<<"[AMQPClient " << m_client_id.c_str() << "] Idle timeout set to "<<m_idle_timeout_ms<<"."<<std::endl;
		}

		co_set = true;
	} else {
		std::cout<<"[AMQPClient " << m_client_id.c_str() << "] No idle timeout has been explicitely set."<<std::endl;
	}

	if(m_logfile_name!="") {
		if(m_logfile_name=="stdout") {
			m_logfile_file=stdout;
		} else {
			// Opening the output file in write + append mode just to be safe in case the user does not change the file name
			// between different executions of the S-LDM
			m_logfile_file=fopen(m_logfile_name.c_str(),"wa");
		}
	}

	/* First version of the code without the caching mechanism. Kept here for reference. */
	/*
		//LevelOfDetail set into the tilesys class as private variables
		tilesys.setLevelOfDetail(levelOfdetail);

		std::cout << "[AMQP Client] Quadkey algoritm started." << std::endl;

		//here we get the vector containing all the quadkeys in the range at a given level of detail
		quadKeys = tilesys.LatLonToQuadKeyRange(min_latitude, max_latitude, min_longitude, max_longitude);
		//Quadkeys unifier algorithm
		tilesys.unifyQuadkeys(quadKeys);

		//Here we create a string to pass to the filter (SQL like)
		std::string s;

		for(size_t i = 0; i < quadKeys.size(); i++) {
			s.insert(s.length(), "quadkeys LIKE ''");
			//l = s.length() - 1;
			s.insert(s.length() - 1, quadKeys.at(i));
			s.insert(s.length() - 1, "%");
			if(i < quadKeys.size()-1){
				s.insert(s.length(), " OR ");
			}
		}

		std::cout << "[AMQP Client] Quadkey algoritm correctly terminated." << std::endl;

		// Set the AMQP filter
		set_filter(opts, s);
	*/

	/* Second version of the code without the caching mechanism. Kept here for reference. */
	// This code has been moved into a dedicated function inside QuadKeyTS.h/.cpp (getQuadKeyFilter())
	//
	// std::string line;
	// bool cache_file_found = false;
	// std::ifstream ifile("cachefile.sldmc");
	// uint64_t bf = 0.0,af = 0.0;

	// if(m_logfile_name!="") {
	// 	bf=get_timestamp_ns();
	// }

	// if(ifile.is_open()) {
	// 	std::cout<<"[AMQPClient " << m_client_id.c_str() << "] Cache file available: reading the parameters..."<< std::endl;

	// 	while(getline(ifile, line)) {
	// 		fromfile.push_back(line);
	// 	}

	// 	double minlatff = stod(fromfile.at(0));
	// 	double maxlatff = stod(fromfile.at(1));
	// 	double minlonff = stod(fromfile.at(2));
	// 	double maxlonff = stod(fromfile.at(3));

	// 	// fprintf(stdout,"From File we get max_latitude: %.40lf\n",maxlatff);
	// 	// fprintf(stdout,"Actual max_latitude parameter%.40lf\n",max_latitude);
	// 	// fprintf(stdout,"From File we get min_latitude: %.40lf\n",minlatff);
	// 	// fprintf(stdout,"Actual min_latitude parameter%.40lf\n",min_latitude);

	// 	if(doublecomp(minlatff, min_latitude) && doublecomp(maxlatff, max_latitude) && doublecomp(minlonff, min_longitude) && doublecomp(maxlonff, max_longitude) && fromfile.size() > 4){
	// 		cache_file_found = true;
	// 	}
	// } else {
	// 	std::cout<<"[AMQPClient " << m_client_id.c_str() << "] No cache file found!"<<std::endl;
	// }

	// ifile.close();

	// if(cache_file_found == false) {
	// 	std::ofstream ofile("cachefile.sldmc");

	// 	std::cout<<"[AMQPClient " << m_client_id.c_str() << "] New coordinates: recomputing quadkeys..."<<std::endl;
	// 	//LevelOfDetail set into the tilesys class as private variables
	// 	tilesys.setLevelOfDetail(levelOfdetail);
	// 	// Here we get the vector containing all the quadkeys in the range at a given level of detail
	// 	quadKeys = tilesys.LatLonToQuadKeyRange(min_latitude, max_latitude, min_longitude, max_longitude);

	// 	// Add the range information to the cache file
	// 	if(ofile.is_open()) {
	// 		ofile << std::fixed << std::setprecision(6) << min_latitude << "\n" << max_latitude << "\n" << min_longitude << "\n" << max_longitude << "\n";
	// 	}

	// 	// Quadkeys unifier algorithm
	// 	// tilesys.unifyQuadkeys(quadKeys);
	// 	quadKeys=tilesys.unifyQuadkeys2(quadKeys);
	// 	tilesys.checkdim(quadKeys);

	// 	// Write the computed Quadkeys to the cache file
	// 	std::ofstream file;
	// 	if(ofile.is_open()) {
	// 		for(size_t i = 0; i < quadKeys.size(); i++){
	// 			ofile << quadKeys.at(i) << "\n";
	// 		}
	// 	}

	// 	ofile.close();

	// 	std::cout<<"[AMQP Client " << m_client_id.c_str() << "] Finished: Quadkey cache file created."<<std::endl;

	// 	// Here we create a string to pass to the filter (SQL like)
	// 	std::string s;

	// 	for(size_t i = 0; i < quadKeys.size(); i++) {
	// 		s.insert(s.length(), "quadkeys LIKE ''");
	// 		//l = s.length() - 1;
	// 		s.insert(s.length() - 1, quadKeys.at(i));
	// 		s.insert(s.length() - 1, "%");
	// 		if(i < quadKeys.size()-1){
	// 			s.insert(s.length(), " OR ");
	// 		}
	// 	}

	// 	// Set the AMQP filter
	// 	set_filter(opts, s);
	// } else {
	// 	std::cout<<"[AMQPClient " << m_client_id.c_str() << "] Filter setup from a cache file... "<<std::endl;

	// 	std::string s;

	// 	for(size_t i = 4; i < fromfile.size(); i++) {
	// 		s.insert(s.length(), "quadkeys LIKE ''");
	// 		//l = s.length() - 1;
	// 		s.insert(s.length() - 1, fromfile.at(i));
	// 		s.insert(s.length() - 1, "%");
	// 		if(i < fromfile.size()-1){
	// 			s.insert(s.length(), " OR ");
	// 		}
	// 	}

	// 	// Set the AMQP filter
	// 	set_filter(opts, s);
	// }

	// if(m_logfile_name!="") {
	// 	af=get_timestamp_ns();

	// 	fprintf(m_logfile_file,"[LOG - AMQP STARTUP (Client %s)] Area=%.7lf:%.7lf-%.7lf:%7lf QKCacheFileFound=%d ProcTimeMilliseconds=%.6lf\n",
	// 		m_client_id.c_str(),
	// 		min_latitude,min_longitude,max_latitude,max_longitude,
	// 		cache_file_found,(af-bf)/1000000.0);
	// }

	if(m_quadKey_filter!="") {
		set_filter(opts,m_quadKey_filter);

		std::cout << "[AMQPClient " << m_client_id.c_str() << "] QuadKey filter successfully set." << std::endl;
	} else {
		std::cout << "[AMQPClient " << m_client_id.c_str() << "] QuadKey filter not set. Any message will be received." << std::endl;
	}

	std::cout << "[AMQPClient " << m_client_id.c_str() << "] Connecting to AMQP broker at: " << conn_url_ << std::endl;

	proton::connection conn;
	if(co_set == true) {
		std::cout << "[AMQPClient " << m_client_id.c_str() << "] Connecting with user-defined connection options." << std::endl;
		conn = c.connect(conn_url_,co);
	} else {
		std::cout << "[AMQPClient " << m_client_id.c_str() << "] Connecting with default connection options." << std::endl;
		conn = c.connect(conn_url_);
	}

	if(m_quadKey_filter!="") {
		conn.open_receiver(addr_, proton::receiver_options().source(opts));
	} else{
		conn.open_receiver(addr_);
	}
}

void 
AMQPClient::on_message(proton::delivery &d, proton::message &msg) {

	uint64_t on_msg_timestamp_us = get_timestamp_us();

	etsiDecoder::etsiDecodedData_t decodedData;

	uint64_t bf = 0.0,af = 0.0;
	uint64_t main_bf = 0.0,main_af = 0.0;

	if(m_logfile_name!="") {
		main_bf=get_timestamp_ns();

		// This additional log line has been commented out to avoid being too verbose
		// fprintf(m_logfile_file,"[NEW MESSAGE RX]\n");
	}

	if(m_printMsg == true) {
		std::cout << msg.body() << std::endl;
	}

	proton::codec::decoder qpid_decoder(msg.body());
	proton::binary message_bin;
	uint8_t *message_bin_buf;

	// Check if a binary message has been received
	// If no binary data has been received, just ignore the current AMQP message
	if(qpid_decoder.next_type () == proton::BINARY) {
		qpid_decoder >> message_bin;

		message_bin_buf=message_bin.data ();
	} else {
		// This message should be ignored
		if(m_printMsg == true) {
			std::cerr << "Error: received a message in a non-binary AMQP type." << std::endl;
		}
		return;
	}

	if(m_logfile_name!="") {
		bf=get_timestamp_ns();
	}

	Security::Security_error_t sec_retval;
	storedCertificate_t certificateData;
	// Decode the content of the message, using the decoder-module frontend class
	// m_decodeFrontend.setPrintPacket(true); // <- uncomment to print the bytes of each received message. Should be used for debug only, and should be kept disabled when deploying the S-LDM.
	if(m_decodeFrontend.decodeEtsi(message_bin_buf, message_bin.size (), decodedData, sec_retval,certificateData, etsiDecoder::decoderFrontend::MSGTYPE_AUTO)!=ETSI_DECODER_OK) {
		std::cerr << "Error! Cannot decode ETSI packet!" << std::endl;
		return;
	} else {
		if (!certificateData.digest.empty()) {
			m_certStore_ptr->insert_or_assign(certificateData.digest,certificateData);
		}
	}

	if(m_logfile_name!="") {
		af=get_timestamp_ns();

		fprintf(m_logfile_file,"[LOG - MESSAGE DECODER (Client %s)] ProcTimeMilliseconds=%.6lf\n",m_client_id.c_str(),(af-bf)/1000000.0);
	}

	ldmmap::vehicleData_t vehdata;
	vehicleDataVector_t(PO_vec);
	ldmmap::LDMMap::LDMMap_error_t db_retval;
	uint64_t MBD_retval;

	if (decodedData.type==etsiDecoder::ETSI_DECODED_CAM || decodedData.type==etsiDecoder::ETSI_DECODED_CAM_NOGN) {			
		if (decodeCAM(decodedData,msg,on_msg_timestamp_us,main_bf,m_client_id,vehdata)==false) {
			return;
		}

		certificateData.stationID=vehdata.stationID;
		certificateData.msg_timestamp=vehdata.gnTimestamp;
		if (sec_retval!=Security::SECURITY_OK) {
			switch (sec_retval) {
				// to be determined when to call MBD and what its actions will be
				// m_MBDetector_ptr .createReport() ?
				case Security::SECURITY_VERIFICATION_FAILED:
					// at the moment it's either for bad decode (e.g. DENM not supported) or no certificates present
					break;
				case Security::SECURITY_INVALID_CERTIFICATE:
					// certificate verification failed
					std::cout <<"\n\nCERTIFICATE INVALID\n";
					break;
				case Security::SECURITY_DIGEST:
					std::cout <<"\n\nSECURITY ";
					if (m_certStore_ptr->isValid(certificateData.digest)!=e_DigestValid_retval::DIGEST_OK) {
						//digest invalid can use switch for each case instead
						std::cout <<"DIGEST INVALID\n";
					} else {std::cout <<"DIGEST VALID\n";}
					break;
			}
		}

		if (m_MBDetection_enabled==true) {
			MBD_retval=m_MBDetector_ptr->processMessage(decodedData,vehdata,PO_vec);
			if (MBD_retval!=0) {
				std::cerr <<"[WARNING] Misbehaviour detected for vehicle " <<vehdata.stationID <<". Message discarded with MB_CODE " <<MBD_retval <<std::endl;
				return;
			}
		}

		std::cout << "[DEBUG] Updating vehicle with stationID: " << vehdata.stationID << std::endl;
		db_retval=m_db_ptr->insertVehicle(vehdata);				
		
		if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
			std::cerr << "[WARNING] Insert on the database for vehicle " <<vehdata.stationID << "failed!" << std::endl;
		}

	} else if (decodedData.type==etsiDecoder::ETSI_DECODED_DENM || decodedData.type==etsiDecoder::ETSI_DECODED_DENM_NOGN) {
		if (decodeDENM(decodedData,msg,on_msg_timestamp_us,main_bf,m_client_id,vehdata)==false) {
			return;
		}



	} else if (decodedData.type==etsiDecoder::ETSI_DECODED_CPM || decodedData.type==etsiDecoder::ETSI_DECODED_CPM_NOGN) {
		if (decodeCPM(decodedData,msg,on_msg_timestamp_us,main_bf,m_client_id,PO_vec)==false) {
			return;
		}

		if (m_MBDetection_enabled==true) {
			MBD_retval=m_MBDetector_ptr->processMessage(decodedData,vehdata,PO_vec);
			if (MBD_retval!=0) {
				std::cerr <<"[WARNING] Misbehaviour detected for vehicle " <<vehdata.stationID <<". Message discarded with MB_CODE " <<MBD_retval <<std::endl;
				return;
			}
		}

		for (ldmmap::vehicleData_t PO_data:PO_vec) {
			db_retval=m_db_ptr->insertVehicle(PO_data);

			if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
				std::cerr << "[WARNING] Insert on the database for Perceived Object " <<PO_data.stationID << "failed!" << std::endl;
			}
		}

	} else if (decodedData.type==etsiDecoder::ETSI_DECODED_VAM || decodedData.type==etsiDecoder::ETSI_DECODED_VAM_NOGN) {
		if (decodeVAM(decodedData,msg,on_msg_timestamp_us,main_bf,m_client_id,vehdata)==false) {
			return;
		}

		if (m_MBDetection_enabled==true) {
			MBD_retval=m_MBDetector_ptr->processMessage(decodedData,vehdata,PO_vec);
			if (MBD_retval!=0) {
				std::cerr <<"[WARNING] Misbehaviour detected for vehicle " <<vehdata.stationID <<". Message discarded with MB_CODE " <<MBD_retval <<std::endl;
				return;
			}
		}

		db_retval=m_db_ptr->insertVehicle(vehdata);

		if(db_retval!=ldmmap::LDMMap::LDMMAP_OK && db_retval!=ldmmap::LDMMap::LDMMAP_UPDATED) {
			std::cerr << "[WARNING] Insert on the database for VRU " <<vehdata.stationID << "failed!" << std::endl;
		}
	} else {
		std::cerr << "[WARNING] Message type not supported!" << std::endl;
		return;
	}
	return;
}

void 
AMQPClient::on_container_stop(proton::container &c) {
	if(m_logfile_name!="" && m_logfile_name!="stdout") {
		fclose(m_logfile_file);
	}
}

bool AMQPClient::decodeCAM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id,ldmmap::vehicleData_t &vehdata) {

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

		if(m_db_ptr->lookupVehicle(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
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
	if (decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue!=HeadingValue_unavailable) {
		vehdata.heading = decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.heading.headingValue/10.0;
	} else {
		vehdata.heading = ldmmap::e_DataUnavailableValue::heading;
	}
	if (decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue!=SpeedValue_unavailable) {
		vehdata.speed_ms = decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.speed.speedValue/100.0;
	} else {
		vehdata.speed_ms=ldmmap::e_DataUnavailableValue::speed;
	}
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

	//INSERIMENTO VALORI AGGIUNTIVI
	if (decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue!=LongitudinalAccelerationValue_unavailable) {
		vehdata.longitudinalAcceleration=decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.longitudinalAcceleration.longitudinalAccelerationValue/10.0;
	} else {
		vehdata.longitudinalAcceleration=ldmmap::e_DataUnavailableValue::longitudinalAcceleration;
	}
	if (decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue!=CurvatureValue_unavailable) {
		vehdata.curvature=decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.curvature.curvatureValue;
	} else {
		vehdata.curvature=ldmmap::e_DataUnavailableValue::curvature;
	}
	if (decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection!=DriveDirection_unavailable) {
		vehdata.driveDirection=decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.driveDirection;
	} else {
		vehdata.driveDirection=ldmmap::e_DataUnavailableValue::driveDirection;
	}
	if (decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue!=YawRateValue_unavailable) {
		vehdata.yawRate=decoded_cam->cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency.yawRate.yawRateValue/10.0;
	} else {
		vehdata.yawRate=ldmmap::e_DataUnavailableValue::yawRate;
	}
	/*
	// Manage the low frequency container data
	// Check if this CAM contains the low frequency container
	// and if the ext. lights hack for older versions CAMs is disable
	// If yes, store the exterior lights status
	// If not, check if an older information about the exterior lights of the current vehicle already exist in the database (using m_db_ptr->lookupVehicle()),
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

		if(m_db_ptr->lookupVehicle(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
			vehdata.exteriorLights = retveh.vehData.exteriorLights;
		} else {
			vehdata.exteriorLights = ldmmap::OptionalDataItem<uint8_t>(false);
		}
	}
	*/

	// If logging is enabled, compute also an "instantaneous update period" metric (i.e., how much time has passed between two consecutive vehicle updates)
	if(m_logfile_name!="") {
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookupVehicle(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
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
			(main_af-main_bf)/1000000.0,m_db_ptr->getVehicleCardinality ());
		
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

bool AMQPClient::decodeDENM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id,ldmmap::vehicleData_t &vehdata) {
	std::cout << " \nSTART DENM" << std::endl;//per test

	uint64_t main_af = 0.0;
	uint64_t bf_InsideArea = 0;
	uint64_t af_InsideArea = 0;
	uint64_t bf_updateDatabaseDENM = 0;
	uint64_t af_updateDatabaseDENM = 0;
	uint64_t bf_lookupAndUpdateEvent_time = 0;
	uint64_t af_lookupAndUpdateEvent_time = 0;
	uint64_t lookupAndUpdateEvent_time = 0;
	uint64_t bf_insertEvent_time = 0;
	uint64_t af_insertEvent_time = 0;
	uint64_t af_databaseTimestamp_ns = 0;
	uint64_t bf_databaseTimestamp_ns = 0;
	uint64_t insertEvent_time = 0;

	DENM_t *decoded_denm = (DENM_t *) decodedData.decoded_msg;
	double lat = decoded_denm->denm.management.eventPosition.latitude/10000000.0;
	double lon = decoded_denm->denm.management.eventPosition.longitude/10000000.0;
	double ele = decoded_denm->denm.management.eventPosition.altitude.altitudeValue/100.0;
	uint64_t originatingStationID = decoded_denm->denm.management.actionID.originatingStationID;
	uint64_t sequenceNumber = decoded_denm->denm.management.actionID.sequenceNumber;
	uint32_t stationTypeID = decoded_denm->denm.management.stationType;
	m_logfile_name="stdout";
	m_logfile_file=stdout;

	if (m_logfile_name!=""){
		bf_InsideArea = get_timestamp_ns();
	}

	if(m_areaFilter.isInside(lat,lon)==false) {

		return false;
	} else {
		std::cout << "Inside Area Filter" << std::endl;//per test
	}

	if(m_logfile_name!="") {
		af_InsideArea = get_timestamp_ns();

		fprintf(m_logfile_file,"[LOG - AREA FILTER (Client %s)] ProcTimeMilliseconds=%.6lf\n",m_client_id.c_str(),(af_InsideArea - bf_InsideArea)/1000000.0);
	}

	if(m_logfile_name!="") {
		bf_updateDatabaseDENM = get_timestamp_ns();
	}

	// Update the database
	ldmmap::LDMMap::returnedEventData_t retEvent;
	ldmmap::eventData_t eveData;
	ldmmap::LDMMap::event_LDMMap_error_t db_everetval;
	uint64_t nearUpdateEvent_key = 0;

	eveData.gnTimestampDENM = decodedData.gnTimestamp*1e3; // Convert to microseconds

	//Management container
	//ActionID
	eveData.on_msg_timestampDENM_us = on_msg_timestamp_us;
	eveData.insertEventTimestamp_us = get_timestamp_us();
	eveData.originatingStationID = originatingStationID;
	eveData.sequenceNumber = sequenceNumber;

	eveData.eventTermination = ldmmap::OptionalDataItem<uint64_t>(decoded_denm->denm.management.termination);
	eveData.detectionTime = get_timestamp_us();
	eveData.referenceTime = get_timestamp_us();
	eveData.eventLatitude = lat;
	eveData.eventLongitude = lon;
	eveData.eventElevation = ele;
	eveData.eventRelevanceDistance = ldmmap::OptionalDataItem<double> (decoded_denm->denm.management.relevanceDistance);
	eveData.eventRelevanceTrafficDirection = ldmmap::OptionalDataItem<double> (decoded_denm->denm.management.relevanceTrafficDirection);
	if (decoded_denm->denm.management.validityDuration != nullptr) {
		eveData.eventValidityDuration = 10;
	} else {
		eveData.eventValidityDuration = 10; //default value
	}

	eveData.eventTransmissionInterval = ldmmap::OptionalDataItem<uint64_t>(decoded_denm->denm.management.transmissionInterval);
	eveData.eventStationType = stationTypeID;

	// Situation container
	if (decoded_denm->denm.situation != nullptr) {

		// If the eventType is not available, set the cause code to unknown
		if(decoded_denm->denm.situation->eventType.causeCode != -1) {
			ldmmap::e_EventTypeLDM CauseCode = static_cast<ldmmap::e_EventTypeLDM>(decoded_denm->denm.situation->eventType.causeCode);
			eveData.eventCauseCode = CauseCode;
		} else {
			eveData.eventCauseCode = ldmmap::EventType_LDM_unknown;
		}
	} else {
		eveData.eventCauseCode = ldmmap::EventType_LDM_unknown;
	}

	uint64_t keyEvent = m_db_ptr->KEY_EVENT(lat,lon,ele,eveData.eventCauseCode);

	if (!eveData.eventTermination.isAvailable()) {
		bf_lookupAndUpdateEvent_time = get_timestamp_ns();
		db_everetval = m_db_ptr->lookupAndUpdateEvent(keyEvent,eveData.eventLatitude,eveData.eventLongitude,
			eveData.eventCauseCode,eveData,retEvent, nearUpdateEvent_key);
		af_lookupAndUpdateEvent_time = get_timestamp_ns();
		lookupAndUpdateEvent_time = af_lookupAndUpdateEvent_time - bf_lookupAndUpdateEvent_time;
		if (db_everetval == 6) {
			//std::cout <<"EVENT NOT FOUND" << std::endl; //For test
		} else if (db_everetval == 2 || db_everetval == 3) {
			eventMapModified.store(true);
			std::cout << "Updated Near Event Key: " << nearUpdateEvent_key << std::endl;
		}
		//std::cout <<"Result of lookupAndUpdate = " << db_everetval << std::endl; //For test
		if (db_everetval == ldmmap::LDMMap::event_LDMMAP_ITEM_NOT_FOUND) {
			bf_insertEvent_time	= get_timestamp_ns();
			db_everetval = m_db_ptr->insertEvent(eveData,keyEvent);
			af_insertEvent_time = get_timestamp_ns();
			insertEvent_time = af_insertEvent_time - bf_insertEvent_time;
			eventMapModified.store(true);
			//std::cout <<"INSERT EVENT with KEY: " <<keyEvent << std::endl; //For test
		}
	} else {
		db_everetval = m_db_ptr->removeEvent(keyEvent);
		eventMapModified.store(true);
		//std::cout <<"REMOVE EVENT with KEY: " <<keyEvent << std::endl; //For test
	}

	if(m_logfile_name!="") {
		af_updateDatabaseDENM = get_timestamp_ns();

		fprintf(m_logfile_file,"[LOG - DATABASE UPDATE (Client %s)] FunctionReturnValue=%d ProcTimeMilliseconds=%.6lf\n",
			m_eventclient_id.c_str(),
			db_everetval, (af_updateDatabaseDENM - bf_updateDatabaseDENM)/1e3);
	}

	if(m_logfile_name!="") {
		main_af = get_timestamp_ns();

		logfprintf(m_logfile_file,std::string("FULL DENM PROCESSING (Client") + m_eventclient_id + std::string(")"), "eventKey=%lu"
			"OriginatingStationID=%u sequenceNumber=%d Coordinates=%.7lf:%.7lf:%.2lf eventStationType=%lf"
			" DENM_ReferenceTime=%ld GNTimestamp=%lu DENM_DetectionTime=%ld DENM_ValidityDuration=%d ProcTimeMilliseconds=%.6lf EventCardinality=%d\n",
			keyEvent,originatingStationID,sequenceNumber,lat,lon,ele,
			eveData.eventStationType,eveData.referenceTime,eveData.gnTimestampDENM,eveData.detectionTime, eveData.eventValidityDuration,
			(main_af-main_bf)/1e6,m_db_ptr->getEventCardinality ());

	}else {
		std::cerr << "Warning! Problem to print a DENM!" << std::endl;
	}
	//std::cout <<"END OF DENM\n" << std::endl; //For test

	// Write the DENM timestamps to a CSV file
	std::ofstream file("DENMtimestamps_25v.csv", std::ios::app);
	if (!file) {
		std::cerr << "Errore nell'apertura del file!" << std::endl;
		return false;
	}

	// If the file is empty, write the header
	static bool first_time = true;
	if (first_time) {
		file << "TotalMessageProcessingTimeDENM [us],DatabaseUpdateDENM [us],"
		"LookupAndUpdateEvent_Time [us],InsertEvent_Time [us],DecodedMessageProcessingTime [us]\n";
		first_time = false;
	}

	// Write the timestamps to the file
	file << (main_af-main_bf)/1e3 << "," << (af_updateDatabaseDENM-bf_updateDatabaseDENM)/1e3 << ","
	<< lookupAndUpdateEvent_time/1e3 << "," << insertEvent_time/1e3 << "," << (main_af-eveData.gnTimestampDENM)/1e3 << "\n";

	file.close(); // Close the file

	/*std::ofstream file1("DENM.csv", std::ios::app);
	if (!file1) {
		std::cerr << "Errore nell'apertura del file!" << std::endl;
		return;
	}

	// File per
	static bool first_time1 = true;
	if (first_time1) {
		file1 << "KEY_EVENT, near Event Key, Operation, Latitude, Longitude, Elevation, Cause_Code\n";
		first_time = false;
	}

	// Converte key_event in una stringa
	std::ostringstream key_event_str;
	key_event_str << keyEvent;

	// Imposta precisione numerica
	// Scrivi i dati nel file CSV
	file << std::fixed << std::setprecision(20) << keyEvent << ", " // key_event, senza notazione scientifica
			<< nearUpdateEvent_key << ", " // near_event_key, senza notazione scientifica
			<< db_everetval << ", " // operazione
			<< std::fixed << std::setprecision(6)  // Formattiamo i numeri decimali (lat, lon, ele) con 6 decimali
			<< lat << ", "
			<< lon << ", "
			<< ele << ", "
			<< eveData.eventCauseCode << "\n";// Scrive l'evento di causa come intero

	file1.close(); // Chiudi il file*/
	return true;
}

bool AMQPClient::decodeCPM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id,std::vector<ldmmap::vehicleData_t> &PO_vec) {

	uint64_t bf = 0.0,af = 0.0;
	uint64_t main_af = 0.0;

	uint64_t fromStationID;

	CPM_t *decoded_cpm = (CPM_t *) decodedData.decoded_msg;
	double fromLat = asn1cpp::getField(decoded_cpm->payload.managementContainer.referencePosition.latitude, double) / 10000000.0;
	double fromLon = asn1cpp::getField(decoded_cpm->payload.managementContainer.referencePosition.longitude, double) / 10000000.0;

	fromStationID = asn1cpp::getField(decoded_cpm->header.stationID, uint64_t);

	double l_inst_period=0.0;

	if(m_logfile_name!="") {
	    bf=get_timestamp_ns();
	}

	int wrappedContainer_size = asn1cpp::sequenceof::getSize(decoded_cpm->payload.cpmContainers);
	for (int i=0; i<wrappedContainer_size; i++)
	{
		auto wrappedContainer = asn1cpp::sequenceof::getSeq(decoded_cpm->payload.cpmContainers,WrappedCpmContainer,i);
		WrappedCpmContainer__containerData_PR present = asn1cpp::getField(wrappedContainer->containerData.present,WrappedCpmContainer__containerData_PR);
	    if(present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
	    {
	        auto POcontainer = asn1cpp::getSeq(wrappedContainer->containerData.choice.PerceivedObjectContainer,PerceivedObjectContainer);
			int PObjects_size = asn1cpp::sequenceof::getSize(POcontainer->perceivedObjects);
	        for(int j=0; j<PObjects_size;j++)
	        {
	            ldmmap::LDMMap::returnedVehicleData_t PO_ret_data;
				auto PO_seq = asn1cpp::makeSeq(PerceivedObject);
				PO_seq = asn1cpp::sequenceof::getSeq(POcontainer->perceivedObjects,PerceivedObject,j);

				//Translate to ego vehicle coordinates
				ldmmap::vehicleData_t PO_data;
				PO_data.detected = true;
				PO_data.vehicleLength = asn1cpp::getField(PO_seq->objectDimensionX->value,long);
				PO_data.vehicleWidth = asn1cpp::getField(PO_seq->objectDimensionY->value,long);
				PO_data.heading = asn1cpp::getField(PO_seq->angles->zAngle.value,double) / DECI;
				PO_data.xSpeed = asn1cpp::getField(PO_seq->velocity->choice.cartesianVelocity.xVelocity.value,long);
				PO_data.xSpeed = asn1cpp::getField(PO_seq->velocity->choice.cartesianVelocity.yVelocity.value,long);
				PO_data.speed_ms = (sqrt (pow(PO_data.xSpeed,2) +
											pow(PO_data.ySpeed,2)))/CENTI;

				double lonPO, latPO, from_x,from_y,xDistance, yDistance;
				double gammar=0;
				double kr=0;
				xDistance = asn1cpp::getField(PO_seq->position.xCoordinate.value,double)/100;
				yDistance = asn1cpp::getField(PO_seq->position.yCoordinate.value,double)/100;
				transverse_mercator_t tmerc = UTMUPS_init_UTM_TransverseMercator();
				TransverseMercator_Forward(&tmerc, fromLon, fromLat, fromLon, &from_x, &from_y, &gammar, &kr);
				from_x += xDistance;
				from_y += yDistance;
				TransverseMercator_Reverse(&tmerc, fromLon, from_x, from_y, &latPO, &lonPO, &gammar, &kr);
				PO_data.lat = latPO;
				PO_data.lon = lonPO;
				PO_data.camTimestamp = static_cast<long>(asn1cpp::getField(decoded_cpm->payload.managementContainer.referenceTime,long)) - static_cast<long>(asn1cpp::getField(PO_seq->measurementDeltaTime,long));
				PO_data.perceivedBy = asn1cpp::getField(decoded_cpm->header.stationID,long);
				PO_data.stationType = ldmmap::StationType_LDM_detectedPassengerCar;

				if(m_recvCPMmap[fromStationID].find(asn1cpp::getField(PO_seq->objectId,long)) == m_recvCPMmap[fromStationID].end()){
					// First time we have received this object from this vehicle
					//If PO id is already in local copy of LDM
					if(m_db_ptr->lookupVehicle(asn1cpp::getField(PO_seq->objectId,long),PO_ret_data) == ldmmap::LDMMap::LDMMAP_OK)
					{
						// We need a new ID for object
						std::set<uint64_t> IDs;
						m_db_ptr->getAllIDsVehicles (IDs);
						int newID = 1;
						for (int num : IDs) {
							if (num == newID) {
								++newID;
							} else if (num > newID) {
								break;
							}
						}
						//Update recvCPMmap
						m_recvCPMmap[fromStationID][asn1cpp::getField(PO_seq->objectId,long)] = newID;
						PO_data.stationID = newID;
					}
					else
					{
						//Update recvCPMmap
						m_recvCPMmap[fromStationID][asn1cpp::getField(PO_seq->objectId,long)] = asn1cpp::getField(PO_seq->objectId,long);
						PO_data.stationID = asn1cpp::getField(PO_seq->objectId,long);
					}
				}
				else
				{
					PO_data.stationID = m_recvCPMmap[fromStationID][asn1cpp::getField(PO_seq->objectId,long)];
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
	            if(m_opts_ptr->ageCheck_enabled == true && gn_timestamp != UINT64_MAX) {
	                ldmmap::LDMMap::returnedVehicleData_t retveh;

	                if(m_db_ptr->lookupVehicle(PO_data.stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
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

	            if(m_logfile_name!="") {
	                ldmmap::LDMMap::returnedVehicleData_t retveh;

	                if(m_db_ptr->lookupVehicle(PO_data.stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
	                    l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
	                } else {
	                    l_inst_period=-1.0;
	                }

	                if(m_logfile_name!="") {
	                    logfprintf(m_logfile_file,std::string("FULL CPM Perceived Object PROCESSING (Client ") + m_client_id + std::string(")"),"StationID=%u Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf\n",
	                               PO_data.stationID,PO_data.lat,PO_data.lon,
	                               PO_data.heading,
	                               l_inst_period);
	                }
	            }
				PO_vec.emplace_back(PO_data);
	        }
	    }
	}
	return true;
}

bool AMQPClient::decodeVAM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id, ldmmap::vehicleData_t &vehdata) {
	
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

	if(m_areaFilter.isInside(lat,lon)==false) {
		return false;
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
	if(m_opts_ptr->ageCheck_enabled == true && gn_timestamp != UINT64_MAX){
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookupVehicle(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
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
	
	vehdata.vehicleWidth = ldmmap::OptionalDataItem<long>(false);
	vehdata.vehicleLength = ldmmap::OptionalDataItem<long>(false);
	
	// If logging is enabled, compute also an "instantaneous update period" metric (i.e., how much time has passed between two consecutive vehicle updates)
	if(m_logfile_name!="") {
		ldmmap::LDMMap::returnedVehicleData_t retveh;

		if(m_db_ptr->lookupVehicle(stationID,retveh)==ldmmap::LDMMap::LDMMAP_OK) {
			l_inst_period=(get_timestamp_us()-retveh.vehData.timestamp_us)/1000.0;
		} else {
			l_inst_period=-1.0;
		}

		if(m_logfile_name!="") {
			logfprintf(m_logfile_file,std::string("FULL VAM PROCESSING (Client ") + m_client_id + std::string(")"),"StationID=%u Coordinates=%.7lf:%.7lf Heading=%.1lf InstUpdatePeriod=%.3lf\n",
				stationID,lat,lon,
				vehdata.heading,
				l_inst_period);
		}
	}
	
	ASN_STRUCT_FREE(asn_DEF_VAM,decoded_vam);

	return true;
}