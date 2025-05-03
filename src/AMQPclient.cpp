#include "AMQPclient.h"
#include "QuadKeyTS.h"
#include "utils.h"
#include <fstream>
#include <iomanip>
#include <proton/reconnect_options.hpp>
#include <time.h>

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

	// Decode the content of the message, using the decoder-module frontend class
	// m_decodeFrontend.setPrintPacket(true); // <- uncomment to print the bytes of each received message. Should be used for debug only, and should be kept disabled when deploying the S-LDM.
	if(m_decodeFrontend.decodeEtsi(message_bin_buf, message_bin.size (), decodedData, etsiDecoder::decoderFrontend::MSGTYPE_AUTO)!=ETSI_DECODER_OK) {
		std::cerr << "Error! Cannot decode ETSI packet!" << std::endl;
		return;
	}

	if(m_logfile_name!="") {
		af=get_timestamp_ns();

		fprintf(m_logfile_file,"[LOG - MESSAGE DECODER (Client %s)] ProcTimeMilliseconds=%.6lf\n",m_client_id.c_str(),(af-bf)/1000000.0);
	}

	// Send packet to misbehaviour detector
	m_MBDetector_ptr->processMessage(decodedData,msg,on_msg_timestamp_us,main_bf,m_client_id);
	return;
}

void 
AMQPClient::on_container_stop(proton::container &c) {
	if(m_logfile_name!="" && m_logfile_name!="stdout") {
		fclose(m_logfile_file);
	}
}
