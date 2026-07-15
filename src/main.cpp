#include <atomic>
#include <iostream>
#include <unistd.h>
#include <condition_variable>
#include <thread>
#include <time.h>
#include <vector>
#include <string>
#include <sys/wait.h>
#include <spawn.h>
#include "curl/curl.h"

#include "LDMmap.h"
#include "vehicle-visualizer.h"
#include "QuadKeyTS.h"
#include "AMQPclient.h"
#include "JSONserver.h"
#include "utils.h"
#include "timers.h"
#include "frameBuffer.h"

extern "C" {
	#include "options.h"
	#include "CAM.h"
	#include "DENM.h"
}

#include "etsiDecoderFrontend.h"

#define DB_CLEANER_INTERVAL_SECONDS 1
#define DB_DELETE_OLDER_THAN_SECONDS 1 // This value should NEVER be set greater than (5-DB_CLEANER_INTERVAL_SECONDS/60) minutes or (300-DB_CLEANER_INTERVAL_SECONDS) seconds - doing so may break the database age check functionality!
#define GREEN_ESCAPE "\033[32m"
#define YELLOW_ESCAPE "\033[33m"
#define RED_ESCAPE "\033[31m"
#define RESET_ESCAPE "\033[0m"

#define MAX_VEHICLES_PER_FRAME 2000


// Global atomic flag to terminate all the threads in case of errors
std::atomic<bool> terminatorFlag;

// Global pointer to a visualizer object (to be accessed by both DBcleaner_callback() and VehVizUpdater_callback())
vehicleVisualizer* globVehVizPtr=nullptr;
// Global mutex (plus condition variable) to synchronize the threads using the object pointer defined above
std::mutex syncmtx;
std::condition_variable synccv;

// Global structure (plus related mutex) to store references to AMQPClient objects to terminate their connections in case of errors
std::unordered_map<int,AMQPClient*> amqpclimap;
std::mutex amqpclimutex;

void AMQPclient_t(ldmmap::LDMMap *db_ptr,options_t *opts_ptr,std::string logfile_name,std::string clientID,unsigned int clientIndex,indicatorTriggerManager *itm_ptr,std::string quadKey_filter,AMQPClient *main_amqp_ptr,MisbehaviourDetector *mbd_ptr,CertificateStore *certStore_ptr) {
	if(clientIndex >= MAX_ADDITIONAL_AMQP_CLIENTS-1) {
		fprintf(stderr,"[FATAL ERROR] Error: there is a bug in the code, which attemps to spawn too many AMQP clients.\nPlease report this bug to the developers.\n");
		fprintf(stderr,"Bug details: client id: %s - client index: %u - max supported clients: %u\n",clientID.c_str(),clientIndex,MAX_ADDITIONAL_AMQP_CLIENTS-1);
		terminatorFlag = true;
		return;
	}

	if(opts_ptr->amqp_broker_x[clientIndex].amqp_reconnect_after_local_timeout_expired==true) {
		std::cout << "[AMQPClient "<< clientID << "] This client will be restarted if a local idle timeout error occurs." << std::endl;
	}

	AMQPClient recvClient(std::string(options_string_pop(opts_ptr->amqp_broker_x[clientIndex].broker_url)), std::string(options_string_pop(opts_ptr->amqp_broker_x[clientIndex].broker_topic)), opts_ptr->min_lat,opts_ptr->max_lat, opts_ptr->min_lon, opts_ptr->max_lon, opts_ptr, db_ptr, logfile_name);

	// If this flag is set to true, the client will be restarted after an error, instead of being terminated
	bool cli_restart = false;

	do {
		cli_restart = false;

		try {
			// The indicator trigger manager is disabled by default in AMQPClient, unless it is explicitely enabled with a call to setIndicatorTriggerManager(true)
			if(opts_ptr->indicatorTrgMan_enabled==true) {
				recvClient.setIndicatorTriggerManager(itm_ptr);
			}

			// Set Misbehaviour Detector in any case, if disabled messages will just pass through
			recvClient.setMisbehaviourDetector(mbd_ptr);

			// Set username, if specified
			if(options_string_len(opts_ptr->amqp_broker_x[clientIndex].amqp_username)>0) {
				recvClient.setUsername(std::string(options_string_pop(opts_ptr->amqp_broker_x[clientIndex].amqp_username)));
			}

			// Set password, if specified
			if(options_string_len(opts_ptr->amqp_broker_x[clientIndex].amqp_password)>0) {
				recvClient.setPassword(std::string(options_string_pop(opts_ptr->amqp_broker_x[clientIndex].amqp_password)));
			}

			// Set connection options (they all default to "false" - see also options.c/broker_options_inizialize())
			recvClient.setConnectionOptions(opts_ptr->amqp_broker_x[clientIndex].amqp_allow_sasl,opts_ptr->amqp_broker_x[clientIndex].amqp_allow_insecure,opts_ptr->amqp_broker_x[clientIndex].amqp_reconnect);
			recvClient.setIdleTimeout(opts_ptr->amqp_broker_x[clientIndex].amqp_idle_timeout);

			recvClient.setClientID(clientID);

			// Set the QuadKey filter
			if(opts_ptr->quadkFilter_enabled==true) {
				recvClient.setFilter(quadKey_filter);
			}

			amqpclimutex.lock();
			amqpclimap[clientIndex]=&recvClient;
			amqpclimutex.unlock();

			proton::container(recvClient).run();
		} catch (const std::exception& e) {
			if(opts_ptr->amqp_broker_x[clientIndex].amqp_reconnect_after_local_timeout_expired==true && std::string(e.what()) == "amqp:resource-limit-exceeded: local-idle-timeout expired") {
				std::cerr << "[AMQPClient "<< clientID << "] Exception occurred: " << e.what() << std::endl;
				std::cout << "[AMQPClient "<< clientID << "] Attempting to restart the client after a local idle timeout expired error..." << std::endl;
				recvClient.force_container_stop();
				sleep(1);
				cli_restart = true;
			} else {
				amqpclimutex.lock();
				amqpclimap.erase(clientIndex);
				amqpclimutex.unlock();

				std::cerr << "[AMQPClient "<< clientID << "] Exception occurred: " << e.what() << std::endl;
				terminatorFlag = true;

				main_amqp_ptr->force_container_stop();
			}
		}
	} while(cli_restart==true);

	return;
}

typedef struct vizOptions {
	ldmmap::LDMMap *db_ptr;
	options_t *opts_ptr;
} vizOptions_t;

typedef struct nnModelUpdaterOptions {
	ldmmap::LDMMap *db_ptr;
	uint64_t period_ms;
	uint16_t max_frame_size;
	uint16_t pack_size;
	char *gnn_snapshot_path;
	double sumo_netoffset_x;
	double sumo_netoffset_y;
	char *gnn_csv_out_path;
	// other communication config parameters...
} nnModelUpdaterOptions_t;

void clearVisualizerObject(uint64_t id,void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

	vizObjPtr->sendObjectClean(std::to_string(id));
}

void clearEventVisualizerObject(uint64_t id,void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

	vizObjPtr->sendEventObjectClean(id);
}

struct cleanerArgs {
	ldmmap::LDMMap *db_ptr;
	CertificateStore *certStore_ptr;
	MisbehaviourDetector *mbd_ptr;
};
void *DBcleaner_callback(void *arg) {
	// Get the pointer to the database
	cleanerArgs *args = static_cast<cleanerArgs *>(arg);
	ldmmap::LDMMap *db_ptr = args->db_ptr;
	CertificateStore *certStore_ptr = args->certStore_ptr;
	MisbehaviourDetector *mbd_ptr = args->mbd_ptr;

	// Create a new timer
	Timer tmr(DB_CLEANER_INTERVAL_SECONDS*1e3);
	std::cout << "[INFO] Database cleaner started. The DB will be garbage collected every " << DB_CLEANER_INTERVAL_SECONDS << " seconds." << std::endl;

        if(tmr.start()==false) {
                std::cerr << "[ERROR] Fatal error! Cannot create timer for the DB cleaner thread!" << std::endl;
                terminatorFlag = true;
                pthread_exit(nullptr);
        }


	std::unique_lock<std::mutex> synclck(syncmtx);
	synccv.wait(synclck);

	POLL_DEFINE_JUNK_VARIABLE();

	// used for cleanups that need to be done less frequently than DB_CLEANER_INTERVAL_SECONDS
	int counter=0;

	while(terminatorFlag == false && tmr.waitForExpiration()==true) {
			// ---- These operations will be performed periodically ----

			counter+=DB_CLEANER_INTERVAL_SECONDS;
			// check every 30 seconds
			if (counter%30==0) {
				mbd_ptr->cleanupPendingEvents(); // cleanup for events that have been left pending
				// check every 10 minutes
				if (counter>=10*60) {
					certStore_ptr->deleteOlderThan(10*60*1e3); //removing certificates older than 10 minutes
					counter=0;
				}
			}
			db_ptr->deleteVehicleOlderThanAndExecute(DB_DELETE_OLDER_THAN_SECONDS*1e3,clearVisualizerObject,static_cast<void *>(globVehVizPtr));

			// Delete events older than the specified validity duration
			db_ptr->deleteEventOlderThanAndExecute(clearEventVisualizerObject,static_cast<void *>(globVehVizPtr));
			// --------
	}

	if(terminatorFlag == true) {
		std::cerr << "[WARN] Database cleaner terminated due to error." << std::endl;
	}

	pthread_exit(nullptr);
}

void updateVisualizer(ldmmap::vehicleData_t vehdata,void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

	vizObjPtr->sendObjectUpdate(std::to_string(vehdata.stationID),vehdata.lat,vehdata.lon,static_cast<int>(vehdata.stationType),vehdata.heading);
}

void updateEventVisualizer(ldmmap::eventData_t eveData,uint64_t key, void *vizObjVoidPtr) {
	vehicleVisualizer *vizObjPtr = static_cast<vehicleVisualizer *>(vizObjVoidPtr);

		vizObjPtr->sendEventObjectUpdate(key, eveData.eventLatitude, eveData.eventLongitude, eveData.eventElevation, eveData.eventCauseCode);
		//printf("EVENT_KEY (Visualizer): %lu\n", key);
}

void *VehVizUpdater_callback(void *arg) {
	// Get the pointer to the visualizer options/parameters
	vizOptions_t *vizopts_ptr = static_cast<vizOptions_t *>(arg);
	// Get a direct pointer to the database
	ldmmap::LDMMap *db_ptr = vizopts_ptr->db_ptr;

	// Get the central lat and lon values stored in the DB
	std::pair<double,double> centralLatLon= db_ptr->getCentralLatLon();

	// Create a new veheicle visualizer object reading the (IPv4) address and port from the options (the default values are set as a macro in options/options.h)
	vehicleVisualizer vehicleVisObj(vizopts_ptr->opts_ptr->vehviz_nodejs_port,std::string(options_string_pop(vizopts_ptr->opts_ptr->vehviz_nodejs_addr)));

	// Start the node.js server and perform an initial connection with it
	vehicleVisObj.setHTTPPort(vizopts_ptr->opts_ptr->vehviz_web_interface_port);
	vehicleVisObj.startServer();
	vehicleVisObj.connectToServer ();
	vehicleVisObj.sendMapDraw(centralLatLon.first, centralLatLon.second,
		vizopts_ptr->opts_ptr->min_lat,vizopts_ptr->opts_ptr->min_lon,
		vizopts_ptr->opts_ptr->max_lat,vizopts_ptr->opts_ptr->max_lon,
		vizopts_ptr->opts_ptr->ext_lat_factor,vizopts_ptr->opts_ptr->ext_lon_factor);

	globVehVizPtr=&vehicleVisObj;

	synccv.notify_all();

	// Create a new timer
	Timer tmr(vizopts_ptr->opts_ptr->vehviz_update_interval_sec*1e3);
	std::cout << "[INFO] Vehicle visualizer updater started. Updated every " << vizopts_ptr->opts_ptr->vehviz_update_interval_sec << " seconds." << std::endl;

        if(tmr.start()==false) {
                std::cerr << "[ERROR] Fatal error! Cannot create timer for the Vehicle Visualizer update thread!" << std::endl;
                terminatorFlag = true;
                pthread_exit(nullptr);
        }

        POLL_DEFINE_JUNK_VARIABLE();

        while(terminatorFlag == false && tmr.waitForExpiration()==true) {

                        // ---- These operations will be performed periodically ----

                        db_ptr->executeOnAllVehicleContents(&updateVisualizer, static_cast<void *>(&vehicleVisObj));
						db_ptr->executeOnAllEventContents(&updateEventVisualizer, static_cast<void *>(&vehicleVisObj));

			// --------
	}

	if(terminatorFlag == true) {
		std::cerr << "[WARN] Vehicle visualizer updater terminated due to error." << std::endl;
	}

	pthread_exit(nullptr);
}

typedef struct addVdToFrameArgs {
	FrameBuffer *fbPtr;
	uint64_t reftime_ms;
	uint64_t window_size_ms;
} addVdToFrameArgs_t;

void addVdToFrame(ldmmap::vehicleData_t vehdata, void *args) {
	addVdToFrameArgs_t *targs = static_cast<addVdToFrameArgs_t *>(args);
	if(targs->reftime_ms-vehdata.gnTimestamp <= targs->window_size_ms) {
		targs->fbPtr->add(&vehdata);
	}
	else{
		std::cout << "Vehicle with stationID " << vehdata.stationID << " is outside the time window for the current frame. Not adding it to the frame buffer." << std::endl;
	}
}

void randomFillFrameBuffer(FrameBuffer* fbPtr, int num_vehicles){
	FrameBuffer::vehicleSnapshot_t vs;
	vs.stationID = 1001;
	vs.width = 1.8;
	vs.length = 4.5;
	vs.stationType = ldmmap::e_StationTypeLDM::StationType_LDM_passengerCar;
	vs.x = 1.1;
	vs.y = 2.2;
	vs.speed = 10.0;
	vs.heading = 45.0;
	for(int i=0;i<num_vehicles;i++){
		fbPtr->addCustom(&vs, get_timestamp_us());
		vs.stationID++;
		vs.x += 0.1;
		vs.y += 0.1;
		vs.heading += 0.5;
	}
}

extern char **environ;
pid_t uv_spawn_gnn(char *fifo_path, uint16_t pack_size, char *gnn_snapshot_path, char* gnn_csv_out_path) {
	posix_spawn_file_actions_t fa;
	posix_spawn_file_actions_init(&fa);

	// ========= change working directory to gnn
	int rc = posix_spawn_file_actions_addchdir_np(&fa, "gnn");
	if (rc != 0) {
        posix_spawn_file_actions_destroy(&fa);
        return -1;
    }

	// ========= prepare arguments vector
	std::vector<char*> argv;

	// -- push back macros while transforming to char*
	#define PB(strname) argv.push_back(strname);
	#define PBC(strname,strval) char strname[] = strval; argv.push_back(strname);
	#define PBCI(val,sz) char cstr_##val[sz]; sprintf(cstr_##val,"%d",val); PB(cstr_##val);

	// -- push back the arguments
    PBC(uv,"uv");
	PBC(run,"run");
	PBC(rcv_py_path,"rcv.py");
	PBC(opt_f,"-f");
	PB(fifo_path);
	PBC(opt_p,"-p");
	PBCI(pack_size,5);
	PBC(opt_w,"-s");
	PB(gnn_snapshot_path);
	PBC(opt_o, "-O");
	PB(gnn_csv_out_path);
    PB(nullptr);


	// ========= spawn the process
	pid_t pid;
    rc = posix_spawnp(
        &pid,
        "uv",
        &fa,   // file actions
        nullptr,   // spawn attrs
        argv.data(),
        environ
    );

	// ========= cleanup & error check
	posix_spawn_file_actions_destroy(&fa);
    if (rc != 0) {
        // errno-style error check
        return -1;
    }

    return pid;
	#undef PB
	#undef PBC
	#undef PBCI
}

void *nnModelUpdater_callback(void* arg) {
	// This function should periodically read from the database and update the neural network model

	nnModelUpdaterOptions_t* opts = static_cast<nnModelUpdaterOptions_t*>(arg);
	ldmmap::LDMMap* db_ptr = opts->db_ptr;

	// create fifo pipe with mkfifo
	int fifofd=-1;
	std::string fifo_path = "/tmp/nn_mup_fifo" + std::to_string(getpid());
	if (mkfifo(fifo_path.c_str(), 0660) < 0){
		std::cerr << "[ERROR] Cannot create FIFO pipe for Neural Network Model Updater!" << std::endl;
		terminatorFlag = true;
		pthread_exit(nullptr);
	}
	else{
		std::cout << "[INFO] FIFO pipe created at " << GREEN_ESCAPE << fifo_path << RESET_ESCAPE << " for Neural Network Model Updater." << std::endl;
		// open fifo for writing
		fifofd = open(fifo_path.c_str(), O_RDWR | O_NONBLOCK);
		if (fifofd < 0) {
			std::cerr << "[ERROR] Cannot open FIFO pipe for Neural Network Model Updater!" << std::endl;
			// created but can't open, so unlink it
			unlink(fifo_path.c_str());
			terminatorFlag = true;
			pthread_exit(nullptr);
		}
	}

	// spawn the gnn model and attach it to the pipe
	std::string gnn_relative_snapshot_path = std::string("../") + opts->gnn_snapshot_path;
	std::string gnn_relative_csv_out_path = std::string("../") + opts->gnn_csv_out_path;
	pid_t pygnn_pid = uv_spawn_gnn(const_cast<char*>(fifo_path.c_str()), opts->pack_size, const_cast<char*>(gnn_relative_snapshot_path.c_str()), const_cast<char*>(gnn_relative_csv_out_path.c_str()));
	if (pygnn_pid < 0) {
		std::cerr << "[ERROR] Cannot spawn the GNN model process for Neural Network Model Updater!" << std::endl;
		close(fifofd);
		unlink(fifo_path.c_str());
		terminatorFlag = true;
		pthread_exit(nullptr);
	}
	else {
		std::cout << "[INFO] Spawned GNN model process with PID " << GREEN_ESCAPE << pygnn_pid << RESET_ESCAPE << " for Neural Network Model Updater." << std::endl;
	}

	// create frame object
	auto central_lat_lon = db_ptr->getCentralLatLon();
	FrameBuffer frameBuf(fifofd, opts->max_frame_size, central_lat_lon.first, central_lat_lon.second, opts->sumo_netoffset_x, opts->sumo_netoffset_y, 1);

	// Create a new timer
	Timer tmr(opts->period_ms);
	std::cout << "[INFO] Neural Network Model Updater started. Updating every " << opts->period_ms << " milliseconds." << std::endl;
	if(tmr.start()==false) {
		std::cerr << "[ERROR] Fatal error! Cannot create timer for the Neural Network Model Updater thread!" << std::endl;
		unlink(fifo_path.c_str());
		terminatorFlag = true;
		pthread_exit(nullptr);
	}

	POLL_DEFINE_JUNK_VARIABLE();

	while (terminatorFlag == false && tmr.waitForExpiration()==true) {
		// Implement the logic to read from the database and update the neural network model
		// ---- These operations will be performed periodically ----
		// Placeholder: print a message indicating the update operation

		// get time for window reference
		addVdToFrameArgs_t addVdArgs = {
			.fbPtr = &frameBuf,
			.reftime_ms = get_timestamp_ms_gn(),
			.window_size_ms = opts->period_ms
		};
		db_ptr->executeOnAllVehicleContents(&addVdToFrame, static_cast<void *>(&addVdArgs));
		frameBuf.flushToFd(FrameBuffer::serialization_t::json);
		// --------
	}

	if (!frameBuf.empty()) {
		frameBuf.flushToFd(FrameBuffer::serialization_t::json);
	}

	if (terminatorFlag == true) {
		std::cerr << "[WARN] Neural Network Model Updater terminated due to error." << std::endl;
	}
	close(fifofd);
	unlink(fifo_path.c_str());
	pthread_exit(nullptr);
}

int main(int argc, char **argv) {
	curl_global_init(CURL_GLOBAL_DEFAULT);
	terminatorFlag = false;

	// DB cleaner thread ID
	pthread_t dbcleaner_tid;
	// Vehicle visualizer update thread ID
	pthread_t vehviz_tid;
	// Thread attributes (unused, for the time being)
	// pthread_attr_t tattr;

	pthread_t nn_updater_tid;
	// thread periodically reading from db and updating nn model

	// First of all, parse the options
	options_t sldm_opts;

	// Read options from command line
	options_initialize(&sldm_opts);
	if(parse_options(argc, argv, &sldm_opts)) {
		fprintf(stderr,"Error while parsing the options with the C options module.\n");
		exit(EXIT_FAILURE);
	}

	std::time_t now = std::time(nullptr);
	fprintf(stdout,"[INFO] The S-LDM started at %.24s, corresponding to GNTimestamp = %lu\n",std::ctime(&now),get_timestamp_ms_gn());
	fprintf(stdout,"[INFO] S-LDM version: %s\n",VERSION_STR);

	// Print, as an example, the full (internal + external) area covered by the S-LDM
	std::cout << "This S-LDM instance will cover the full area defined by: [" << 
		sldm_opts.min_lat-sldm_opts.ext_lat_factor << "," << sldm_opts.min_lon-sldm_opts.ext_lon_factor << "],[" <<
		sldm_opts.max_lat+sldm_opts.ext_lat_factor  << "," << sldm_opts.max_lon+sldm_opts.ext_lon_factor << "]" <<
		std::endl;
	if(sldm_opts.cross_border_trigger==true) {
		std::cout << "Cross-border trigger mode enabled." << std::endl;
	}

	/* ----------------- TEST AREA (insert here your test code, which will be removed from the final version of main()) ----------------- */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */

	std::cout << "*-*-*-*-*-* [INFO] S-LDM startup auto-tests started... *-*-*-*-*-*" << std::endl;

	// Create a new veheicle visualizer object
	//vehicleVisualizer vehicleVisObj;

	//vehicleVisObj.startServer();
	//vehicleVisObj.connectToServer ();

	// Draw the sample vehicle on the map (simulating 5 updates)
	//vehicleVisObj.sendMapDraw(45.562149, 8.055311);
	//vehicleVisObj.sendObjectUpdate("veh1",45.562149, 8.055311);
	// sleep(1);
	// vehicleVisObj.sendObjectUpdate("veh1",45.562139, 8.055311);
	// sleep(1);
	// vehicleVisObj.sendObjectUpdate("veh1",45.562129, 8.055311);
	// sleep(1);
	// vehicleVisObj.sendObjectUpdate("veh1",45.562119, 8.055311);
	// sleep(1);
	// vehicleVisObj.sendObjectUpdate("veh1",45.562109, 8.055311);

	/* CAM sample (GeoNet,BTP,CAM) (85 bytes) */
	static unsigned char cam[85] = {
		0x11, 0x00, 0x50, 0x01, 0x20, 0x50, 0x02, 0x80, /* ..P. P.. */
		0x00, 0x2d, 0x01, 0x00, 0x14, 0x00, 0x00, 0x00, /* ........ */
		0x00, 0x00, 0x00, 0x03, 0xcf, 0x37, 0x73, 0x6b, /* .....7sk */
		0x1a, 0xda, 0xad, 0xe3, 0x04, 0x90, 0x1e, 0x5e, /* .......^ */
		0x00, 0x01, 0x00, 0xb4, 0x00, 0x00, 0x00, 0x00, /* ........ */
		0x07, 0xd1, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, /* ........ */
		0x00, 0x01, 0xc1, 0xa0, 0x00, 0x5a, 0x0f, 0xef, /* ...C.Z.. */
		0x56, 0x8d, 0xfb, 0x4a, 0x3a, 0x3f, 0xff, 0xff, /* ...>.... */
		0xfc, 0x23, 0xb7, 0x74, 0x3e, 0x20, 0xa8, 0xcf, /* .#.t> p. */
		0xc0, 0x8b, 0x7e, 0x83, 0x18, 0x8a, 0xf3, 0x37, /* .C~....7 */
		0xfe, 0xeb, 0xff, 0xf6, 0x08                   /* .. */
	};

	/* DENM sample (140 bytes) */
	static unsigned char denm[104] = {
		0x11, 0x00, 0xf1, 0x01, 0x20, 0x40, 0x01, 0x00, /* .... @.. */
		0x00, 0x30, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00, /* .0...... */
		0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* <....... */
		0x97, 0xe0, 0xf1, 0x5c, 0x1a, 0xda, 0x91, 0x36, /* ...\...6 */
		0x04, 0x90, 0x39, 0x55, 0x00, 0x00, 0x00, 0x00, /* ..9U.... */
		0x1a, 0xda, 0x91, 0x35, 0x04, 0x90, 0x39, 0x55, /* ...5..9U */
		0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* .d...... */
		0x07, 0xd2, 0x00, 0x00, 0x02, 0x01, 0x2e, 0x5d, /* .......] */
		0xa4, 0xe7, 0x20, 0x17, 0x2e, 0xd2, 0x73, 0x80, /* .. ...s. */
		0x00, 0x0f, 0xf2, 0xfc, 0x3f, 0xe8, 0xc0, 0x00, /* ....?... */
		0x00, 0x00, 0x55, 0xd6, 0xb4, 0x9d, 0x20, 0x1d, /* ..U... . */
		0x69, 0x3a, 0x40, 0x10, 0x00, 0x00, 0x00, 0x00, /* i:@..... */
		0xdb, 0xba, 0x1f, 0x0f, 0x08, 0x20, 0x18, 0x00, /* ..... .. */
	};

	unsigned char *ptr = &cam[0];

	// Sample DENM from byte array
	uint8_t denm2_bytes[190];
	std::string denm2_content="11000501204001800086020040EE000014008CFDF01F6D075DE0B1E41B5EA6B506950D1386B801FB1B5EA82D06950FAB01F400000000000007D200000201F01F6D07C7780FB68380020F6BBC1679E3DAEF059E7D103912D71DEE1AB0C80C80001E0788600050141090230D483C7F84826FE283F302C67000C77ECD1F77563380080BF658FBBE31920040DFB297DDF18CE0020AFD96BEEF6C64801037EC89F77663380080BF642FBBB319C00415FB2E7DE318CE00202FD983EF1AC6700102";
	
	for (unsigned int i = 0; i < denm2_content.length(); i += 2) {
		std::string byteString = denm2_content.substr(i, 2);
		uint8_t byte = (uint8_t) strtol(byteString.c_str(), nullptr, 16);
		denm2_bytes[i/2] = byte;
	}

	etsiDecoder::decoderFrontend decodeFrontend;
	etsiDecoder::etsiDecodedData_t decodedData;

	Security::Security_error_t sec_retval;
	storedCertificate_t certificateData;
	if(decodeFrontend.decodeEtsi((uint8_t *)&denm2_bytes[0], 190, decodedData, sec_retval,certificateData)!=ETSI_DECODER_OK) {
		std::cerr << "Error! Cannot decode ETSI packet!" << std::endl;
	}

	if(decodedData.type == etsiDecoder::ETSI_DECODED_CAM) {
		CAM_t *decoded_cam;

		decoded_cam = (CAM_t *) decodedData.decoded_msg;

		printf("GNTimestamp: %u\n",decodedData.gnTimestamp);

		printf("Lat: %.7lf, Lon: %.7lf\n",
			(double)decoded_cam->cam.camParameters.basicContainer.referencePosition.latitude/10000000.0,
			(double) decoded_cam->cam.camParameters.basicContainer.referencePosition.longitude/10000000.0);
	} else if(decodedData.type == etsiDecoder::ETSI_DECODED_DENM) {
		DENM_t *decoded_denm;

		decoded_denm = (DENM_t *) decodedData.decoded_msg;

		printf("GNTimestamp: %u\n",decodedData.gnTimestamp);

		printf("GeoArea: \nLat: %.7lf, Lon: %.7lf, DistA %u, DistB %u, Angle %u\n",
				(double)decodedData.posLat/10000000.0,
				(double) decodedData.posLong/10000000.0,
				decodedData.distA,
				decodedData.distB,
				decodedData.angle);
	}

	// Test with a db
	ldmmap::LDMMap dbtest;
	ldmmap::vehicleData_t veh1 = {.stationID=188321312, .lat=45.562149, .lon=8.055311, .elevation=440, .heading=120, .speed_ms=17, .gnTimestamp=34235235235, .timestamp_us=0};
	veh1.timestamp_us = get_timestamp_us(); // now
	dbtest.insertVehicle(veh1);
	std::printf("Test vehicle 1 inserted @ %lu\n",veh1.timestamp_us);

	ldmmap::vehicleData_t veh2 = {.stationID=288321312, .lat=45.512149, .lon=8.355311, .elevation=440, .heading=100, .speed_ms=17, .gnTimestamp=34235235235, .timestamp_us=0};
	veh2.timestamp_us = get_timestamp_us()-2*1e6; // 2 seconds ago
	dbtest.insertVehicle(veh2);
	std::printf("Test vehicle 2 inserted @ %lu\n",veh2.timestamp_us);

	ldmmap::vehicleData_t veh3 = {.stationID=388321312, .lat=45.592149, .lon=8.855311, .elevation=440, .heading=80, .speed_ms=17, .gnTimestamp=34235235235, .timestamp_us=0};
	veh3.timestamp_us = get_timestamp_us()-5*1e6; // 5 seconds ago
	dbtest.insertVehicle(veh3);
	std::printf("Test vehicle 3 inserted @ %lu\n",veh3.timestamp_us);

	ldmmap::vehicleData_t veh4 = {.stationID=488321312, .lat=45.362149, .lon=8.755311, .elevation=440, .heading=10, .speed_ms=17, .gnTimestamp=34235235235, .timestamp_us=0};
	veh4.timestamp_us = get_timestamp_us()-7*1e6; // 7 seconds ago
	dbtest.insertVehicle(veh4);
	std::printf("Test vehicle 4 inserted @ %lu\n",veh4.timestamp_us);

	// Print all the contents of the test DB (should be equal to 4)
	dbtest.printAllVehicleContents("Before deletion");

	// Print the size of the test DB
	std::cout << "Number of elements stored in the LDMMap DB: " << dbtest.getVehicleCardinality() << std::endl;

	// Delete now all the vehicles older than 5.5 seconds
	dbtest.deleteVehicleOlderThan(5500); // Only 188321312, 288321312 and 388321312 should remain in the DB

	// Now print all the contents of the DB again
	dbtest.printAllVehicleContents("After deletion");

	// Print the size of the test DB again (should be equal to 3)
	std::cout << "Number of elements stored in the LDMMap DB: " << dbtest.getVehicleCardinality() << std::endl;

	dbtest.setCentralLatLon(45.562149,8.055311); // Set a central lat lon for testing the visualizer thread

	dbtest.clear();

	std::cout << "*-*-*-*-*-* [INFO] S-LDM startup auto-tests terminated. The S-LDM will start now. *-*-*-*-*-*" << std::endl;

	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */
	/* ------------------------------------------------------------------------------------------------------------------------------------ */

	// Get the log file name from the options, if available, to enable log mode inside the AMQP client and the S-LDM modules
	std::string logfile_name="";
	if(options_string_len(sldm_opts.logfile_name)>0) {
		logfile_name=std::string(options_string_pop(sldm_opts.logfile_name));
		if(logfile_name!="stdout") {
			time_t rawtime;
			struct tm * timeinfo;
  			char buffer [25] = {NULL};
  			time (&rawtime);
  			timeinfo = localtime (&rawtime);
  			strftime (buffer,25,"-%Y%m%d-%H:%M:%S",timeinfo);
			logfile_name += buffer;
		}

	}

	// Create a new DB object
	ldmmap::LDMMap *db_ptr = new ldmmap::LDMMap();

	// Create a CertificateStore object (the same object will be then accessed by all the AMQP clients, when using more than one client)
	CertificateStore *certStore_ptr = new CertificateStore();

	// Create a MisbehaviourDetector object (the same object will be then accessed by all the AMQP clients, when using more than one client)
	// Options passed just for future uses, may get removed
	MisbehaviourDetector *mbd_ptr=new MisbehaviourDetector(sldm_opts.min_lat,sldm_opts.min_lon,sldm_opts.max_lat,sldm_opts.max_lon,certStore_ptr,db_ptr,logfile_name);

	// Set a central latitude and longitude depending on the coverage area of the S-LDM (to be used only for visualization purposes -
	// - it does not affect in any way the performance or the operations of the LDMMap DB module)
	db_ptr->setCentralLatLon((sldm_opts.min_lat+sldm_opts.max_lat)/2.0, (sldm_opts.min_lon+sldm_opts.max_lon)/2.0);

	// Before starting the AMQP client event loop, we should create a parallel thread, reading periodically 
	// (e.g. every 5 s) the database through the pointer "db_ptr" and "cleaning" the entries which are too old
	// pthread_attr_init(&tattr);
	// pthread_attr_setdetachstate(&tattr,PTHREAD_CREATE_DETACHED);
	cleanerArgs args;
	args.db_ptr=db_ptr;
	args.certStore_ptr=certStore_ptr;
	args.mbd_ptr=mbd_ptr;
	pthread_create(&dbcleaner_tid,NULL,DBcleaner_callback,(void *) &args);
	// pthread_attr_destroy(&tattr);

	// We should also start here a second parallel thread, reading periodically the database (e.g. every 500 ms) and sending the vehicle data to
	// the vehicleVisualizer
	// pthread_attr_init(&tattr);
	// pthread_attr_setdetachstate(&tattr,PTHREAD_CREATE_DETACHED);
	vizOptions_t vizParams = {db_ptr,&sldm_opts};
	pthread_create(&vehviz_tid,NULL,VehVizUpdater_callback,(void *) &vizParams);
	// pthread_attr_destroy(&tattr);

	// third thread to read periodically from db and update python nn model
	if (sldm_opts.gnn_trigger_enabled==true){
		char *gnn_snapshot_path=nullptr;
		if(options_string_len(sldm_opts.gnn_snapshot_path)>0)
			gnn_snapshot_path=options_string_pop(sldm_opts.gnn_snapshot_path);

		char *gnn_csv_out_path=nullptr;
		if(options_string_len(sldm_opts.gnn_csv_out_path)>0)
			gnn_csv_out_path=options_string_pop(sldm_opts.gnn_csv_out_path);
		
		nnModelUpdaterOptions_t nnMUP = {db_ptr, sldm_opts.gnn_step_len_ms, MAX_VEHICLES_PER_FRAME, sldm_opts.gnn_pack_size, gnn_snapshot_path, sldm_opts.gnn_sumo_netoffset_x, sldm_opts.gnn_sumo_netoffset_y, gnn_csv_out_path}; // every 100 ms, max frame size 2000 vehicles, 100 frames
		pthread_create(&nn_updater_tid,NULL,nnModelUpdater_callback,(void *) &nnMUP);
	}

	// Create an indicatorTriggerManager object (the same object will be then accessed by all the AMQP clients, when using more than one client)
	indicatorTriggerManager itm(db_ptr,&sldm_opts);

	if(sldm_opts.left_indicator_trg_enable==true) {
		itm.setLeftTurnIndicatorEnable(true);
	}

	// Set up the AMQP QuadKey filter for the AMQP client(s) (if more clients are spawned, the filter should be the same for all of them)
	// -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
	QuadKeys::QuadKeyTS tilesys;
	std::string filter_str;
	bool cache_file_found=false;
	tilesys.setLevelOfDetail(16);

	FILE *logfile_file = nullptr;
	uint64_t bf = 0.0,af = 0.0;

	// This is just to log the time needed to compute the full QuadKey filter, if requested by the user
	if(logfile_name!="") {
		if(logfile_name=="stdout") {
			logfile_file=stdout;
		} else {
			// Opening the output file in write + append mode just to be safe in case the user does not change the file name
			// between different executions of the S-LDM
			logfile_file=fopen(logfile_name.c_str(),"wa");
		}

		bf=get_timestamp_ns();
	}

	filter_str=tilesys.getQuadKeyFilter(sldm_opts.min_lat-sldm_opts.ext_lat_factor,sldm_opts.min_lon-sldm_opts.ext_lon_factor,sldm_opts.max_lat+sldm_opts.ext_lat_factor,sldm_opts.max_lon+sldm_opts.ext_lon_factor,&cache_file_found);

	// This is just to log the time needed to compute the full QuadKey filter, if requested by the user
	if(logfile_name!="") {
		af=get_timestamp_ns();

		fprintf(logfile_file,"[LOG - QUADKEY FILTER COMPUTATION] Area (internal)=%.7lf:%.7lf-%.7lf:%7lf Area (full)=%.7lf:%.7lf-%.7lf:%7lf QKCacheFileFound=%d ProcTimeMilliseconds=%.6lf\n",
			sldm_opts.min_lat,sldm_opts.min_lon,sldm_opts.max_lat,sldm_opts.max_lon,
			sldm_opts.min_lat-sldm_opts.ext_lat_factor,sldm_opts.min_lon-sldm_opts.ext_lon_factor,sldm_opts.max_lat+sldm_opts.ext_lat_factor,sldm_opts.max_lon+sldm_opts.ext_lon_factor,
			cache_file_found,(af-bf)/1000000.0);

		if(logfile_name!="stdout" && logfile_file!=nullptr) {
			fclose(logfile_file);
		}
	}

	// -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

	// Create the main AMQP client object
	AMQPClient mainRecvClient(std::string(options_string_pop(sldm_opts.amqp_broker_one.broker_url)), std::string(options_string_pop(sldm_opts.amqp_broker_one.broker_topic)), sldm_opts.min_lat, sldm_opts.max_lat, sldm_opts.min_lon, sldm_opts.max_lon, &sldm_opts, db_ptr, logfile_name);

	// Create the JSONserver object for the on-demand JSON-over-TCP interface
	JSONserver jsonsrv(db_ptr);

	// Start the on-demand JSON-over-TCP interface if enabled through the corresponding option
	if(sldm_opts.od_json_interface_enabled==true) {
		jsonsrv.setServerPort(sldm_opts.od_json_interface_port);
		if(jsonsrv.startServer()!=true) {
			fprintf(stderr,"Critical error: cannot start the JSON server for data retrieval from other services.\n");
			exit(EXIT_FAILURE);
		}
	}

	// Start the AMQP client event loop (additional clients, if requested by the user)
	std::vector<std::thread> amqp_x_threads;

	if(sldm_opts.num_amqp_x_enabled>0) {
		std::cout << "[INFO] Additional AMQP clients will be used by the current instance of the S-LDM. Total number of AMQP clients (including the main one): " << sldm_opts.num_amqp_x_enabled+1 << std::endl;

		for(unsigned int i=0;i<sldm_opts.num_amqp_x_enabled;i++) {
			amqp_x_threads.emplace_back(AMQPclient_t,db_ptr,&sldm_opts,(logfile_name == "stdout" ? "stdout" : logfile_name + std::to_string(i+2)),
				std::to_string(i+2),i,&itm,filter_str,&mainRecvClient,mbd_ptr,certStore_ptr);
		}
	}

	if(sldm_opts.amqp_broker_one.amqp_reconnect_after_local_timeout_expired==true) {
		std::cout << "[AMQPClient 1] This client will be restarted if a local idle timeout error occurs." << std::endl;
	}

	// If this flag is set to true, the client will be restarted after an error, instead of being terminated
	bool cli_restart = false;

	do {
		cli_restart = false;

		// Start the AMQP client event loop (main client)
		try {
			// The indicator trigger manager is disabled by default in AMQPClient, unless it is explicitely enabled with a call to setIndicatorTriggerManager(true)
			if(sldm_opts.indicatorTrgMan_enabled==true) {
				mainRecvClient.setIndicatorTriggerManager(&itm);
			}

			// Activate Misbehaviour Detector is enabled
			mainRecvClient.setMisbehaviourDetector(mbd_ptr);

			// Set username, if specified
			if(options_string_len(sldm_opts.amqp_broker_one.amqp_username)>0) {
				mainRecvClient.setUsername(std::string(options_string_pop(sldm_opts.amqp_broker_one.amqp_username)));
			}

			// Set password, if specified
			if(options_string_len(sldm_opts.amqp_broker_one.amqp_password)>0) {
				mainRecvClient.setPassword(std::string(options_string_pop(sldm_opts.amqp_broker_one.amqp_password)));
			}

			// Set connection options (they all default to "false" - see also options.c/broker_options_inizialize())
			mainRecvClient.setConnectionOptions(sldm_opts.amqp_broker_one.amqp_allow_sasl,sldm_opts.amqp_broker_one.amqp_allow_insecure,sldm_opts.amqp_broker_one.amqp_reconnect);
			mainRecvClient.setIdleTimeout(sldm_opts.amqp_broker_one.amqp_idle_timeout);

			mainRecvClient.setClientID("1");

			// Set the QuadKey filter
			if(sldm_opts.quadkFilter_enabled==true) {
				mainRecvClient.setFilter(filter_str);
			}

			proton::container(mainRecvClient).run();
		} catch (const std::exception& e) {
			if(sldm_opts.amqp_broker_one.amqp_reconnect_after_local_timeout_expired==true && std::string(e.what()) == "amqp:resource-limit-exceeded: local-idle-timeout expired") {
				std::cerr << "[AMQPClient 1] Exception occurred: " << e.what() << std::endl;
				std::cout << "[AMQPClient 1] Attempting to restart the client after a local idle timeout expired error..." << std::endl;
				mainRecvClient.force_container_stop();
				sleep(1);
				cli_restart = true;
			} else {
				std::cerr << e.what() << std::endl;
				terminatorFlag = true;
			}
		}
	} while(cli_restart==true);

	pthread_join(dbcleaner_tid,nullptr);
	pthread_join(vehviz_tid,nullptr);
	if (sldm_opts.gnn_trigger_enabled==true) {
		pthread_join(nn_updater_tid,nullptr);
	}

	if(sldm_opts.num_amqp_x_enabled>0) {
		fprintf(stdout,"[INFO] Terminating the other AMQP clients...\n");
		// Close the connection on all the other brokers (if multiple clients are used)
		amqpclimutex.lock();
		for (auto const& [key, val] : amqpclimap) {
			fprintf(stdout,"[INFO] Terminating client %d...\n",key+2);
			val->force_container_stop();
		}
		amqpclimutex.unlock();
	}

	// Joining threads from additional AMQP clients
	for(std::vector<std::thread>::size_type i=0;i<amqp_x_threads.size();i++) {
		amqp_x_threads[i].join();
	}

	db_ptr->clear();

	// Freeing the options
	options_free(&sldm_opts);

	return 0;
}
