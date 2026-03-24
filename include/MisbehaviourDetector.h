#ifndef MB_DETECTOR_H
#define MB_DETECTOR_H

#include <mutex>
#include <map>

#include "LDMmap.h"
#include "etsiDecoderFrontend.h"
#include "OSMStore.h"

#include <proton/message.hpp>

// shift for bitmap operations
#define MB_CODE_CONV(MB_CODE) 1<<MB_CODE
#define MB_CODE_CLEAR(MB_CODE) !MB_CODE_CONV(MB_CODE)

// returns true if misbehavour is detected false otherwise
#define MB_CODE_CHECK(MB_CODE,FIELD_N) ((1<<FIELD_N) & MB_CODE)!=0

extern "C" {
	#include "CAM.h"
	#include "VAM.h"
	#include "DENM.h"
	#include "CPM.h"
	#include "options.h"
}

typedef struct MBDOptions {
	bool useHaversineDistance;
	double tolerance;
	double maxTimeForConsecutive;
	bool ignoreSecurity;
	double cpmToleranceMultiplier;
	std::string weatherAPIKey;
} MBDOptions_t;

typedef struct pendingEvent {
	uint64_t keyEvent;
	uint64_t endOfChecks; // timestamp to reach while holding the event as pending to perform followup checks
	ldmmap::eventData_t evedata;
	uint64_t EMB_CODE;
	uint64_t unavailables;
	std::set<uint64_t> reporters;
	osmium::object_id_type eventRoad;
	double eventHeading;
} pendingEvent_t;

// Empty MBD, received messages from AMQPclient are just processed and inserted in LDM 
class MisbehaviourDetector {
	public:
		MisbehaviourDetector(double minlat, double minlon, double maxlat, double maxlon, CertificateStore *certStore_ptr, ldmmap::LDMMap *db_ptr, std::string logfile_name) : m_logfile_name(logfile_name) {
			m_certStore_ptr=certStore_ptr;
			m_db_ptr=db_ptr;
			m_logfile_file=nullptr;
			if (m_logfile_name!="") {
				if (m_logfile_name=="stdout") {
					m_logfile_file=stdout;
				} else {
					m_logfile_file=fopen(m_logfile_name.c_str(),"wa");
				}
			}
			Init(minlat, minlon, maxlat, maxlon);
		}

		MisbehaviourDetector(double minlat, double minlon, double maxlat, double maxlon, CertificateStore *certStore_ptr, ldmmap::LDMMap *db_ptr) {
			m_certStore_ptr=certStore_ptr;
			m_db_ptr=db_ptr;
			m_logfile_name="";
			m_logfile_file=nullptr;
			Init(minlat, minlon, maxlat, maxlon);
		}
		
		//remove from reported list, need to find a use for it
        void notifyOpTermination(uint64_t stationID);
		
		typedef enum {
			//class 1
			MB_SPEED_IMP,
			MB_DIRECTION_SPEED_IMP,
			MB_ACCELERATION_IMP,
			MB_CURVATURE_IMP,
			MB_YAW_RATE_IMP,
			
			//class 3
			MB_NOT_ON_ROAD,
			MB_INSIDE_BUILDING,
			MB_HEADING_NOT_FOLLOWING_ROAD,
			MB_SPEED_OVER_ROAD_LIMIT,

			// VAM specific checks
			MB_MOVEMENT_CONTROL,
			MB_ENVIRONMENT,

			//class 2
			MB_BEACON_FREQ_INC,
			// The following codes correspond to: [first_name] change inconsistent with [second_name]
			MB_POSITION_SPEED_INC,
			MB_POSITION_HEADING_INC,
			MB_HEADING_SPEED_INC,
			MB_HEADING_YAW_RATE_INC,
			MB_SPEED_ACCELERATION_INC,
			MB_POS_AND_HEADING_DIRECTION_INC,
			MB_LENGTH_WIDTH_INC, // USED AS SIZECLASS INCONSISTENCY IN VAM CHECKS
			MB_ACCELERATION_INC,
			MB_CURVATURE_SPEED_INC,
			MB_CURVATURE_HEADING_INC,
			MB_CURVATURE_YAW_RATE_INC,
			MB_YAW_RATE_SPEED_INC,
			MB_YAW_RATE_CURVATURE_INC,

			//additional class 2 (class 1.5)
			MB_POSITION_SPEED_IMP,
			MB_HEADING_YAW_RATE_IMP,
			MB_SPEED_ACCELERATION_IMP,

			MB_BEACON_FREQ_LOW,
		} mbdMisbehaviourCode_e;

		typedef enum {
			// Commented out codes for same/similar checks on different events

			// General
			EMB_DISTANCE_DENM_CAM,

			// Traffic Jam
			EMB_ROAD_TYPE,
			EMB_EVENT_HISTORY_INC,
			EMB_UNLIKELY_STATISTICS,
			EMB_UNLIKELY_NEARBY_VEHICLES,
			// Traffic Jam, Dangerous end of queue
			EMB_REPORTER_SPEED,
			EMB_AHEAD_VEH_SPEED,
			EMB_PREDICTED_PATH_INC,
			// Traffic Jam, Traffic jam ahead
			EMB_REPORTER_SLOW_DOWN,
			EMB_SURROUNDING_VEH_SPEED_LDM,
			EMB_SURROUNDING_VEH_SPEED_CAM,

			// Stationary vehicle warning
			EMB_CAM_SPEED,
			// Stationary vehicle warning, Stopped vehicle
			EMB_POSITION_IMP,
			EMB_BUS_TRAM_STOP,
			// Stationary vehicle warning, Breakdown / Post-crash
			EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED,
			EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY,

			// Special vehicle warning, Emergency vehicle in operation
			EMB_CURVATURE_CHANGE_HEADING_INC,
			EMB_CURVATURE_CHANGE_YAW_RATE_INC,
			EMB_STATION_TYPE,
			EMB_STATION_TYPE_REPORTER_CAM,
			EMB_LIGHTBAR_ACTIVE_CAM,
				// EMB_EVENT_HISTORY_INC,
				// EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED,
				// EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY,
			
			// Exchange of IRCs
			EMB_IRC_VEHICLE_MASS,
			EMB_IRC_EVENT_SPEED_INC,
			EMB_IRC_EVENT_HEADING_INC,
			EMB_IRC_BEHAVIOUR_ACCELERATION,

			// Dangerous situation
				// EMB_CAM_SPEED,
				// EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED,
				// EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY,
			
			// Adverse weather condition
			EMB_WEATHER_INFO_INC,
			EMB_WEATHER_UNLIKELY_STATISTICS,
				// EMB_SURROUNDING_VEH_BEHAVIOUR_SPEED,
				// EMB_SURROUNDING_VEH_BEHAVIOUR_HEADING_PATH_HISTORY,
		} mbdEventMisbehaviourCode_e;

		typedef enum {
			// CAM
			UNAV_STATION_TYPE,		// VAM
			UNAV_LONGITUDE,			// VAM CPM
			UNAV_LATITUDE,			// VAM CPM
			//UNAV_ALTITUDE,		// VAM
			UNAV_HEADING,			// VAM
			UNAV_SPEED,				// VAM
			UNAV_DRIVE_DIRECTION,	//
			UNAV_LENGTH,			//
			UNAV_WIDTH,				//
			UNAV_ACCELERATION,		// VAM
			UNAV_CURVATURE,			//
			UNAV_YAW_RATE,			//


		} mbdUnavailableField_e;

		uint64_t processCAM(proton::binary message_bin, ldmmap::vehicleData_t vehdata, Security::Security_error_t sec_retval, storedCertificate_t certificateData);
		// to be used for new DENMs but also to process the reporting of old DENMs that were pending future CAM verifications
		void processDENM(proton::binary message_bin, ldmmap::eventData_t evedata, Security::Security_error_t sec_retval, storedCertificate_t certificateData);
		uint64_t processVAM(proton::binary message_bin, ldmmap::vehicleData_t vehdata, Security::Security_error_t sec_retval, storedCertificate_t certificateData);
		uint64_t processCPM(proton::binary message_bin, std::vector<ldmmap::vehicleData_t> PO_vec, Security::Security_error_t sec_retval, storedCertificate_t certificateData);
		void cleanupPendingEvents();

		// "protected" just in case new classes will be derived from this one
	protected:
		std::mutex m_already_reported_mutex;
		std::set<uint64_t> m_already_reported; // List of vehicles for which a report has been sent already (potentially use to not trust them)
		
	private:
		std::string m_logfile_name;
		FILE *m_logfile_file;

		MBDOptions_t m_opts;

		ldmmap::LDMMap *m_db_ptr;
		CertificateStore *m_certStore_ptr;
		OSMStore *m_osmStore;

		std::map<uint64_t, ldmmap::vehicleData_t> m_lastMessageCache;
		std::map<uint64_t, proton::binary> m_lastBinMessageCache;
		// to keep tracking of events still in active verification
		// when an event has been verified it can be inserted in the db and there's no need to keep it anymore
		std::map<uint64_t, pendingEvent_t> m_pendingEvents;

		std::map<int,ldmmap::vehicleData_t> averages, deviations;
		std::map<int,double> maxSpeeds, maxAccelerations, maxCurvatures, maxYawRates;
		std::map<int,double> maxBrakings; //absolute values of acceleration during braking
		std::map<int,double> maxJerks;
		std::map<int,long> vehicleMasses;

		uint64_t weatherTimestamp;
		int picto, picto_detailed;

		std::vector<int> misbehavioursCAM, misbehavioursVAM, misbehavioursCPM, misbehavioursDENM;
		FILE *log_summary;
		FILE *log_csv;
		int msgNumber;
		
		// to reuse the checks on CPMs set tolMult on values >1 for increased tolerance, default is 1
		uint64_t individualCAMchecks(ldmmap::vehicleData_t vehdata, uint64_t &unavailables, double tolMult=1);
		// to reuse the checks on CPMs set tolMult on values >1 for increased tolerance, default is 1
		uint64_t individualVAMchecks(ldmmap::vehicleData_t vehdata, uint64_t &unavailables, double tolMult=1);
		uint64_t individualCPMchecks(std::vector<ldmmap::vehicleData_t> PO_vec, uint64_t &unavailables);
		uint64_t individualDENMchecks(ldmmap::eventData_t evedata, uint64_t &unavailables, bool &pending);
		void eventDecision(pendingEvent_t currentEvent);
		void Init(double minlat, double minlon, double maxlat, double maxlon);
};

#endif // MB_DETECTOR_H