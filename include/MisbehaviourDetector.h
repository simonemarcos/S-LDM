#ifndef MB_DETECTOR_H
#define MB_DETECTOR_H

#include <mutex>
#include <list>
#include <map>

#include "LDMmap.h"
#include "etsiDecoderFrontend.h"

#include <proton/message.hpp>

// shift for bitmap operations
#define MB_CODE_CONV(MB_CODE) 1<<MB_CODE

// returns true if misbehavour is detected false otherwise
#define MB_CODE_CHECK(MB_CODE,FIELD_N) return ((1<<FIELD_N) & MB_CODE)!=0

extern "C" {
	#include "CAM.h"
	#include "options.h"
}

typedef struct MBDOptions {
	bool useHaversineDistance;
	double tolerance;
	double maxTimeForConsecutive;
	bool ignoreSecurity;
} MBDOptions_t;

// Empty MBD, received messages from AMQPclient are just processed and inserted in LDM 
class MisbehaviourDetector {
	public:
		MisbehaviourDetector(std::string logfile_name) : m_logfile_name(logfile_name) {
			m_logfile_file=nullptr;
			if (m_logfile_name!="") {
				if (m_logfile_name=="stdout") {
					m_logfile_file=stdout;
				} else {
					m_logfile_file=fopen(m_logfile_name.c_str(),"wa");
				}
			}
			Init();
		}

		MisbehaviourDetector() {
			m_logfile_name="";
			m_logfile_file=nullptr;
			Init();
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

			//class 2
			MB_BREACON_FREQ_INC,
			// The following codes correspond to: [first_name] change inconsistent with [second_name]
			MB_POSITION_SPEED_INC,
			MB_POSITION_HEADING_INC,
			MB_HEADING_SPEED_INC,
			MB_HEADING_YAW_RATE_INC,
			MB_SPEED_ACCELERATION_INC,
			MB_POS_AND_HEADING_DIRECTION_INC,
			MB_LENGTH_WIDTH_INC,
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
		} mbdMisbehaviourCode_e;
		
		uint64_t processMessage(etsiDecoder::etsiDecodedData_t decoded_data, ldmmap::vehicleData_t vehdata,
			std::vector<ldmmap::vehicleData_t> PO_vec);
		
		// "protected" just in case new classes will be derived from this one
	protected:
		std::mutex m_already_reported_mutex;
		std::list<uint64_t> m_already_reported; // List of vehicles for which a report has been sent already (potentially use to not trust them)
		
	private:
		std::string m_logfile_name;
		FILE *m_logfile_file;

		MBDOptions_t m_opts;

		std::map<uint64_t, ldmmap::vehicleData_t> m_lastMessageCache;

		std::map<int,ldmmap::vehicleData_t> averages, deviations;
		std::map<int,double> maxSpeeds, maxAccelerations, maxCurvatures, maxYawRates;
		std::map<int,double> maxBrakings; //absolute values of acceleration during braking
		std::map<int,double> maxJerks;
		
		uint64_t individualCAMchecks(ldmmap::vehicleData_t vehdata, uint64_t &unavailables);
		void Init();
};

#endif // MB_DETECTOR_H