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

// Empty MBD, received messages from AMQPclient are just processed and inserted in LDM 
class MisbehaviourDetector {
	public:
		MisbehaviourDetector(options_t *opts_ptr, std::string logfile_name) : m_opts_ptr(opts_ptr), m_logfile_name(logfile_name) {
			m_logfile_file=nullptr;
			if (m_logfile_name!="") {
				if (m_logfile_name=="stdout") {
					m_logfile_file=stdout;
				} else {
					m_logfile_file=fopen(m_logfile_name.c_str(),"wa");
				}
			}
			StatsInit();
		}

		MisbehaviourDetector(options_t *opts_ptr) : m_opts_ptr(opts_ptr) {
			m_logfile_name="";
			m_logfile_file=nullptr;
			StatsInit();
		}
		
		//remove from reported list, need to find a use for it
        void notifyOpTermination(uint64_t stationID);
		
		typedef enum {
			MB_SPEED_VALUE_IMP,
			MB_DIRECTION_SPEED_IMP,
			MB_ACCELERATION_VALUE_IMP,
			MB_CURVATURE_IMP,
			MB_YAW_RATE_IMP,

			MB_BREACON_FREQ_INC,
			MB_POSITION_SPEED_INC,
			MB_POSITION_HEADING_INC,
		} mbdMisbehaviourCode_e;
		
		uint64_t processMessage(etsiDecoder::etsiDecodedData_t decoded_data, ldmmap::vehicleData_t vehdata,
			std::vector<ldmmap::vehicleData_t> PO_vec);
		
		// "protected" just in case new classes will be derived from this one
	protected:
		std::mutex m_already_reported_mutex;
		std::list<uint64_t> m_already_reported; // List of vehicles for which a report has been sent already (potentially use to not trust them)
		
	private:
		options_t *m_opts_ptr;
		std::string m_logfile_name;
		FILE *m_logfile_file;

		std::map<uint64_t, ldmmap::vehicleData_t> m_lastMessageCache;

		std::map<int,ldmmap::vehicleData_t> averages, deviations;
		std::map<int,double> maxSpeeds, maxAccelerations, maxCurvatures, maxYawRates;

		uint64_t individualCAMchecks(ldmmap::vehicleData_t vehdata, uint64_t &unavailables);
		void StatsInit();

};

#endif // MB_DETECTOR_H