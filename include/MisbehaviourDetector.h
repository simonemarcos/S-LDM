#ifndef MB_DETECTOR_H
#define MB_DETECTOR_H

#include <mutex>
#include <list>
#include <map>

#include "LDMmap.h"
#include "etsiDecoderFrontend.h"
#include "utils.h"
#include "areaFilter.h"
#include "triggerManager.h"

#include <proton/message.hpp>

extern "C" {
	#include "CAM.h"
	#include "options.h"
}

// Empty MBD, received messages from AMQPclient are just processed and inserted in LDM 
class MisbehaviourDetector {
	public:
		MisbehaviourDetector(ldmmap::LDMMap *db_ptr, options_t *opts_ptr, std::string logfile_name) : m_db_ptr(db_ptr), m_opts_ptr(opts_ptr), m_logfile_name(logfile_name) {
			m_areaFilter.setOptions(m_opts_ptr);
			m_logfile_file=nullptr;
			if (m_logfile_name!="") {
				if (m_logfile_name=="stdout") {
					m_logfile_file=stdout;
				} else {
					m_logfile_file=fopen(m_logfile_name.c_str(),"wa");
				}
			}
		}

		MisbehaviourDetector(ldmmap::LDMMap *db_ptr, options_t *opts_ptr) : m_db_ptr(db_ptr), m_opts_ptr(opts_ptr) {
			m_areaFilter.setOptions(m_opts_ptr);
			m_logfile_name="";
			m_logfile_file=nullptr;
		}
		
		void setDBpointer(ldmmap::LDMMap *db_ptr) {m_db_ptr = db_ptr;}
		bool processMessage(etsiDecoder::etsiDecodedData_t decoded_data, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id);
		void setDetectionEnable(bool m_enable) {m_detection_enabled = m_enable;}

		//remove from reported list, need to find a use for it
        void notifyOpTermination(uint64_t stationID);

		
		// "protected" just in case new classes will be derived from this one
		protected:
		std::mutex m_already_reported_mutex;
		std::list<uint64_t> m_already_reported; // List of vehicles for which a report has been sent already (potentially use to not trust them)
		
		private:
		ldmmap::LDMMap *m_db_ptr;
		options_t *m_opts_ptr;
		std::map<uint64_t, std::map<uint64_t,uint64_t>> m_recvCPMmap;  //! Structure mapping, for each CV that we have received a CPM from, the CPM's PO ids with the ego LDM's PO ids
		
        bool m_detection_enabled;
		
		std::string m_logfile_name;
		FILE *m_logfile_file;
		areaFilter m_areaFilter;
		indicatorTriggerManager *m_indicatorTrgMan_ptr;
		bool m_indicatorTrgMan_enabled;
		
		inline ldmmap::OptionalDataItem<uint8_t> manage_LowfreqContainer(CAM_t *decoded_cam,uint32_t stationID);
		void decodeCAM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id);
		void decodeCPM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id);
		void decodeVAM(etsiDecoder::etsiDecodedData_t decodedData, proton::message &msg, uint64_t on_msg_timestamp_us, uint64_t main_bf, std::string m_client_id);

};

#endif // MB_DETECTOR_H