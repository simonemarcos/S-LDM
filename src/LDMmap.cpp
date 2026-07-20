#include "LDMmap.h"
#include "utils.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <functional>
#include "dendatadef.h"
#include "ActionID.h"
#include "etsiDecoderFrontend.h"
#include "timers.h"
#include <atomic>
#include <thread>
#include <vehicle-visualizer.h>



#define DEFINE_KEYS(k_up,k_low,stationID) \
		uint32_t k_low = stationID & 0x0000FFFF; \
		uint32_t k_up = (stationID & 0xFFFF0000) >> 16;

#define COMPOSE_KEYS(k_up,k_low) (((uint64_t) k_up) << 16) + k_low;

#define DEG_2_RAD(val) ((val)*M_PI/180.0)



namespace ldmmap {

	// Key generation for the event (DENM)
	int LDMMap::KEY_EVENT(double latitude, double longitude, double elevation, e_EventTypeLDM TypeEvent){

		double truncated_latitude = std::trunc(latitude*1e6)/1e6;
		double truncated_longitude = std::trunc(longitude*1e6)/1e6;
		double truncated_elevation = std::trunc(elevation*1e2)/1e2;
		std::ostringstream str_lat;
		std::ostringstream str_lon;
		std::ostringstream str_ele;
		std::ostringstream str_TypeEvent;

		// Set precision for the string streams
		str_lat << std::fixed << std::setprecision(6) << truncated_latitude;
		str_lon << std::fixed << std::setprecision(6) << truncated_longitude;
		str_ele << std::fixed << std::setprecision(2) << truncated_elevation;

		str_TypeEvent << std::to_string(TypeEvent);

		std::string strconc = str_ele.str() + str_lat.str() + str_lon.str() + str_TypeEvent.str();
		std::size_t EventKey = std::hash<std::string>{}(strconc);

		return EventKey;
	}

	static inline double haversineDist(double lat_a, double lon_a, double lat_b, double lon_b) {
		// 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
		return 12742000.0*asin(sqrt(sin(DEG_2_RAD(lat_b-lat_a)/2)*sin(DEG_2_RAD(lat_b-lat_a)/2)+cos(DEG_2_RAD(lat_a))*cos(DEG_2_RAD(lat_b))*sin(DEG_2_RAD(lon_b-lon_a)/2)*sin(DEG_2_RAD(lon_b-lon_a)/2)));
	}

	LDMMap::LDMMap() {
		m_card = 0;
		m_eventcard = 0;


		m_central_lat = 0.0;
		m_central_lon = 0.0;
	}

	LDMMap::~LDMMap() {
		clear();
		clearEvent();
	}

	LDMMap::LDMMap_error_t
	LDMMap::insertVehicle(vehicleData_t newVehicleData) {
		LDMMap_error_t retval;
		DEFINE_KEYS(key_upper,key_lower,newVehicleData.stationID);

		if(m_card==UINT64_MAX) {
			return LDMMAP_MAP_FULL;
		}

		std::shared_mutex *mapmutexptr;
		std::unordered_map<uint32_t,returnedVehicleData_t> lowerMap;

		if(m_ldmmap.count(key_upper) == 0) {
			mapmutexptr = new std::shared_mutex();
			lowerMap[key_lower].vehData = newVehicleData;
			lowerMap[key_lower].phData = new PHpoints();

			std::lock_guard<std::shared_mutex> lk(m_mainmapmut);

			m_ldmmap[key_upper] = std::pair<std::shared_mutex*,std::unordered_map<uint32_t,returnedVehicleData_t>>(mapmutexptr,lowerMap);
			m_card++;

			retval = LDMMAP_OK;
		} else {
			if(m_ldmmap[key_upper].second.count(key_lower) == 0) {
				m_card++;
				retval = LDMMAP_OK;
			} else {
				retval = LDMMAP_UPDATED;
			}

			std::lock_guard<std::shared_mutex> lk(*m_ldmmap[key_upper].first);
			m_ldmmap[key_upper].second[key_lower].vehData = newVehicleData;

			// INSERT operation -> create a new PHpoints object
			if(retval == LDMMAP_OK) {
				m_ldmmap[key_upper].second[key_lower].phData = new PHpoints();
			}
		}

		// std::cout << "Updating vehicle: " << newVehicleData.stationID << std::endl;
		m_ldmmap[key_upper].second[key_lower].phData->insert(newVehicleData);

		return retval;
	}

	LDMMap::event_LDMMap_error_t
	LDMMap::insertEvent(eventData_t newEventData, uint64_t eventkey_map) {
		event_LDMMap_error_t reteveval;
		if (m_eventcard == UINT64_MAX) {
			return event_LDMMAP_FULL;
		}
		//printf("EVENTKEY IN INSERT EVENT: %lu\n",eventkey_map); // For debug purposes
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		if (m_eventldmmap.count(eventkey_map)==0) {
			newEventData.insertEventTimestamp_us = get_timestamp_us();

			m_eventldmmap[eventkey_map].eventData = newEventData;
			m_eventcard++;
			reteveval = event_LDMMAP_OK;
		}else {
			reteveval = event_LDMMAP_ITEM_EXIST;
		}

		return reteveval;
	}

	LDMMap::LDMMap_error_t
	LDMMap::removeVehicle(uint64_t stationID) {
		DEFINE_KEYS(key_upper,key_lower,stationID);

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		if(m_ldmmap.count(key_upper) == 0) {
			return LDMMAP_ITEM_NOT_FOUND;
		} else {
			if(m_ldmmap[key_upper].second.count(key_lower) == 0) {
				return LDMMAP_ITEM_NOT_FOUND;
			} else {
				std::lock_guard<std::shared_mutex> lk(*m_ldmmap[key_upper].first);
				m_ldmmap[key_upper].second.erase(stationID);
				m_card--;

			}
		}

		return LDMMAP_OK;
	}

	LDMMap::event_LDMMap_error_t
	LDMMap::removeEvent(uint64_t eventkey_map){

		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		if(m_eventldmmap.count(eventkey_map) == 0){
			return event_LDMMAP_ITEM_NOT_FOUND;
		} else {

			m_eventldmmap.erase(eventkey_map);
			m_eventcard--;
			return event_LDMMAP_REMOVED;
		}
	}

	LDMMap::LDMMap_error_t
	LDMMap::lookupVehicle(uint64_t stationID,returnedVehicleData_t &retVehicleData) {
		DEFINE_KEYS(key_upper,key_lower,stationID);

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		if(m_ldmmap.count(key_upper) == 0) {
			return LDMMAP_ITEM_NOT_FOUND;
		} else {
			if(m_ldmmap[key_upper].second.count(key_lower) == 0) {
				return LDMMAP_ITEM_NOT_FOUND;
			} else {
				std::shared_lock<std::shared_mutex> lk(*m_ldmmap[key_upper].first);
				retVehicleData = m_ldmmap[key_upper].second[key_lower];
			}
		}

		return LDMMAP_OK;
	}

    // This function looks for an event in the event map and updates it if found, or finds a close event with the same cause code
	LDMMap::event_LDMMap_error_t
	LDMMap:: lookupAndUpdateEvent(uint64_t eventkey_map, double LatNewEve, double LonNewEve, uint64_t NewCauseCode,
		eventData_t &UpEveData,returnedEventData_t &UpEventData , uint64_t &closestEvent_key) {
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);
		std::cout << "new eventkey_map: " << eventkey_map << std::endl;
		if (m_eventldmmap.count(eventkey_map)!=0) {
			m_eventldmmap[eventkey_map] = UpEventData;
			return event_LDMMAP_UPDATED;
		}

		double minDistanceEvents = 10;

		// Looking for the closest event with the same causeCode
		for (const auto &[key, val]:m_eventldmmap) {
			if (val.eventData.eventCauseCode == NewCauseCode) {
				double distance = haversineDist(LatNewEve,	LonNewEve,
					val.eventData.eventLatitude, val.eventData.eventLongitude);

				if (distance <= 10.0 && distance <= minDistanceEvents) {
					minDistanceEvents = distance;
					closestEvent_key = key;
					//For debug purposes
					//std::cout <<"Closest EventKey: " << closestEvent_key << std::endl;
					//std::cout <<"minDistanceEvents: " << minDistanceEvents << std::endl;
				}
			}
		}

		// If a close event is found, update its data
		if (closestEvent_key!=0) {
			m_eventldmmap[closestEvent_key].eventData.insertEventTimestamp_us = UpEveData.insertEventTimestamp_us;
			m_eventldmmap[closestEvent_key].eventData.eventValidityDuration = UpEveData.eventValidityDuration;
			return event_LDMMAP_NEAR_EVENT_UPDATED;
		} else {
			return event_LDMMAP_ITEM_NOT_FOUND;
		}
	}

	/*
	LDMMap::event_LDMMap_error_t
	LDMMap::lookupEvent(uint64_t eventkey_map, returnedEventData_t &retEveData, uint64_t &found_event_key) {
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		// Se l'evento con la chiave specificata esiste, restituiamo subito il risultato
		if (m_eventldmmap.count(eventkey_map) != 0) {
			retEveData = m_eventldmmap[eventkey_map];
			found_event_key = eventkey_map;
			return event_LDMMAP_ITEM_EXIST;
		}

		// Se l'evento non è presente con quella chiave, cerchiamo eventi vicini con lo stesso causeCode
		for (const auto &event_entry : m_eventldmmap) {
			double distance = euclideanDistMeters(
				retEveData.eventData.eventLatitude, retEveData.eventData.eventLongitude,
				event_entry.second.eventData.eventLatitude, event_entry.second.eventData.eventLongitude
				);
			std::cout <<"lat exisiting event: " << event_entry.second.eventData.eventLatitude << std::endl;
			std::cout <<"lon exisiting event: " << event_entry.second.eventData.eventLongitude << std::endl;
			std::cout <<"lat new event: " << retEveData.eventData.eventLatitude << std::endl;
			std::cout <<"lon new event: " << retEveData.eventData.eventLongitude << std::endl;
			//std::cout << "Distance: " << distance << std::endl;
			if (distance <= 2.0 && event_entry.second.eventData.eventCauseCode == retEveData.eventData.eventCauseCode) {
				found_event_key = event_entry.first; // Salviamo la chiave dell'evento trovato
				return event_LDMMAP_ITEM_EXIST;
			}
		}

		return event_LDMMAP_ITEM_NOT_FOUND;
	}

	LDMMap::event_LDMMap_error_t
	LDMMap::lookupEvent(uint64_t eventkey_map, returnedEventData_t &retEveData) {
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		if(m_eventldmmap.count(eventkey_map) != 0){
			uint64_t now = get_timestamp_us(); // get the current timestamp
			std::cout << "NOW: " << now << std::endl;

			//Check if the event is expired
			if (now-m_eventldmmap[eventkey_map].eventData.insertEventTimestamp_us >= m_eventldmmap[eventkey_map].eventData.eventValidityDuration*1e3) {

				return event_LDMMAP_ITEM_EXPIRED;
			}
			return event_LDMMAP_ITEM_EXIST;
		} else {
			return event_LDMMAP_ITEM_NOT_FOUND;
		}
	}

	LDMMap::event_LDMMap_error_t
	LDMMap::updateEvent(uint64_t eventkey_map, returnedEventData_t &updatedEventData) {
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		if (m_eventldmmap.count(eventkey_map) != 0) {
			// Retrieve the existing event
			eventData_t &existingEvent = m_eventldmmap[eventkey_map].eventData;

			// Update only the non-coordinate parameters
			existingEvent.eventCauseCode = updatedEventData.eventData.eventCauseCode;
			existingEvent.eventValidityDuration = updatedEventData.eventData.eventValidityDuration;
			//existingEvent.insertEventTimestamp_us = updatedEventData.insertEventTimestamp_us;

			std::cout << "EVENT UPDATED! Key: " << eventkey_map << std::endl;

			return event_LDMMAP_UPDATED;
		}

		return event_LDMMAP_ITEM_NOT_FOUND;
	}

	LDMMap::event_LDMMap_error_t
	LDMMap::updateEvent(uint64_t eventkey_map, returnedEventData_t &updatedEventData) {
		std::shared_lock<std::shared_mutex>lk(m_eventmapmut);

		if(m_eventldmmap.count(eventkey_map)!=0) {
			m_eventldmmap[eventkey_map]= updatedEventData;


			std::cout << "EVENT UPDATED!" << eventkey_map << std::endl;

			return event_LDMMAP_UPDATED;
		}
	}
	*/

	LDMMap::LDMMap_error_t
	LDMMap::rangeSelectVehicle(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles) {
		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto const& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock_shared();

			for(auto const& [keyl, vall] : val.second) {
				if(haversineDist(lat,lon,vall.vehData.lat,vall.vehData.lon)<=range_m) {
					selectedVehicles.push_back(vall);
				}
			}

			val.first->unlock_shared();
		}

		return LDMMAP_OK;
	}


	LDMMap::LDMMap_error_t
	LDMMap::rangeSelectVehicle(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles) {
		returnedVehicleData_t retData;

		// Get the latitude and longitude of the speficied vehicle
		if(lookupVehicle(stationID,retData)!=LDMMAP_OK) {
			return LDMMAP_ITEM_NOT_FOUND;
		}

		// Perform a rangeSelect() centered on that latitude and longitude values
		return rangeSelectVehicle(range_m,retData.vehData.lat,retData.vehData.lon,selectedVehicles);
	}

	void
	LDMMap::deleteVehicleOlderThan(double time_milliseconds) {
		uint64_t now = get_timestamp_us();

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock();

			for (auto mit=val.second.cbegin();mit!=val.second.cend();) {
				if(((double)(now-mit->second.vehData.timestamp_us))/1000.0 > time_milliseconds) {
					mit = val.second.erase(mit);
					m_card--;
				} else {
					++mit;
				}
			}

			val.first->unlock();
		}
	}


	void
	LDMMap::deleteEventOlderThan(returnedEventData_t validityDuration) {
		uint64_t evenow = get_timestamp_us();
		//std::lock_guard<std::shared_mutex> lk(m_eventmainmapmut);
		for (auto eit = m_eventldmmap.begin(); eit != m_eventldmmap.end();) {
			if ((double) ((evenow-eit -> second.eventData.eventValidityDuration)/1000.0) > validityDuration.eventData.eventValidityDuration*1000) {
				eit = m_eventldmmap.erase(eit);
				m_eventcard--;
			} else {
				++eit;
			}
		}
	}

	void
	LDMMap::deleteVehicleOlderThanAndExecute(double time_milliseconds,void (*oper_fcn)(uint64_t,void *),void *additional_args) {
		uint64_t now = get_timestamp_us();
		//std::cout << "NOW V: " << now << std::endl;

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock();

			for (auto mit=val.second.cbegin();mit!=val.second.cend();) {
				if(((double)(now-mit->second.vehData.timestamp_us))/1000.0 > time_milliseconds) {
					// With respect to deleteOlderThan(), this function will also call oper_fcn() for each deleted entry
					oper_fcn(mit->second.vehData.stationID,additional_args);
					mit = val.second.erase(mit);

					m_card--;
				} else {
					++mit;
				}
			}

			val.first->unlock();
		}
	}

	// This function deletes all the events older than a certain validity duration
	void LDMMap::deleteEventOlderThanAndExecute( void (*oper_fcn)(uint64_t, void *), void *additional_args) {
		uint64_t now = get_timestamp_us();
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		// Iterate over the event map and remove events that are older than their validity duration
		for (auto it = m_eventldmmap.cbegin(); it != m_eventldmmap.cend();) {
			// Convert the validity duration from seconds to milliseconds
			uint64_t validityDuration = it->second.eventData.eventValidityDuration*1000.0;
/*
			// For debug purposes
			std::cout <<"\nNOW: "<<now<< " - KEY: " << it->first << std::endl;
			std::cout <<"Insert timestamp: " << it->second.eventData.insertEventTimestamp_us << " - KEY: " << it->first << std::endl;
			std::cout << "Validity duration: " << validityDuration << " - KEY: " << it->first << std::endl;
			std::cout << "TIME_elapsed: " << (double) ((now - it->second.eventData.insertEventTimestamp_us) / 1000.0) << " - KEY: " << it->first << std::endl;
*/
			if (((double)(now - it->second.eventData.insertEventTimestamp_us)) / 1000.0 > validityDuration) {
				// Chiama la funzione di callback per ogni evento rimosso
				oper_fcn(it->first, additional_args);
				it = m_eventldmmap.erase(it);
				m_eventcard--;
				std::cout <<"\n"<<std::endl;
			} else {
				++it;
			}
		}
	}

	void
	LDMMap::clear() {
		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps and clear them
			// Delete also the shared mutex objects

			for(auto const& [keyl, vall] : val.second) {
				vall.phData->clear();
				delete vall.phData;
			}

			val.second.clear();
			delete val.first;
		}

		// Clear the upper map
		m_ldmmap.clear();

		// Set the cardinality of the map to 0 again
		m_card = 0;
	}

	void LDMMap::clearEvent() {
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		// Clear the event map
		m_eventldmmap.clear();

		// Set the cardinality of the map to 0 again
		m_eventcard = 0;
	}

	void
	LDMMap::printAllVehicleContents(std::string label) {
		std::cout << "\n[" << label << "] Vehicle IDs: ";

		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto const& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock_shared();

			for(auto const& [keyl, vall] : val.second) {
				std::cout << vall.vehData.stationID << ", ";
			}

			val.first->unlock_shared();
		}

		std::cout << std::endl;
	}

	void
	LDMMap::printAllEventContents (std::string label) {
		std::cout << "[" << label << "] Event IDs: ";

		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);
		uint64_t i = 0;
		for(auto const& [key, val] : m_eventldmmap) {
			std::cout <<"EVENT_KEY: "<< key <<" - number of event: " << i << "\n";
			i++;
		}
		std::cout << std::endl;
	}

/*
	void
	LDMMap::printEventDetails (uint64_t eventKeyMap, std::string label) {
		//std::shared_lock<std::shared_mutex> lk(m_eventmainmapmut);

		if (m_eventldmmap.count(eventKeyMap)==0) {
			std::cout << "[" << label << "] Event with key " << eventKeyMap << " not found" << std::endl;
			return;
		}
		const eventData_t &eventData = m_eventldmmap[eventKeyMap];

		std::cout << "[" << label << "] Event Details: " << std::endl;

		stampEventData(eventData);
	}
	*/

	void
	LDMMap::executeOnAllVehicleContents(void (*oper_fcn)(vehicleData_t,void *),void *additional_args) {
		std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

		for(auto const& [key, val] : m_ldmmap) {
			// Iterate over the single lower maps
			val.first->lock_shared();

			for(auto const& [keyl, vall] : val.second) {
				// Execute the callback for every entry
				oper_fcn(vall.vehData,additional_args);
			}

			val.first->unlock_shared();
		}
	}

	void
	LDMMap::executeOnAllEventContents(void (*oper_fcn)(eventData_t,uint64_t key,void *),void *additional_args) {
		std::shared_lock<std::shared_mutex> lk(m_eventmapmut);

		for(auto const& [key, val] : m_eventldmmap) {
				// Execute the callback for every entry
				oper_fcn(val.eventData,key,additional_args);
		}

	}

	LDMMap::LDMMap_error_t
    LDMMap::getAllIDsVehicles(std::set<uint64_t> &selectedIDs) {
        std::shared_lock<std::shared_mutex> lk(m_mainmapmut);

        for(auto const& [key, val] : m_ldmmap) {
            // Iterate over the single lower maps
            val.first->lock_shared();

            for(auto const& [keyl, vall] : val.second) {
                selectedIDs.insert(vall.vehData.stationID);
            }

            val.first->unlock_shared();
        }
        return LDMMAP_OK;
    }

}
