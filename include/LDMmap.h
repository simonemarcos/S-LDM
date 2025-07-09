#ifndef LDMMAP_H
#define LDMMAP_H

#include <atomic>
#include <unordered_map>
#include <vector>
#include <shared_mutex>
#include "PHpoints.h"
#include "vehicleDataDef.h" 
#include "dendatadef.h" 
#include "etsiDecoderFrontend.h"
#include "timers.h"
#include <thread>


extern std::atomic<bool> eventMapModified;

namespace ldmmap {
	class LDMMap {
		public:
	    	typedef enum {
	    		LDMMAP_OK,
	    		LDMMAP_UPDATED,
	    		LDMMAP_ITEM_NOT_FOUND,
	    		LDMMAP_MAP_FULL,
	    		LDMMAP_UNKNOWN_ERROR
	    	} LDMMap_error_t;

	    	typedef struct {
	    		vehicleData_t vehData;
	    		PHpoints *phData;
	    	} returnedVehicleData_t;

			typedef struct {
				eventData_t eventData;
			}returnedEventData_t;

	    	typedef enum {
	    		event_LDMMAP_OK,
	    		event_LDMMAP_ITEM_EXIST,
	    		event_LDMMAP_UPDATED,
	    		event_LDMMAP_NEAR_EVENT_UPDATED,
	    		event_LDMMAP_ITEM_EXPIRED,
	    		event_LDMMAP_REMOVED,
	    		event_LDMMAP_ITEM_NOT_FOUND,
	    		event_LDMMAP_FULL,
	    		event_LDMMAP_UNKNOWN_ERROR
	    	} event_LDMMap_error_t;

	    	typedef enum {
	    		LDMMap_insert_OK,
	    		LDMMAP_insert_error
	    	} LDMMap_insert_t;



	    	LDMMap();
	    	~LDMMap();

	    	// This function clears the whole database (to be used only when the dabatase and its content is not going to be accessed again)
	    	void clear();
	    	// This function inserts or updates a vehicle in the database
	    	// The vehicle data should be passed inside a vehicleData_t structure and vehicles are univocally identified by their stationID
	    	// This function returns LDMMAP_OK is a new vehicle has been inserted, LDMMAP_UPDATED is an existing vehicle entry has been updated,
	    	// LDMMAP_MAP_FULL if the database if full and the insert operation failed (this should never happen, in any case)
	    	LDMMap_error_t insertVehicle(vehicleData_t newVehicleData);
	    	// This function removes from the database the vehicle entry with station ID == stationID
	    	// It returns LDMMAP_ITEM_NOT_FOUND if no vehicle with the given stationID was found for removal
	    	// It returns LDMMAP_OK if the vehicle entry was succesfully removed
	    	LDMMap_error_t removeVehicle(uint64_t stationID);
	    	// This function returns the vehicle entry with station ID == stationID - the entry data is returned in retVehicleData
	    	// This function returns LDMMAP_ITEM_NOT_FOUND if no vehicle with the given stationID was found for removal, while
	    	// it returns LDMMAP_OK if the retVehicleData structure was properly filled with the requested vehicle data
	    	LDMMap_error_t lookupVehicle(uint64_t stationID, returnedVehicleData_t &retVehicleData);
	    	// This function returns a vector of vehicles, including their Path History points, located within a certain radius 
	    	// centered on a given latitude and longitude
	    	// For the time being, this function should always return LDMMAP_OK (i.e. to understand if no vehicles are returned, 
	    	// you should check the size of the selectedVehicles vector)
	    	LDMMap_error_t rangeSelectVehicle(double range_m, double lat, double lon, std::vector<returnedVehicleData_t> &selectedVehicles);
	    	// This function is the same as the other method with the same name, but it will return all the vehicles around another
	    	// vehicle (which is also included in the returned vector), given it stationID
	    	// This function may return LDMMAP_ITEM_NOT_FOUND if the specified stationID is not stored inside the database
	    	LDMMap_error_t rangeSelectVehicle(double range_m, uint64_t stationID, std::vector<returnedVehicleData_t> &selectedVehicles);
	    	// This function deletes from the database all the entries older than time_milliseconds ms
	    	// The entries are deleted if their age is > time_milliseconds ms if greater_equal == false, 
	    	// or >= time_milliseconds ms if greater_equal == true
	    	// This function performs a full database read operation
	    	void deleteVehicleOlderThan(double time_milliseconds);
	    	// This function is a combination of deleteOlderThan() and executeOnAllContents(), calling the open_fcn()
	    	// callback for every deleted entry
	    	void deleteVehicleOlderThanAndExecute(double time_milliseconds,void (*oper_fcn)(uint64_t,void *),void *additional_args);
	    	// This function can be used to print all the content of the database
	    	// The stationIDs of the vehicles stored in the LDMMap database will be printed, preceded by an optional string,
	    	// specified with "label"
	    	void printAllVehicleContents(std::string label = "");
	    	// This function reads the whole database, and, for each entry, it executes the "oper_fcn" callback
	    	// This callback should return void (i.e. nothing) and have two arguments:
	    	// - a vehicleData_t structure, in which the data stored in each entry will be made available to the callback
	    	// - a void * pointer, by means of which possible additional arguments can be passed to the callback
	    	// The additional arguments to be passed to the callback, each time it is called, can be specified by setting
	    	// void *additional_args to a value different than "nullptr"
	    	// If additional_args == nullptr, also the second argument of each callback call will be nullptr
	    	void executeOnAllVehicleContents(void (*oper_fcn)(vehicleData_t,void *),void *additional_args);

	    	// This setter sets two facility (private) variables which are used solely for passing a central 
	    	// latitude and longitude to the vehicle visualizer, extracting them from an LDMMap object
	    	// The same applies to the corresponding getter method
	    	void setCentralLatLon(double lat, double lon) {m_central_lat = lat; m_central_lon = lon;}
	    	std::pair<double,double> getCentralLatLon() {return std::make_pair(m_central_lat,m_central_lon);}

	    	int getVehicleCardinality() {return m_card;};
			LDMMap_error_t getAllIDsVehicles(std::set<uint64_t> &selectedIDs);
	    	
	    	//Event functions

			// This function inserts a new event in the event database
			// The event data should be passed inside a eventdata_t structure and events are univocally identified by their keyEventMap
			// This function returns event_LDMMAP_OK if a new event has been inserted, event_LDMMAP_ITEM_EXIST if an existing event is  has been updated,
			// LDMMAP_MAP_FULL if the database if full and the insert operation failed (this should never happen, in any case)
	    	event_LDMMap_error_t insertEvent(eventData_t newEventData, uint64_t eventkey_map);

			// This function removes from the database the event entry with keyEvent == keyEvent
			// It returns event-LDMMAP_ITEM_NOT_FOUND if no event with the given keyEvent was found for removal
			// It returns event_LDMMAP_REMOVED if the event entry was succesfully removed
	    	event_LDMMap_error_t removeEvent(uint64_t KeyEventMap);

			// This function looks up an event in the database, given its keyEventMap. If the event is found in the database, the function returns
			// event_LDMMAP_UPDATED and fills the retEveData structure with the event data. Otherwise, it seeks for an existing event in a radius
			// of 10m with the same cause code. If such an event is found, it updates the event data and returns event_LDMMAP_NEAR_EVENT_UPDATED.
			// If no event is found, it returns event_LDMMAP_ITEM_NOT_FOUND.
			event_LDMMap_error_t lookupAndUpdateEvent(uint64_t eventkey_map,double LatNewEve, double LonNewEve, uint64_t NewCauseCode,
				eventData_t &UpEveData,returnedEventData_t &UpEventData, uint64_t &nearEvent_key);

		    // to implement the lookupAndUpdateEvent() function, we need to find an existing event with the same cause code
	    	event_LDMMap_error_t lookupEvent(uint64_t EventKey, returnedEventData_t &retEveData, uint64_t &found_event_key);

			//event_LDMMap_error_t updateEvent(uint64_t EventKey, returnedEventData_t &updateEventData);

	    	//event_LDMMap_error_t eventrangeselect(double eventrange_m, double evelat, double evelon, double elevation, std::vector<returnedVehicleData_t> &selectedVehicles);

	    	void deleteEventOlderThan(returnedEventData_t validityDuration);

			void clearEvent();

			void printAllEventContents(std::string label = "");

            //void printEventDetails ( uint64_t EventKey, std::string label = "");

			void deleteEventOlderThanAndExecute(void (*oper_fcn)(uint64_t,void *),void *additional_args);

			//void deleteEventOlderThanAndExecute(uint64_t validityDuration,void (*oper_fcn)(uint64_t,void *),void *additional_args);

			// This function reads the whole database, and, for each entry, it executes the "oper_fcn" callback
			// This callback should return void (i.e. nothing) and have two arguments:
			// - a eventData_t structure, in which the data stored in each entry will be made available to the callback
			// - a void * pointer, by means of which possible additional arguments can be passed to the callback
			// The additional arguments to be passed to the callback, each time it is called, can be specified by setting
			// void *additional_args to a value different than "nullptr"
			// If additional_args == nullptr, also the second argument of each callback call will be nullptr
			void executeOnAllEventContents(void (*oper_fcn)(eventData_t,uint64_t,void *),void *additional_args);

	    	int getEventCardinality() {return m_eventcard;};

			//static std::size_t KEY_EVENT(double latitude, double longitude, double elevation, e_EventTypeLDM TypeEvent);
	    	int KEY_EVENT(double latitude, double longitude, double elevation, e_EventTypeLDM TypeEvent);

		private:
			// Main database structure
			std::unordered_map<uint32_t,std::pair<std::shared_mutex*,std::unordered_map<uint32_t,returnedVehicleData_t>>> m_ldmmap;
			// Database cardinality (number of entries stored in the database)
			uint64_t m_card;
			// Shared mutex protecting the main database structure
			std::shared_mutex m_mainmapmut;

			// Facility variables representing a central point around which all vehicle entries are ideally located
			// They can be gathered and set using the setCentralLatLon() and getCentralLatLon() methods
			// By default, they are both set to 0.0
			double m_central_lat;
			double m_central_lon;

			//Main database structure for events
			std::unordered_map<uint64_t,returnedEventData_t> m_eventldmmap;
			// Shared mutex protecting the event database structure
			std::shared_mutex m_eventmapmut;
			// Cardinality of the event database
			uint64_t m_eventcard;
			//std::shared_mutex m_eventcardmut;

			double m_eventcentral_lat;
			double m_eventcentral_lon;
	};
}

#endif // LDMMAP_H