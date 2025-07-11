#ifndef VEHDATADEF_H
#define VEHDATADEF_H

#include <unordered_map>
#include <vector>
#include <shared_mutex>
#include "optionalDataItem.h"

#define vehicleDataVector_t(name) std::vector<ldmmap::vehicleData_t> name;

// Facility macro to convert from DEG to RAD
#define DEG_2_RAD(val) ((val)*M_PI/180.0)

// Unavailable value for the heading
#define LDM_HEADING_UNAVAILABLE 3601.0

namespace ldmmap {

	typedef enum StationTypeLDM {
		StationType_LDM_unknown = 0,
		StationType_LDM_pedestrian = 1,
		StationType_LDM_cyclist	= 2,
		StationType_LDM_moped = 3,
		StationType_LDM_motorcycle = 4,
		StationType_LDM_passengerCar = 5,
		StationType_LDM_bus	= 6,
		StationType_LDM_lightTruck = 7,
		StationType_LDM_heavyTruck = 8,
		StationType_LDM_trailer = 9,
		StationType_LDM_specialVehicles	= 10,
		StationType_LDM_tram = 11,
		StationType_LDM_roadSideUnit = 15,
		StationType_LDM_specificCategoryVehicle1 = 100,
		StationType_LDM_specificCategoryVehicle2 = 101,
		StationType_LDM_specificCategoryVehicle3 = 102,
		StationType_LDM_specificCategoryVehicle4 = 103,
		StationType_LDM_specificCategoryVehicle5 = 104,
		StationType_LDM_specificCategoryVehicle6 = 105,
		StationType_LDM_specificCategoryVehicle7 = 106,
		StationType_LDM_specificCategoryVehicle8 = 107,
		StationType_LDM_specificCategoryVehicle9 = 108,
		StationType_LDM_detectedPedestrian = 110,
		StationType_LDM_detectedPassengerCar = 115,
		StationType_LDM_detectedTruck = 117,

		StationType_LDM_unspecified= 120
	} e_StationTypeLDM;

	typedef enum DataUnavailableValue {
		speed=UINT64_MAX,
		longitudinalAcceleration=INT16_MAX,
		curvature=INT16_MAX,
		driveDirection=UINT8_MAX,
		yawRate=INT16_MAX,
		heading=UINT64_MAX,
	} e_DataUnavailableValue;

	// This structure contains all the data stored in the database for each vehicle (except for the PHPoints)
	typedef struct vehicleData {
		uint64_t stationID;
		double lat;
		double lon;
		double elevation;
		double heading; // Heading between 0 and 360 degrees
		double speed_ms;
		uint64_t gnTimestamp;
		long camTimestamp; // This is the CAM message GenerationDeltaTime
		uint64_t timestamp_us;
		uint64_t on_msg_timestamp_us;
		OptionalDataItem<long> vehicleWidth;
		OptionalDataItem<long> vehicleLength;
		e_StationTypeLDM stationType;

		std::string sourceQuadkey;

		// Low frequency container data
		OptionalDataItem<uint8_t> exteriorLights; // Bit string with exterior lights status
		
		// CPM patch
		uint64_t lastCPMincluded;
		uint64_t perceivedBy;
		long xDistance;
		long yDistance;
		long xSpeed;
		long ySpeed;
		//long xAcc;
		//long yAcc;
		//long longitudinalAcceleration;
		long confidence;
		long angle;
		bool detected;

		// VAM patch
		uint8_t macaddr[6];
		double rssi_dBm;
		std::string ipaddr;
		std::string publicipaddr;

		//patch MBD
		uint8_t driveDirection;
		int16_t longitudinalAcceleration;
		int16_t curvature;
		int16_t yawRate;

		//patch security
		std::string certDigest;
	} vehicleData_t;
}

#endif // VEHDATADEF_H
