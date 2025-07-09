#ifndef DENDATADEF_H
#define DENDATADEF_H

#include <unordered_map>
#include <vector>
#include <shared_mutex>
#include <string>
#include "optionalDataItem.h"

#define eventDataVector_t(name) std::vector<ldmmap::eventData_t> name;

 #define DEG_2_RAD(val) ((val)*M_PI/180.0)
namespace ldmmap{

	typedef enum EventTypeLDM {
		EventType_LDM_unknown = 0,
		EventType_LDM_trafficCondition = 1,
		EventType_LDM_accident = 2,
		EventType_LDM_adverseWeatherCondition_adhesion = 6,
		EventType_LDM_hazardousLocation_SurfaceCondition = 9,
		EventType_LDM_hazardousLocation_ObstacleOnTheRoad = 11,
		EventType_LDM_humanPresenceOnTheRoad = 12,
		EventType_LDM_wrongWayDriving = 14,
		EventType_LDM_rescueAndRecoveryWorkInProgress = 15,
		EventType_LDM_adverseWeatherCondition_Extreme = 17,
		EventType_LDM_adverseWeatherCondition_Visibility = 18,
		EventType_LDM_adverseWeatherCondition_Precipitation = 19,
		EventType_LDM_slowVehicle = 26,
		EventType_LDM_dangerousEndOfQueue = 27,
		EventType_LDM_vehicleBreakdown = 91,
		EventType_LDM_postCrash = 92,
		EventType_LDM_humanProblem = 93,
		EventType_LDM_stationaryVehicle = 94,
		EventType_LDM_emergencyVehicleApproaching = 95,
		EventType_LDM_hazardousLocation_DangerousCurve = 96, 
		EventType_LDM_collisionRisk = 97,
		EventType_LDM_dangerousSituation = 99,
		EventType_LDM_impassability = 5,
		EventType_LDM_aquaplaning = 7,
		EventType_LDM_publicTransportVehicleApproaching = 28,
		EventType_LDM_railwayLevelCrossing = 100,
	} e_EventTypeLDM;


	//For the visualizer
	typedef enum OperationType {
		OperationType_LDM_unknown,
		OperationType_LDM_insert,
		OperationType_LDM_update,
		OperationType_LDM_remove,
		OperationType_LDM_used,
	} e_OperationType_t;

/* Una volta che funziona il tutto, spostare le variabili della struc ALaCarte dentro la struct eventData
	typedef  struct eventALaCarte{
		long eventLanePosition;

		//ImpactReductionContainer 
		long eventHeightLonCarrLeft;
		long eventHeightLonCarrRight;
		long eventPosLonCarrLeft;
		long eventPosLonCarrRight;
		// PositionofPillars ->come lo devo dichiarare?
		long eventPosCentMass;
		long eventWheelBaseVehicle;
		long eventTurningRadius;
		long eventPosFrontAx;
		//PositionOfOccupants ->come lo devo dichiarare?
		uint64_t eventPositionOfOccupant; //è un enum
		long eventVehicleMass;
		long eventRequestResponseIndication;
		long eventTemperature;
		//RoadWorksContainerExtended -> struct annidate. Devo 		
		long eventPositionSolutionType;
		//StationaryVehicleContainer

	}eventALaCarte_t;
	*/
	typedef struct eventData {
		uint64_t on_msg_timestampDENM_us;
		uint64_t gnTimestampDENM;
		uint64_t timestampDENM_us;
		uint64_t insertEventTimestamp_us;


		//MANAGEMENT CONTAINER
		//ActionID
		uint64_t originatingStationID;
		uint64_t sequenceNumber;
		uint64_t detectionTime;
		uint64_t referenceTime;
		OptionalDataItem<uint64_t> eventTermination;
		//eventPosition
		double eventLatitude;
		double eventLongitude;
		double eventElevation;
		double semiMajorConfidence;
		double semiMinorConfidence;
		double semiMajorOrientation;
		OptionalDataItem<double> eventRelevanceDistance; //OPTIONAL
		OptionalDataItem<double>  eventRelevanceTrafficDirection; //OPTIONAL
		uint64_t eventValidityDuration; //DEFAULT defaultValidity
		OptionalDataItem<u_int64_t>eventTransmissionInterval; //OPTIONAL
		double eventStationType;

		//STUATION CONTAINER
		e_EventTypeLDM eventCauseCode;
		/*
		OptionalDataItem<long> eventInformationQuality;
		OptionalDataItem<long> eventCauseCodeType;
		OptionalDataItem<long> eventSubCauseCodeType;

		//LOCATION CONTAINER -> OPTIONAL
		OptionalDataItem<long>  eventSpeed; //speed value
		OptionalDataItem<long>  eventPositionHeading; //heading value
		OptionalDataItem<double>  eventTraces;
		OptionalDataItem<long>  eventRoadType;
		*/
	} eventData_t;
}

#endif //DENDATADEF_H