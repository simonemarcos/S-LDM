#include <map>

#include "osmium/io/any_input.hpp"
#include "osmium/visitor.hpp"
#include "osmium/handler.hpp"
#include "osmium/handler/node_locations_for_ways.hpp"
#include "osmium/osm/way.hpp"
#include "osmium/geom/haversine.hpp"

#include "osmium/handler/node_locations_for_ways.hpp"
#include "osmium/index/map/flex_mem.hpp"

#include "boost/geometry.hpp"
using point2d = boost::geometry::model::d2::point_xy<double,boost::geometry::cs::geographic<boost::geometry::degree>>;
using way_linestring = boost::geometry::model::linestring<point2d>;
using way_polygon = boost::geometry::model::polygon<point2d>;

typedef struct HighwayInfo_s {
    int width;
    double maxSpeed;
} HighwayInfo_t;

class OSMStore {
public:
    
    OSMStore(double minlat, double minlon, double maxlat, double maxlon);

    // returns the highway id that matches the distance constraint, if no highway is found returns -1
    osmium::object_id_type checkIfPointOnRoad(double lat, double lon);

    // return true is heading is consistent with the road, false otherwise
    bool checkHeadingMatchesRoad(double heading, double lat, double lon, osmium::object_id_type highwayID);

    // returns true if position is inside a building, false otherwise
    bool checkIfPointInBuilding(double lat, double lon);

    // return true if speed is over the road type limit
    bool checkSpeedOverTypeLimit(double speed, osmium::object_id_type highwayID);

    void printAll();
private:

    double sumPositionOnRoad=0; double countPositionOnRoad=0;
    double sumHeading=0; double countHeading=0;
    double sumBuilding=0; double countBuilding=0;
    double m_minlat, m_lat_increment;
    double m_minlon, m_lon_increment;

    std::map<osmium::object_id_type,HighwayInfo_t> m_highways_info;
    std::map<std::tuple<int,int>,std::map<osmium::object_id_type,way_linestring>> m_highways_by_sector;

    std::map<std::tuple<int,int>,std::map<osmium::object_id_type,way_polygon>> m_buildings_by_sector;
};