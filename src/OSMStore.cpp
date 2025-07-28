#include "OSMStore.h"

#include "asn_utils.h"
#include "utils.h"
#include "curl/curl.h"

#include <iostream>
#include <filesystem>

#include "boost/geometry/geometries/polygon.hpp"


std::map<osmium::object_id_type,HighwayInfo_t> highways_info;
std::map<osmium::object_id_type,way_linestring> highways_linestrings;
std::map<osmium::object_id_type,way_polygon> buildings_polygons;

std::map<std::string,double> maxSpeedByRoadType={
    {"",14},
    {"road",14},
    {"motorway",14},
    {"trunk",14},
    {"primary",14},
    {"secondary",14},
    {"tertiary",14},
    {"unclassified",14},
    {"residential",14},
    {"motorway_link",14},
    {"trunk_link",14},
    {"primary_link",14},
    {"secondary_link",14},
    {"tertiary_link",14},
    {"living_street",14},
    {"service",14},
    {"track",14},
    {"escape",14},
    {"raceway",14},
};

class CustomHandler : public osmium::handler::Handler {
public:
    static void way(const osmium::Way& way) {
        // check if it's a building or a highway
        if (way.tags().get_value_by_key("building","")!="") { // check if building
            way_polygon newPolygon;
            for (osmium::NodeRef nr:way.nodes()) {
                boost::geometry::append(newPolygon.outer(),point2d(nr.location().lat(),nr.location().lon()));
            }
            buildings_polygons.emplace(way.id(),newPolygon);
        } else if (way.tags().get_value_by_key("highway","")!="") { // check if highway (could be avoided since we only query buildings and highways)
            way_linestring newLinestring;
            for (osmium::NodeRef nr:way.nodes()) {
                newLinestring.emplace_back(point2d(nr.location().lat(),nr.location().lon()));
            }
            highways_linestrings.emplace(way.id(),newLinestring);
    
            HighwayInfo_t info;
            std::string width_str=way.tags().get_value_by_key("width","");
            width_str="";
            if (width_str=="") {
                width_str=way.tags().get_value_by_key("lanes","");
                if (width_str=="") {
                    info.width=3;
                } else {
                    info.width=std::stoi(width_str)*3;
                }
            } else {
                info.width=std::stoi(width_str);
            }
            std::string maxSpeed_str=way.tags().get_value_by_key("maxspeed","");
            // possibly need to insert the measuring unit check
            maxSpeed_str="";
            if (maxSpeed_str=="") {
                // if no maxspeed tag is present get the default for the road type
                maxSpeed_str=way.tags().get_value_by_key("highway","");
                info.maxSpeed=maxSpeedByRoadType[maxSpeed_str];
            } else {
                info.maxSpeed=std::stof(maxSpeed_str);
            }
            highways_info.emplace(way.id(),info);
        }
    }


};

size_t WriteCallback(void* ptr, size_t size, size_t nmemb, void* stream) {
    size_t written = fwrite(ptr, size, nmemb, (FILE *)stream);
    return written;
}

OSMStore::OSMStore(double minlat, double minlon, double maxlat, double maxlon) {
    // CHECK FOR EXISTING CACHE
    double minlat_r, minlon_r, maxlat_r, maxlon_r, last_modified;
    int cached;
    FILE* cache=fopen("last_bounding_box.txt","r");
    if (cache==nullptr) {
        std::cout <<"[INFO] [OSMStore] No cache file found."  <<std::endl;
        cached=0;
    } else {
        cached=fscanf(cache,"%lf",&last_modified);
        if (cached==EOF) {
            std::cout <<"[INFO] [OSMStore] Empty cache." <<std::endl;
            cached=0;
        } else {
            fscanf(cache,"%lf %lf %lf %lf",&minlat_r,&minlon_r,&maxlat_r,&maxlon_r);
            if (minlat_r!=minlat || minlon_r!=minlon || maxlat_r!=maxlat || maxlon_r!=maxlon) {
                std::cout <<"[INFO] [OSMStore] Cached data is for different bounding box." <<std::endl;
                cached=0;
            } else if (get_timestamp_s()-last_modified>2628000) { // data 1 month old
                std::cout <<"[INFO] [OSMStore] Cached data is too old." <<std::endl;
                cached=0;
            } else {
                std::cout <<"[INFO] [OSMStore] Valid cache found." <<std::endl;
            }
        }
        fclose(cache);
    }
    // IF CACHE NOT PRESENT OR OUTDATED WRITE CACHE FILES
    if (!cached) {
        // DELETE OLD MAP DATA (not needed with single file cache)
        //std::filesystem::path folder_path = "maps";
        //for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
        //    if (std::filesystem::is_regular_file(entry.status())) {
        //        try {
        //            std::filesystem::remove(entry.path());
        //        } catch (const std::filesystem::filesystem_error& ex) {
        //            std::cerr << "Error deleting " << entry.path() << ": " << ex.what() << std::endl;
        //        }
        //    }
        //}
        cache=fopen("last_bounding_box.txt","w");
        last_modified=get_timestamp_s();
        fprintf(cache,"%lf\n%lf %lf %lf %lf",last_modified,minlat,minlon,maxlat,maxlon);
        fclose(cache);
        
        // LIBCURL FETCH DATA ONLY IF NOT CACHED
        std::string minlat_s=std::to_string(minlat);
        std::string minlon_s=std::to_string(minlon);
        std::string maxlat_s=std::to_string(maxlat);
        std::string maxlon_s=std::to_string(maxlon);
        
        std::string url="https://overpass-api.de/api/interpreter";
        std::string query=
            "("
                // [highway=pedestrian] may be used by vehicles, remove from filtered out tags if necessary
                "way[highway]"
                "[highway!=\"busway\"][highway!=\"cycleway\"][highway!=\"footway\"][highway!=\"path\"]"
                "[highway!=\"corridor\"][highway!=\"steps\"][highway!=\"bridleway\"][highway!=\"bus_guideway\"]"
                "[highway!=\"via_ferrata\"][highway!=\"proposed\"][highway!=\"construction\"][highway!=\"pedestrian\"]"
                "("+minlat_s+","+minlon_s+","+maxlat_s+","+maxlon_s+");"

                "way[building]("+minlat_s+","+minlon_s+","+maxlat_s+","+maxlon_s+");"
            ");"
            "(._;>;);"
            "out;";
        CURL *curl=curl_easy_init();
        curl_easy_setopt(curl,CURLOPT_URL,url.c_str()); 
        curl_easy_setopt(curl,CURLOPT_POST, 1L);
        curl_easy_setopt(curl,CURLOPT_POSTFIELDS,query.c_str());
        curl_easy_setopt(curl,CURLOPT_POSTFIELDSIZE,query.length());

        std::string file="maps/map.osm";
        FILE *fp=fopen(file.c_str(),"w");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);

        CURLcode res=curl_easy_perform(curl);
        fclose(fp);
        if (res!=CURLE_OK) {
            std::cout <<"Curl error: " <<res <<std::endl;
        }
        curl_easy_cleanup(curl);
    }

    // LIBOSMIUM READ FILE
    try {
        osmium::io::File input_file{"maps/map.osm"};
        osmium::io::Reader reader{input_file};

        CustomHandler handler;
        highways_linestrings.clear();
        buildings_polygons.clear();

        osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location> index;
        osmium::handler::NodeLocationsForWays<osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>> loc_handler{index};

        osmium::apply(reader,loc_handler,handler);

        reader.close();
        //std::remove("map.osm");

        for (auto w:highways_info) {
            m_highways_info.emplace(w.first,w.second);
        }
    } catch(const std::exception &e) {
        std::cerr <<e.what() <<std::endl;
    }

    int max_size=250;
    m_minlat=minlat;
    m_minlon=minlon;
    int lat_slice=1;
    // distance of 2 bottom corners
    double lat_dist=haversineDist(minlat,minlon,maxlat,minlon);
    if (lat_dist>max_size) {
        lat_slice=std::ceil(lat_dist/max_size);
    }
    int lon_slice=1;
    // distance of 2 left corners
    double lon_dist=haversineDist(minlat,minlon,minlat,maxlon);
    if (lon_dist>max_size) {
        lon_slice=std::ceil(lon_dist/max_size);
    }
    m_lat_increment=(maxlat-minlat)/lat_slice;
    m_lon_increment=(maxlon-minlon)/lon_slice;
    double lat=minlat;
    double lon=minlon;
    int i=0, j=0;
    double tA=get_timestamp_us();
    while (i<lat_slice) {
        lon=minlon;
        j=0;
        while (j<lon_slice) {
            std::map<osmium::object_id_type,way_linestring> new_highways_linestrings;
            std::map<osmium::object_id_type,way_polygon> new_buildings_polygons;
            way_polygon sectorPolygon;
            boost::geometry::append(sectorPolygon.outer(),point2d(lat,lon));
            boost::geometry::append(sectorPolygon.outer(),point2d(lat+m_lat_increment,lon));
            boost::geometry::append(sectorPolygon.outer(),point2d(lat+m_lat_increment,lon+m_lon_increment));
            boost::geometry::append(sectorPolygon.outer(),point2d(lat,lon+m_lon_increment));
            for (auto w:highways_linestrings) {
                if (boost::geometry::intersects(w.second,sectorPolygon)) {
                    new_highways_linestrings.emplace(w);
                }
            }
            for (auto b:buildings_polygons) {
                if (boost::geometry::intersects(b.second,sectorPolygon)) {
                    new_buildings_polygons.emplace(b);
                }
            }
            m_highways_by_sector.emplace(std::tuple<int,int>(i,j),new_highways_linestrings);
            m_buildings_by_sector.emplace(std::tuple<int,int>(i,j),new_buildings_polygons);
            j++;
            lon+=m_lon_increment;
        }
        i++;
        lat+=m_lat_increment;
    }
    double tB=get_timestamp_us();
    std::cout <<"[INFO] [OSMStore] Time for api call+file read in ms: " <<(tB-tA)/1000.0 <<std::endl;

    std::cout <<"[INFO] [OSMStore] OSM Data retrieved." <<std::endl;
    //printAll();
}

void OSMStore::printAll() {
    FILE *fout=fopen("output.txt","w");
    for (auto sec:m_highways_by_sector) {
        fprintf(fout,"\nSector: %d, %d",std::get<0>(sec.first),std::get<1>(sec.first));
        //std::cout <<"\nSector: " <<std::get<0>(sec.first) <<", " <<std::get<1>(sec.first) <<std::endl;
        for (auto w:sec.second) {
            fprintf(fout,"\n\tWay ID: %ld Node count: %d Width: %d Max Speed: %ld",w.first,w.second.size(),m_highways_info.at(w.first).width,m_highways_info.at(w.first).maxSpeed);
            //std::cout <<"Way ID: " <<w.first;
            //std::cout <<" Node count: " <<w.second.size();
            //std::cout <<" Width: " <<m_highways_info.at(w.first).width;
            //std::cout <<" Max Speed: " <<m_highways_info.at(w.first).maxSpeed <<std::endl;
        }
    }
    fclose(fout);
}

osmium::object_id_type OSMStore::checkIfPointOnRoad(double lat, double lon) {
    osmium::object_id_type closestWay;
    double closestDist=2e10;

    double tA=get_timestamp_us();

    point2d p(lat,lon);
    int i=std::floor((lat-m_minlat)/m_lat_increment);
    int j=std::floor((lon-m_minlon)/m_lon_increment);
    std::tuple<int,int> index(i,j);

    int counter=0;
    for (auto way:m_highways_by_sector.at(index)) {
        double dist=boost::geometry::distance(p,way.second);
        if (dist<closestDist) {
            closestDist=dist;
            closestWay=way.first;
            if (dist<3) break;
        }
    }

    double tB=get_timestamp_us();
    sumPositionOnRoad+=tB-tA;
    countPositionOnRoad++;
    //std::cout <<"\nTime for onRoad calculations in us: " <<tB-tA;
    //std::cout <<"\nCurrent average time for onRoad calculations in us: " <<sumPositionOnRoad/countPositionOnRoad;

    if (closestDist<3) {
        //std::cout <<"\nClose enough. Closest way: " <<closestWay <<" Distance: " <<closestDist <<std::endl;
    } else {
        closestWay=-1;
        //std::cout <<"\nNot close enough. Closest way: " <<closestWay <<" Distance: " <<closestDist <<std::endl;
    }

    return closestWay;
}

bool OSMStore::checkHeadingMatchesRoad(double heading, double lat, double lon, osmium::object_id_type highwayID) {
    bool retval=false;
    double tolerance=0.2;

    int closestIndex;
    double closestDist=2e10;    

    double tA=get_timestamp_ns();

    int i=std::floor((lat-m_minlat)/m_lat_increment);
    int j=std::floor((lon-m_minlon)/m_lon_increment);
    std::tuple<int,int> index(i,j);
    way_linestring way=m_highways_by_sector.at(index).at(highwayID);

    point2d p(lat,lon);
    for (int i=0;i<way.size();i++) {
        double dist=boost::geometry::distance(p,way.at(i));
        if (dist<closestDist) {
            closestIndex=i;
        }
    }

    const double radiansFactor=M_PI/180;
    double dLat;
    double dLon;
    //std::cout <<"\nHeading: " <<heading;
    if (closestIndex>0) {
        double dLat = (way.at(closestIndex).x() - way.at(closestIndex-1).x()) * radiansFactor;
        double dLon = (way.at(closestIndex).y() - way.at(closestIndex-1).y()) * radiansFactor;
        const double prevToClosestNodeHeading=fmod((atan2(dLon,dLat)/radiansFactor)+360,360); // "heading" from node before closest and closest
        //std::cout <<"\nPrevious to closest heading:" <<prevToClosestNodeHeading;
        if (heading<prevToClosestNodeHeading*(1+tolerance) && heading>prevToClosestNodeHeading*(1+tolerance)) {
            retval=true;
        } else { //to check opposite driving direction needs to be skipped if the road is one-way only
            heading=fmod(heading+180,360);
            if (heading<prevToClosestNodeHeading*(1+tolerance) && heading>prevToClosestNodeHeading*(1+tolerance)) {
                retval=true;
            }
        }
    }

    if (closestIndex<way.size()-1) {
        const double dLat = (way.at(closestIndex+1).x() - way.at(closestIndex).x()) * radiansFactor;
        const double dLon = (way.at(closestIndex+1).y() - way.at(closestIndex).y()) * radiansFactor;
        const double closestToNextNodeHeading=fmod((atan2(dLon,dLat)/radiansFactor)+360,360); // "heading" from closest node to the one after
        //std::cout <<"\nClosest to next heading: " <<closestToNextNodeHeading;
        if (heading<closestToNextNodeHeading*(1+tolerance) && heading>closestToNextNodeHeading*(1+tolerance)) {
            retval=true;
        } else { //to check opposite driving direction needs to be skipped if the road is one-way only
            heading=fmod(heading+180,360);
            if (heading<closestToNextNodeHeading*(1+tolerance) && heading>closestToNextNodeHeading*(1+tolerance)) {
                retval=true;
            }
        }
    }

    double tB=get_timestamp_ns();
    sumHeading+=tB-tA;
    countHeading++;
    //std::cout <<"\nResult: " <<retval?"true":"false";
    //std::cout <<"\nTime for heading calculations in us: " <<(tB-tA)/1000.0;
    //std::cout <<"\nCurrent average time for heading calculations in us: " <<sumHeading/countHeading/1000.0 <<std::endl;

    return retval;
}

bool OSMStore::checkIfPointInBuilding(double lat, double lon) {
    bool retval=false;

    double tA=get_timestamp_ns();

    point2d p(lat,lon);
    int i=std::floor((lat-m_minlat)/m_lat_increment);
    int j=std::floor((lon-m_minlon)/m_lon_increment);
    std::tuple<int,int> index(i,j);

    int counter=0;
    for (auto building:m_buildings_by_sector.at(index)) {
        if (boost::geometry::covered_by(p,building.second)) {
            retval=true;
            break;
        }
    }

    double tB=get_timestamp_ns();
    sumBuilding+=tB-tA;
    countBuilding++;
    //std::cout <<"\nResult: " <<retval?"true":"false";
    //std::cout <<"\nTime for building calculations in us: " <<(tB-tA)/1000.0;
    //std::cout <<"\nCurrent average time for building calculations in us: " <<sumBuilding/countBuilding/1000.0 <<std::endl;

    return retval;
}

bool OSMStore::checkSpeedOverTypeLimit(double speed, osmium::object_id_type highwayID) {
    bool retval=false;
    if (speed>highways_info.at(highwayID).maxSpeed) {
        retval=true;
    }
    return retval;
}