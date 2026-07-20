#include "frameBuffer.h"
#include <unistd.h>
#include <GeographicLib/Constants.hpp>
#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>


FrameBuffer::FrameBuffer(int fd, uint16_t maxsz, double lat0, double lon0, double netoffset_x, double netoffset_y, double k0): _fd(fd), _maxsz(maxsz), _idx(0), _lat0(lat0), _lon0(lon0), _netoffset_x(netoffset_x), _netoffset_y(netoffset_y), _tm_converter_ptr(nullptr), _data(nullptr) {
    const double a = GeographicLib::Constants::WGS84_a();
    const double f = GeographicLib::Constants::WGS84_f();
    _tm_converter_ptr = new GeographicLib::TransverseMercator(a,f,k0);

    _tm_converter_ptr->Forward(_lon0, _lat0, _lon0, _x0, _y0);

    _data = new vehicleSnapshot_t[_maxsz];
    _data_us_timestamps = new uint64_t[_maxsz];
}

FrameBuffer::~FrameBuffer() {
    delete[] _data;
    _data = nullptr;
    delete [] _data_us_timestamps;
    _data_us_timestamps = nullptr;
    delete _tm_converter_ptr;
    _tm_converter_ptr = nullptr;
}

void FrameBuffer::setMaxSize(uint16_t maxsz) {
    delete[] _data;
    delete[] _data_us_timestamps;
    _maxsz = maxsz;
    _data = new vehicleSnapshot_t[_maxsz];
    _data_us_timestamps = new uint64_t[_maxsz];
    _idx=0;
}

uint16_t FrameBuffer::getMaxSize() {
    return _maxsz;
}

bool FrameBuffer::addCustom(vehicleSnapshot_t* vs, uint64_t timestamp_us) {
    if(_idx>=_maxsz)
        return false;

    _data[_idx] = *vs;
    _data_us_timestamps[_idx] = timestamp_us;
    _idx++;

    return true;
}

uint16_t FrameBuffer::findVehicleIndexByID(uint64_t stationID) {
    for (uint16_t i = 0; i < _idx; ++i) {
        if (_data[i].stationID == stationID) {
            return i;
        }
    }
    return _idx; // Not found, return an index equal to the current size (out of bounds)
}

bool FrameBuffer::add(ldmmap::vehicleData_t *vd) {
    if(_idx>=_maxsz)
        return false;

    // if found, replace at current, otherwise add at the end and increment index
    uint16_t insertIdx = findVehicleIndexByID(vd->stationID);

    if(insertIdx<_idx && _data_us_timestamps[insertIdx]>vd->on_msg_timestamp_us) {
        // if the vehicle is already present but the new data is older than the one already in the frame, skip the update
        return true;
    }


    double x,y;
    _tm_converter_ptr->Forward(_lon0, vd->lat, vd->lon, x, y);
    x = (x - _x0) + _netoffset_x;
    y = (y - _y0) + _netoffset_y;
    FrameBuffer::vehicleSnapshot_t frame = FrameBuffer::vehicleSnapshot_t {
        vd->stationID,
        (vd->vehicleWidth.isAvailable() ? static_cast<float>(vd->vehicleWidth.getData()) : 0.0f)/1000.0f, // convert from mm to m
        (vd->vehicleLength.isAvailable() ? static_cast<float>(vd->vehicleLength.getData()) : 0.0f)/1000.0f, // convert from mm to m
        vd->stationType,
        x,
        y,
        vd->speed_ms,
        vd->heading
    };

    _data[insertIdx] = frame;
    _data_us_timestamps[insertIdx] = vd->on_msg_timestamp_us;
    // if new, update index
    if(insertIdx==_idx)
        _idx++;

    return true;
}

void FrameBuffer::flushToFd(serialization_t serType) {
    switch(serType){
        case binary:
            write(_fd, &_idx, sizeof(uint16_t)); // First write the number of entries
            write(_fd, _data, sizeof(FrameBuffer::vehicleSnapshot_t)*_idx);
            break;
        case json:
            std::ostringstream ss;
            ss << "[";
            for (size_t i = 0; i < _idx; ++i) {
                const auto& v = _data[i];

                ss << "{"
                << "\"VehicleId\":" << v.stationID << ","
                << "\"Width\":" << std::fixed << std::setprecision(3) << v.width << ","
                << "\"Length\":" << std::fixed << std::setprecision(3) << v.length << ","
                << "\"StationType\":" << static_cast<int>(v.stationType) << ","
                << "\"X\":" << std::fixed << std::setprecision(6) << v.x << ","
                << "\"Y\":" << std::fixed << std::setprecision(6) << v.y << ","
                << "\"Speed\":" << std::fixed << std::setprecision(6) << v.speed << ","
                << "\"Angle\":" << std::fixed << std::setprecision(6) << v.heading
                << "}";

                if (i != _idx - 1)
                    ss << ",";
            }
            ss << "]\n";
            std::string serialized = ss.str();
            write(_fd, serialized.c_str(), serialized.size());
    }
    _idx=0;
}

bool FrameBuffer::empty() {
    return (_idx==0);
}

