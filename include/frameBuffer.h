#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H
#include "vehicleDataDef.h"
#include <GeographicLib/TransverseMercator.hpp>

// Class that stores a map frame snapshot by accumulating multiple vehicle data entries, then allowing to serialize them as a block
class FrameBuffer {
    public:
        typedef enum {
            binary=0,
            json=1,
        } serialization_t;

        // basic block: vehicle snapshot
        typedef struct {
            // id
            uint64_t stationID;
            
            // vinfo
            float width;
            float length;
            ldmmap::e_StationTypeLDM stationType;

            // time info
            double x;
            double y;
            double speed;
            double heading;
        } vehicleSnapshot_t;

        FrameBuffer(int fd, uint16_t maxsz, double lat0, double lon0, double netoffset_x, double netoffset_y, double k0=0.9996);
        ~FrameBuffer();

        void setMaxSize(uint16_t maxsz);
        uint16_t getMaxSize();

        bool add(ldmmap::vehicleData_t *vd);
        bool addCustom(vehicleSnapshot_t* vs, uint64_t timestamp_us);
        void flushToFd(serialization_t serType);
        bool empty();
        uint16_t findVehicleIndexByID(uint64_t stationID);
    private:
        int _fd;
        uint16_t _maxsz;
        uint16_t _idx;
        double _x0;
        double _y0;
        double _netoffset_x;
        double _netoffset_y;
        double _lat0;
        double _lon0;
        GeographicLib::TransverseMercator *_tm_converter_ptr;
        vehicleSnapshot_t *_data;
        uint64_t *_data_us_timestamps;
};

#endif // FRAMEBUFFER_H