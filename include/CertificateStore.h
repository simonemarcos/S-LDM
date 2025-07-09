#include <string>
#include <map>
#include <shared_mutex>

#include "StationID.h"
#include "CertificateBase.h"

//#include "osmium/io/any_input.hpp"
//#include "osmium/visitor.hpp"
//#include "osmium/handler.hpp"
//#include "osmium/osm/way.hpp"
//#include "osmium/geom/haversine.hpp"

typedef enum DigestValid_retval {
    DIGEST_OK,
    DIGEST_NOT_FOUND,
    DIGEST_EXPIRED,
} e_DigestValid_retval;

typedef struct storedCertificate {
    StationID_t stationID;
    uint64_t msg_timestamp;
    uint64_t start;
    uint64_t end;
	IssuerIdentifierSec_t issuer;
    std::string digest; //redundant with the key
} storedCertificate_t;

class CertificateStore {
public:
    void insert_or_assign(std::string digest, storedCertificate_t certificateData);
    e_DigestValid_retval isValid(std::string digest);

    void test();

private:
    std::map<std::string,storedCertificate_t> m_certificateStore; //map each digest with its data
    
    std::shared_mutex m_storeMutex;
};