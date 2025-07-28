#include <string>
#include <map>
#include <shared_mutex>

#include "StationID.h"
#include "CertificateBase.h"

typedef enum DigestValid_retval {
    DIGEST_OK,
    DIGEST_NOT_FOUND,
    DIGEST_EXPIRED,
} e_DigestValid_retval;

typedef struct storedCertificate {
    uint64_t stationID;
    uint64_t msg_timestamp; // received on_message_timestamp, comparable with get_timestamp_us() for age checks
    uint64_t start; // in seconds from unix epoch, directly comparable with get_timestamp_s() 
    uint64_t end; // in seconds from unix epoch, directly comparable with get_timestamp_s()
	IssuerIdentifierSec_t issuer;
    std::string digest; //redundant with the key
} storedCertificate_t;

class CertificateStore {
public:
    void insert_or_assign(std::string digest, storedCertificate_t certificateData);
    void deleteOlderThan(double time_milliseconds);
    e_DigestValid_retval isValid(std::string digest);

    void printAll();

private:
    std::map<std::string,storedCertificate_t> m_certificateStore; //map each digest with its data
    
    std::shared_mutex m_storeMutex;
};