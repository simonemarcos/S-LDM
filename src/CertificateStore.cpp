#include "CertificateStore.h"
#include "utils.h"
#include <iostream>

void CertificateStore::insert_or_assign(std::string digest, storedCertificate_t certificateData) {
    //check if digest already present and update it in case, in theory only the receiving timestamp will change (used for dbcleanup on old digests)
    std::lock_guard<std::shared_mutex> lk(m_storeMutex);
    std::map<std::string,storedCertificate_t>::iterator it=m_certificateStore.find(digest);
    if (it==m_certificateStore.end()) {
        m_certificateStore.emplace(digest,certificateData);
    } else {
        it->second=certificateData;
    }
}

e_DigestValid_retval CertificateStore::isValid(std::string digest) {
    std::shared_lock<std::shared_mutex> lk(m_storeMutex);
    if (m_certificateStore.find(digest)==m_certificateStore.end()) {
        return DIGEST_NOT_FOUND;
    }
    if (get_timestamp_s()>m_certificateStore.at(digest).end) {
        return DIGEST_EXPIRED;
    }
    return DIGEST_OK;
}

//std::vector<osmium::Way> ways;
//class TestHandler : public osmium::handler::Handler {
//public:
//    static void way(const osmium::Way& way) {
//        std::cout <<"Count: " <<ways.size() <<std::endl;
//        ways.emplace_back(way);
//    }
//};

void CertificateStore::test() {
    try {
        //const osmium::io::File input_file{"map.osm"};
        //osmium::io::Reader reader{input_file};

        //TestHandler handler;

        //osmium::apply(reader,handler);

        //reader.close();
    } catch(const std::exception &e) {
        std::cerr <<e.what() <<std::endl;
    }
}
