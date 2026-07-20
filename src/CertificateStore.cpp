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

void CertificateStore::deleteOlderThan(double time_milliseconds) {
    uint64_t now = get_timestamp_us();
    int counter=0;
    std::lock_guard<std::shared_mutex> lk(m_storeMutex);
    for (auto it=m_certificateStore.cbegin();it!=m_certificateStore.cend();) {
        if(((double)(now-it->second.msg_timestamp))/1000.0 > time_milliseconds) {
            it=m_certificateStore.erase(it);
            counter++;
        } else {
            ++it;
        }
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

void CertificateStore::printAll() {
    std::cout <<"\n\nFull digest Store:\n";
    for (auto p:m_certificateStore) {
        std::cout <<"Digest: " <<p.first <<" Timestamp: " <<p.second.msg_timestamp <<std::endl;
    }
}