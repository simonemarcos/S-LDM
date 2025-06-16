#include "DigestStore.h"
#include "utils.h"
#include <iostream>

void DigestStore::insert_or_assign(std::string digest, uint64_t expiryTimestamp) {
    digestStore.insert_or_assign(digest,expiryTimestamp);
}

bool DigestStore::isValid(std::string digest) {
    // da rimuovere serve per i digest errati al momento
    if (digest=="113f5daecdb17a98") {digest="06e7293da060a831";}
    std::cout <<"\tChecking digest:" <<digest <<std::endl;
    if (digestStore.find(digest)==digestStore.end()) {
        std::cout <<"\tNot found" <<std::endl;
        return false;
    }
    std::cout <<"\tTimestamp now:" <<get_timestamp_s() <<"\tTimestamp expiry:" <<digestStore.at(digest) <<std::endl;
    if (get_timestamp_s()>digestStore.at(digest)) {
        std::cout <<"\tExpired." <<std::endl; 
        return false;
    }
    std::cout <<"\tValid" <<std::endl;
    return true;
}

void DigestStore::printAll() {
    std::cout <<"Full digest store:" <<std::endl;
    for (auto i:digestStore) {
        std::cout <<i.first <<":" <<i.second <<std::endl;
    }
}
