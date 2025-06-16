#include <string>
#include <map>

class DigestStore {
public:
    void insert_or_assign(std::string digest,uint64_t expiryTimestamp);
    bool isValid(std::string);
    void printAll();

private:
    std::map<std::string,uint64_t> digestStore; //map each digest with unix epoch expiry in seconds
    
};