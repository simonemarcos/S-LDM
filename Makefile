EXECNAME=SLDM

SRC_DIR=src
OBJ_DIR=obj

SRC_VEHVIS_DIR=vehicle-visualizer/src
OBJ_VEHVIS_DIR=obj/vehicle-visualizer

SRC_OPTIONS_DIR=options
OBJ_OPTIONS_DIR=obj/options

SRC_DECODER_DIR=decoder-module/src
OBJ_OPTIONS_DIR=obj/decoder-module

SRC_ASN1_DIR=decoder-module/asn1/src
OBJ_ASN1_DIR=obj/asn1

SRC_JSON11_DIR=json11
OBJ_JSON11_DIR=obj/json11

SRC_GEOLIB_PORT_DIR=geographiclib-port
OBJ_GEOLIB_PORT_DIR=obj/geographiclib-port

SRC_ASN1CPP_DIR=asn1cpp
OBJ_ASN1CPP_DIR=obj/asn1cpp

SRC_INIHC_DIR=inih/c
OBJ_INIHC_DIR=obj/inih/c

SRC_INIHCPP_DIR=inih/cpp
OBJ_INIHCPP_DIR=obj/inih/cpp

SRC_OSMIUM_DIR=libosmium
OBJ_OSMIUM_DIR=obj/libosmium

SRC=$(wildcard $(SRC_DIR)/*.cpp)
SRC_VEHVIS=$(wildcard $(SRC_VEHVIS_DIR)/*.cc)
SRC_OPTIONS=$(wildcard $(SRC_OPTIONS_DIR)/*.c)
SRC_DECODER=$(wildcard $(SRC_DECODER_DIR)/*.cpp)
SRC_ASN1=$(wildcard $(SRC_ASN1_DIR)/*.c)
SRC_JSON11=$(wildcard $(SRC_JSON11_DIR)/*.cpp)
SRC_GEOLIB_PORT=$(wildcard $(SRC_GEOLIB_PORT_DIR)/*.c)
SRC_ASN1CPP=$(wildcard $(SRC_ASN1CPP_DIR)/*.cpp)
SRC_INIHC=$(wildcard $(SRC_INIHC_DIR)/*.c)
SRC_INIHCPP=$(wildcard $(SRC_INIHCPP_DIR)/*.cpp)
SRC_OSMIUM=$(wildcard $(SRC_OSMIUM_DIR)/*.cpp)

OBJ=$(SRC:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
OBJ_VEHVIS=$(SRC_VEHVIS:$(SRC_VEHVIS_DIR)/%.c=$(OBJ_VEHVIS_DIR)/%.o)
OBJ_OPTIONS=$(SRC_OPTIONS:$(SRC_OPTIONS_DIR)/%.c=$(OBJ_OPTIONS_DIR)/%.o)
OBJ_DECODER=$(SRC_DECODER:$(SRC_DECODER_DIR)/%.c=$(OBJ_DECODER_DIR)/%.o)
OBJ_ASN1=$(SRC_ASN1:$(SRC_ASN1_DIR)/%.c=$(OBJ_ASN1_DIR)/%.o)
OBJ_JSON11=$(SRC_JSON11:$(SRC_JSON11_DIR)/%.cpp=$(OBJ_JSON11_DIR)/%.o)
OBJ_GEOLIB_PORT=$(SRC_GEOLIB_PORT:$(SRC_GEOLIB_PORT_DIR)/%.c=$(OBJ_GEOLIB_PORT_DIR)/%.o)
OBJ_ASN1CPP=$(SRC_ASN1CPP:$(SRC_ASN1CPP_DIR)/%.cpp=$(OBJ_ASN1CPP_DIR)/%.o)
OBJ_INIHC=$(SRC_INIHC:$(SRC_INIHC_DIR)/%.c=$(OBJ_INIHC_DIR)/%.o)
OBJ_INIHCPP=$(SRC_INIHCPP:$(SRC_INIHCPP_DIR)/%.cpp=$(OBJ_INIHCPP_DIR)/%.o)
OBJ_OSMIUM=$(SRC_OSMIUM:$(SRC_OSMIUM_DIR)/%.cpp=$(OBJ_OSMIUM_DIR)/%.o)

OBJ_CC=$(OBJ)
OBJ_CC+=$(OBJ_VEHVIS)
OBJ_CC+=$(OBJ_OPTIONS)
OBJ_CC+=$(OBJ_DECODER)
OBJ_CC+=$(OBJ_ASN1)
OBJ_CC+=$(OBJ_JSON11)
OBJ_CC+=$(OBJ_GEOLIB_PORT)
OBJ_CC+=$(OBJ_ASN1CPP)
OBJ_CC+=$(OBJ_INIHC)
OBJ_CC+=$(OBJ_INIHCPP)
OBJ_CC+=$(OBJ_OSMIUM)

# aggiunto -Igeographiclib-port -Iasn1cpp -Iinih/c -Iinih/cpp -Ilibosmium
CXXFLAGS += -Wall -O3 -Iinclude -Ijson11 -Ivehicle-visualizer/include -Ioptions -std=c++17 -Idecoder-module/include -Idecoder-module/asn1/include -Igeographiclib-port -Iasn1cpp -Iinih/c -Iinih/cpp -Ilibosmium
CFLAGS += -Wall -O3 -Iinclude -Ioptions -Idecoder-module/asn1/include -Igeographiclib-port
LDLIBS += -lcpprest -lpthread -lcrypto -lm -lqpid-proton-cpp -lGeographic -lz -lexpat -lbz2 -lcurl

.PHONY: all clean

all: compilePC

compilePC: CXX = g++
compilePC: CC = gcc
	
compilePCdebug: CXXFLAGS += -g
compilePCdebug: compilePC

compilePC compilePCdebug: $(EXECNAME)

# Standard targets
$(EXECNAME): $(OBJ_CC)
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) $(CXXFLAGS) $(CFLAGS) -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@ mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_VEHVIS_DIR)/%.o: $(SRC_VEHVIS_DIR)/%.cc
	@ mkdir -p $(OBJ_VEHVIS_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_OPTIONS_DIR)/%.o: $(SRC_OPTIONS_DIR)/%.c
	@ mkdir -p $(OBJ_OPTIONS_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_DECODER_DIR)/%.o: $(SRC_DECODER_DIR)/%.cpp
	@ mkdir -p $(OBJ_DECODER_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_ASN1_DIR)/%.o: $(SRC_ASN1_DIR)/%.c
	@ mkdir -p $(OBJ_ASN1_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_JSON11_DIR)/%.o: $(SRC_JSON11_DIR)/%.cpp
	@ mkdir -p $(OBJ_JSON11_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_GEOLIB_PORT_DIR)/%.o: $(SRC_GEOLIB_PORT_DIR)/%.c
	@ mkdir -p $(OBJ_GEOLIB_PORT_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_INIHC_DIR)/%.o: $(SRC_INIHC_DIR)/%.c
	@ mkdir -p $(OBJ_INIHC_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJ_INIHCPP_DIR)/%.o: $(SRC_INIHCPP_DIR)/%.cpp
	@ mkdir -p $(OBJ_INIHCPP_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_OSMIUM_DIR)/%.o: $(SRC_OSMIUM_DIR)/%.cpp
	@ mkdir -p $(OBJ_OSMIUM_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJ_DIR)/*.o $(OBJ_VEHVIS_DIR)/*.o $(OBJ_OPTIONS_DIR)/*.o$(OBJ_DECODER_DIR)/*.o $(OBJ_ASN1_DIR)/*.o $(OBJ_JSON11_DIR)/*.o
	-rm -rf $(OBJ_DIR)
	-rm -rf $(OBJ_VEHVIS_DIR)
	-rm -rf $(OBJ_OPTIONS_DIR)
	-rm -rf $(OBJ_DECODER_DIR)
	-rm -rf $(OBJ_ASN1_DIR)
	-rm -rf $(OBJ_JSON11_DIR)
	-rm -rf $(OBJ_GEOLIB_PORT_DIR)
	-rm -rf $(OBJ_ASN1CPP_DIR)
	-rm -rf $(OBJ_INIHC_DIR)
	-rm -rf $(OBJ_INIHCPP_DIR)
	-rm -rf $(OBJ_OSMIUM_DIR)
	-rm -f cachefile.sldmc
	
fullclean: clean
	$(RM) $(EXECNAME)
