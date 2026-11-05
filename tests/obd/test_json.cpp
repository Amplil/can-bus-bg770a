#include <ArduinoJson.h>
#include <cassert>
#include <string>
#include <sstream>
#include <iostream>
static constexpr int HEX=16;
static std::string String(unsigned long value, int base) {
 std::ostringstream out;if(base==16)out<<std::hex;out<<value;return out.str();
}
static JsonDocument JsonDoc;
#include "sketch-functions.h"
int main() {
 JsonDoc["can_messages"].to<JsonArray>();
 JsonDoc["vehicle_data"].to<JsonObject>();
 unsigned char rpm[]={4,0x41,0x0C,0x1F,0x40,0,0,0};
 updateVehicleData(0x0C,rpm,10);
 unsigned char speed[]={3,0x41,0x0D,45,0,0,0,0};
 updateVehicleData(0x0D,speed,20);
 assert(JsonDoc["vehicle_data"]["engine_rpm"]["value"]==2000);
 assert(JsonDoc["vehicle_data"]["engine_rpm"]["timestamp"]==10);
 assert(JsonDoc["vehicle_data"]["vehicle_speed"]["value"]==45);
 unsigned char distance[]={4,0x41,0x31,0x01,0x02,0,0,0};
 updateVehicleData(0x31,distance,30);
 assert(JsonDoc["vehicle_data"]["distance_traveled"]["value"]==258);
 assert(JsonDoc["vehicle_data"]["distance_traveled"]["meaning"]=="distance_since_dtc_clear");
 for(unsigned i=0;i<60;++i)addCANMessageToJSON(0x123,speed,i);
 assert(JsonDoc["can_messages"].size()==50);
 assert(JsonDoc["can_messages"][0]["timestamp"]==10);
 assert(JsonDoc["can_messages"][0]["type"]=="Unknown");
 assert(JsonDoc["can_messages"][0]["data"].size()==8);
 std::cout<<"Actual sketch JSON accumulation and queue regression tests passed\n";
}
