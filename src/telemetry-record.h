#pragma once
#include <ArduinoJson.h>
#include <cmath>
#include <cstring>

namespace bg770a {
// Required wire fields have zero defaults. A default-filled row (including an
// empty Mode 03 reply) carries no vehicle measurement and must never be sent.
// A stopped vehicle still qualifies through voltage, temperature, distance, etc.
inline bool hasVehicleMeasurement(JsonObjectConst record) {
  static const char* const fields[]={"engine_rpm","vehicle_speed","coolant_temp",
    "engine_load","intake_air_temp","throttle_position","distance_traveled",
    "odometer","control_module_voltage","ambient_air_temp"};
  if(record.isNull())return false;
  for(const char* field:fields) {
    const double value=record[field].as<double>();
    if(std::isfinite(value) && value!=0)return true;
  }
  const char* codes=record["dtc_codes"] | "";
  return record["dtc_count"].as<unsigned>()>0 || std::strspn(codes," \t\r\n")!=std::strlen(codes);
}
}
