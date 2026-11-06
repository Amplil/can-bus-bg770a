#pragma once
#include <ArduinoJson.h>
#include "telemetry-time.h"
#include <cmath>
#include <cstring>

namespace bg770a {
inline void initializeObdFields(JsonObject record) {
  static const char* const fields[]={"engine_rpm","vehicle_speed","coolant_temp",
    "engine_load","intake_air_temp","throttle_position","distance_traveled",
    "odometer","control_module_voltage","ambient_air_temp","dtc_codes","dtc_count"};
  for(const char* field:fields)record[field]=nullptr;
}
// Unreceived OBD fields are null. Any finite acquired value, including zero,
// qualifies; a validated no-fault Mode 03 reply is also an acquired result.
// Rows containing only null OBD fields or non-OBD metadata must never be sent.
inline bool hasVehicleMeasurement(JsonObjectConst record) {
  static const char* const fields[]={"engine_rpm","vehicle_speed","coolant_temp",
    "engine_load","intake_air_temp","throttle_position","distance_traveled",
    "odometer","control_module_voltage","ambient_air_temp"};
  if(record.isNull())return false;
  for(const char* field:fields) {
    if(!record[field].is<double>())continue;
    const double value=record[field].as<double>();
    if(std::isfinite(value))return true;
  }
  if(record["dtc_count"].is<unsigned>())return true;
  const char* codes=record["dtc_codes"].as<const char*>();
  return codes && (!*codes || std::strspn(codes," \t\r\n")!=std::strlen(codes));
}
// A timestamped GPS fix can be uploaded before any OBD response is available.
inline bool hasTelemetryMeasurement(JsonObjectConst record) {
  if(hasVehicleMeasurement(record))return true;
  if(!record["latitude"].is<double>() || !record["longitude"].is<double>())return false;
  const double latitude=record["latitude"].as<double>(),longitude=record["longitude"].as<double>();
  const char* timestamp=record["gps_time"].as<const char*>();
  int64_t epoch;
  return std::isfinite(latitude) && latitude>=-90 && latitude<=90 &&
    std::isfinite(longitude) && longitude>=-180 && longitude<=180 &&
    timestamp && parseHttpTime(timestamp,epoch);
}

// An explicit heartbeat records liveness without inventing a measurement.
inline bool isHeartbeatRecord(JsonObjectConst record) {
  const char* type=record["type"].as<const char*>();
  const char* timestamp=record["time"].as<const char*>();
  int64_t epoch;
  if(!type || std::strcmp(type,"heartbeat")!=0 || !timestamp ||
     !parseHttpTime(timestamp,epoch))return false;
  for(JsonPairConst field:record) {
    if(std::strcmp(field.key().c_str(),"type")!=0 &&
       std::strcmp(field.key().c_str(),"time")!=0 && !field.value().isNull())return false;
  }
  return true;
}
inline bool hasTelemetryRecord(JsonObjectConst record) {
  if(record["type"].isUnbound())return hasTelemetryMeasurement(record);
  const char* type=record["type"].as<const char*>();
  if(type && std::strcmp(type,"telemetry")==0)return hasTelemetryMeasurement(record);
  return isHeartbeatRecord(record);
}
}
