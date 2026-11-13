#include "telemetry-record.h"
#include "telemetry-batch.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <string>

int main() {
  JsonDocument source,batch;
  source["imsi"]="synthetic-device";
  JsonObject record=source["data"].to<JsonArray>().add<JsonObject>();
  record["time"]="2026-11-06T10:44:59+00:00";
  const char* fields[]={"engine_rpm","vehicle_speed","coolant_temp","engine_load",
    "intake_air_temp","throttle_position","distance_traveled","odometer",
    "control_module_voltage","ambient_air_temp"};
  for(const char* field:fields)record[field]=nullptr;
  record["dtc_codes"]=nullptr;record["dtc_count"]=nullptr;
  assert(!bg770a::hasVehicleMeasurement(record));
  record["battery_level"]=99;record["latitude"]=35;
  assert(!bg770a::hasVehicleMeasurement(record));

  // Every field must retain the distinction between measured zero and null.
  for(const char* field:fields) {
    record[field]=0;
    assert(bg770a::hasVehicleMeasurement(record));
    assert(bg770a::makeTelemetryBatch(source,batch,1024)==1);
    std::string encoded;serializeJson(batch,encoded);
    JsonDocument decoded;assert(!deserializeJson(decoded,encoded));
    JsonObjectConst sent=decoded["data"][0].as<JsonObjectConst>();
    assert(sent[field].is<int>() && sent[field]==0);
    for(const char* other:fields)if(std::string(other)!=field)assert(sent[other].isNull());
    record[field]=nullptr;
    assert(!bg770a::hasVehicleMeasurement(record));
  }
  record["coolant_temp"]=-40;
  assert(bg770a::hasVehicleMeasurement(record));
  record["coolant_temp"]=nullptr;
  for(const char* invalid:{"0","garbage"}) {
    record["engine_rpm"]=invalid;
    assert(!bg770a::hasVehicleMeasurement(record));
  }
  record["engine_rpm"]=false;
  assert(!bg770a::hasVehicleMeasurement(record));
  for(double invalid:{std::numeric_limits<double>::quiet_NaN(),
                      std::numeric_limits<double>::infinity()}) {
    record["engine_rpm"]=invalid;
    assert(!bg770a::hasVehicleMeasurement(record));
  }
  record["engine_rpm"]=nullptr;
  record["dtc_count"]=0;
  assert(bg770a::hasVehicleMeasurement(record));
  record["dtc_count"]=nullptr;
  record["dtc_codes"]="";
  assert(bg770a::hasVehicleMeasurement(record));
  record["dtc_codes"]=" \t\r\n";
  assert(!bg770a::hasVehicleMeasurement(record));
  record["dtc_codes"]="MULTI";
  assert(bg770a::hasVehicleMeasurement(record) && record["dtc_count"].isNull());
  record["dtc_codes"]=nullptr;
  // An independently timestamped GPS fix may be sent without any OBD reply.
  record["latitude"]=0;record["longitude"]=0;
  record["gps_time"]="2026-11-06T10:44:59.000+00:00";
  assert(!bg770a::hasVehicleMeasurement(record));
  assert(bg770a::hasTelemetryMeasurement(record));
  assert(bg770a::makeTelemetryBatch(source,batch,1024)==1);
  std::string gpsEncoded;serializeJson(batch,gpsEncoded);
  JsonDocument gpsDecoded;assert(!deserializeJson(gpsDecoded,gpsEncoded));
  JsonObjectConst gpsSent=gpsDecoded["data"][0].as<JsonObjectConst>();
  assert(gpsSent["latitude"]==0 && gpsSent["longitude"]==0);
  for(const char* field:fields)assert(gpsSent[field].isNull());
  assert(gpsSent["dtc_count"].isNull() && gpsSent["dtc_codes"].isNull());
  record["gps_time"]=nullptr;
  assert(!bg770a::hasTelemetryMeasurement(record));
  record["gps_time"]="invalid";
  assert(!bg770a::hasTelemetryMeasurement(record));
  record["gps_time"]="2026-11-06T10:44:59.000+00:00";
  record["latitude"]=91;
  assert(!bg770a::hasTelemetryMeasurement(record));
  record["latitude"]=0;record["longitude"]=181;
  assert(!bg770a::hasTelemetryMeasurement(record));
  record["longitude"]=nullptr;
  assert(!bg770a::hasTelemetryMeasurement(record));

  // Heartbeats require an explicit marker and valid UTC time; they are never
  // treated as acquired measurements or padded with invented numeric values.
  source.clear();source["imsi"]="synthetic-device";
  record=source["data"].to<JsonArray>().add<JsonObject>();
  record["time"]="2026-11-06T10:44:59+00:00";
  assert(!bg770a::hasTelemetryRecord(record));
  record["type"]="heartbeat";
  assert(bg770a::isHeartbeatRecord(record));
  assert(bg770a::hasTelemetryRecord(record));
  assert(!bg770a::hasTelemetryMeasurement(record));
  assert(bg770a::makeTelemetryBatch(source,batch,1024)==1);
  std::string heartbeatEncoded;serializeJson(batch,heartbeatEncoded);
  JsonDocument heartbeatDecoded;assert(!deserializeJson(heartbeatDecoded,heartbeatEncoded));
  JsonObjectConst heartbeatSent=heartbeatDecoded["data"][0].as<JsonObjectConst>();
  assert(heartbeatSent.size()==2 && heartbeatSent["type"]=="heartbeat");
  assert(heartbeatSent["time"]=="2026-11-06T10:44:59+00:00");
  record["time"]="invalid";assert(!bg770a::isHeartbeatRecord(record));
  assert(!bg770a::hasTelemetryRecord(record));
  record.remove("time");assert(!bg770a::isHeartbeatRecord(record));
  record["time"]="2026-11-06T10:44:59+00:00";
  record["type"]="unknown";assert(!bg770a::hasTelemetryRecord(record));
  record["type"]="telemetry";assert(!bg770a::hasTelemetryRecord(record));
  record["engine_rpm"]=0;assert(bg770a::hasTelemetryRecord(record));
  record.remove("engine_rpm");
  record["type"]="heartbeat";record["engine_rpm"]=0;
  assert(!bg770a::isHeartbeatRecord(record));
  assert(!bg770a::hasTelemetryRecord(record));
  record.remove("engine_rpm");record["latitude"]=0;
  assert(!bg770a::isHeartbeatRecord(record));
  record.remove("latitude");record["dtc_count"]=0;
  assert(!bg770a::isHeartbeatRecord(record));
  std::cout<<"Measured zero stays distinct from null; explicit timestamp-only heartbeats batch without fake measurements\n";
}
