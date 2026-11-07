#pragma once
#include <ArduinoJson.h>
#include <stddef.h>
#include <cstring>

namespace bg770a {
// Keep a complete JSON envelope within an application byte budget. A smaller
// payload avoids the modem client's multi-write path; TCP can still segment it.
// Only an acknowledged prefix may later be removed from the source queue.
inline size_t makeTelemetryBatch(const JsonDocument& source, JsonDocument& batch,
                                size_t maxBytes) {
  // Reject aliasing rather than modifying a caller's live queue.
  if(&source==&batch)return 0;
  batch.clear();
  const JsonArrayConst records=source["data"].as<JsonArrayConst>();
  if(source.overflowed() || !source.is<JsonObjectConst>() ||
     records.isNull() || records.size()==0 || maxBytes==0)return 0;
  for(JsonVariantConst record:records)if(!record.is<JsonObjectConst>())return 0;
  // Copy only envelope metadata and the fitting prefix, not the entire growing
  // sparse backlog. Temporary batch memory is bounded by one send plus one row.
  for(JsonPairConst field:source.as<JsonObjectConst>()) {
    if(std::strcmp(field.key().c_str(),"data")!=0)batch[field.key()]=field.value();
  }
  JsonArray prefix=batch["data"].to<JsonArray>();
  for(JsonVariantConst record:records) {
    if(!prefix.add(record) || batch.overflowed()) {batch.clear();return 0;}
    if(measureJson(batch)>maxBytes) {prefix.remove(prefix.size()-1);break;}
  }
  if(prefix.size()==0 || batch.overflowed()) {batch.clear();return 0;}
  return prefix.size();
}
}
