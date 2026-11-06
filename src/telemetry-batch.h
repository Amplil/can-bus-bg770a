#pragma once
#include <ArduinoJson.h>
#include <stddef.h>

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
  if(!batch.set(source) || batch.overflowed()) {batch.clear();return 0;}
  JsonArray prefix=batch["data"].as<JsonArray>();
  while(prefix.size()>1 && measureJson(batch)>maxBytes)prefix.remove(prefix.size()-1);
  if(measureJson(batch)>maxBytes || batch.overflowed()) {batch.clear();return 0;}
  return prefix.size();
}
}
