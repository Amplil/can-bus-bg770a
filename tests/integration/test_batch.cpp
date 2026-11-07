#include "telemetry-batch.h"
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unordered_map>
#include <algorithm>

template<class T>static std::string json(const T& value) {
  std::string output;serializeJson(value,output);return output;
}
static void addRecord(JsonDocument& source,unsigned sequence) {
  JsonObject record=source["data"].as<JsonArray>().add<JsonObject>();
  record["time"]="2026-11-06T10:44:59+00:00";
  record["engine_rpm"]=1000+sequence;
  record["vehicle_speed"]=62;
  record["coolant_temp"]=33;
  record["engine_load"]=41;
  record["intake_air_temp"]=24;
  record["throttle_position"]=35;
  record["distance_traveled"]=42;
  record["odometer"]=12345;
  record["control_module_voltage"]=12.345;
  record["ambient_air_temp"]=-2;
  record["dtc_codes"]="P01AF";
  record["dtc_count"]=1;
  record["time_source"]="ntp";
  record["time_sync_state"]="synced";
  record["utc_epoch_s"]=1793961899LL;
}
class RejectAllocator:public ArduinoJson::Allocator {
 public:
  void* allocate(size_t)override{return nullptr;}
  void deallocate(void*)override{}
  void* reallocate(void*,size_t)override{return nullptr;}
};
class TrackingAllocator:public ArduinoJson::Allocator {
 std::unordered_map<void*,size_t> sizes;
 public:
 size_t current=0,peak=0;
 void* allocate(size_t size)override {
  auto* p=std::malloc(size);if(p){sizes[p]=size;current+=size;peak=std::max(peak,current);}return p;
 }
 void deallocate(void* p)override {if(p){current-=sizes.at(p);sizes.erase(p);std::free(p);}}
 void* reallocate(void* p,size_t size)override {
  if(!p)return allocate(size);
  const auto old=sizes.at(p);auto* q=std::realloc(p,size);
  if(q){sizes.erase(p);sizes[q]=size;current=current-old+size;peak=std::max(peak,current);}return q;
 }
};
int main() {
  JsonDocument source,batch;
  source["imsi"]="synthetic-device";
  source["data"].to<JsonArray>();
  source["time_source"]="ntp";
  source["obd"]["can_ready"]=true;
  source["location"]["source"]="synthetic-location";
  source["location"]["latitude"]=35.0;
  for(unsigned sequence=0;sequence<12;++sequence)addRecord(source,sequence);
  assert(measureJson(source)>1460);

  // Rejected sends do not acknowledge or remove any queue entries.
  const std::string before=json(source);
  const size_t first=bg770a::makeTelemetryBatch(source,batch,1024);
  assert(first>0 && first<12 && measureJson(batch)<=1024);
  assert(json(source)==before && source["data"].size()==12);
  const std::string rejected=json(batch);
  assert(bg770a::makeTelemetryBatch(source,batch,1024)==first);
  assert(json(batch)==rejected && json(source)==before);

  unsigned delivered=0,batches=0;
  while(source["data"].size()) {
    const std::string queued=json(source);
    const size_t count=bg770a::makeTelemetryBatch(source,batch,1024);
    assert(count>0 && measureJson(batch)<=1024);
    assert(json(source)==queued);
    assert(batch["imsi"]==source["imsi"]);
    assert(json(batch["obd"])==json(source["obd"]));
    assert(json(batch["location"])==json(source["location"]));
    assert(batch["time_source"]==source["time_source"]);
    for(size_t index=0;index<count;++index) {
      assert(json(batch["data"][index])==json(source["data"][index]));
      assert(batch["data"][index]["engine_rpm"]==1000+delivered+index);
    }
    // Simulate acceptance: remove exactly this prefix, preserving the tail.
    for(size_t index=0;index<count;++index)source["data"].as<JsonArray>().remove(0);
    delivered+=unsigned(count);++batches;
  }
  assert(delivered==12 && batches>1);

  // Byte limits include UTF-8 bytes and JSON syntax, and permit exact equality.
  source.clear();source["imsi"]="synthetic-device";
  source["label"]="日本語";source["data"].to<JsonArray>();addRecord(source,0);
  const std::string single=json(source);
  const size_t exact=measureJson(source);
  assert(exact==single.size());
  assert(bg770a::makeTelemetryBatch(source,batch,exact)==1);
  assert(json(batch)==single && json(source)==single);
  assert(bg770a::makeTelemetryBatch(source,batch,exact-1)==0);
  assert(batch.isNull() && json(source)==single);
  assert(bg770a::makeTelemetryBatch(source,batch,0)==0 && json(source)==single);
  assert(bg770a::makeTelemetryBatch(source,source,exact)==0 && json(source)==single);

  RejectAllocator rejecting;
  JsonDocument unavailable(&rejecting);
  assert(bg770a::makeTelemetryBatch(source,unavailable,1024)==0);
  assert(json(source)==single);
  JsonDocument overflowed(&rejecting);
  overflowed["data"].to<JsonArray>();assert(overflowed.overflowed());
  assert(bg770a::makeTelemetryBatch(overflowed,batch,1024)==0);

  for(const char* invalid:{"null","[]","{}","{\"data\":[]}",
                          "{\"data\":{}}","{\"data\":[null]}","{\"data\":[1]}"}) {
    source.clear();assert(!deserializeJson(source,invalid));
    const std::string unchanged=json(source);
    assert(bg770a::makeTelemetryBatch(source,batch,1024)==0);
    assert(json(source)==unchanged && batch.isNull());
  }
  TrackingAllocator queueMemory,batchMemory;
  JsonDocument sparse(&queueMemory),bounded(&batchMemory);
  sparse["imsi"]="synthetic-device";
  auto sparseRows=sparse["data"].to<JsonArray>();
  for(unsigned n=0;n<120;++n) {
    auto row=sparseRows.add<JsonObject>();
    row["time"]=std::string("2026-11-06T10:44:59.")+std::to_string(100+n)+"+00:00";
    row["dtc_codes"]="P01AF P02AF P03AF";row["dtc_count"]=3;
  }
  assert(!sparse.overflowed());
  assert(bg770a::makeTelemetryBatch(sparse,bounded,1024)>0 && measureJson(bounded)<=1024);
  assert(queueMemory.peak<40000 && batchMemory.peak<16000);
  std::cout<<"120 sparse DTC rows host allocator peak="<<queueMemory.peak
           <<" bytes; prefix builder peak="<<batchMemory.peak<<" bytes\n";
  std::cout<<"Telemetry batches preserve prefix records and metadata, respect serialized byte limits, "
              "retain rejected queues, and reject oversize/invalid/allocation failures\n";
}
