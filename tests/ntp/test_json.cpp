#include <cassert>
#include <cstdint>
#include <iostream>
#include <string>
#include <ArduinoJson.h>
#include "ntp-clock.h"
static uint32_t fakeMillis=0;
static uint32_t millis() { return fakeMillis; }
static ntp_candidate::Uptime64 NtpUptime;
static ntp_candidate::NtpClock NtpClock;
static void addUtcFields(JsonObject object, unsigned long capturedMillis) {
  const uint32_t low = millis();
  const uint64_t now = NtpUptime.update(low);
  const uint64_t eventTime = now - static_cast<uint32_t>(low - capturedMillis);
  int64_t epoch;
  object["time_sync_state"] = NtpClock.quality(now);
  if (NtpClock.utc(eventTime, epoch)) {
    object["utc_epoch_s"] = epoch;
    object["time_source"] = "ntp";
  } else {
    object["utc_epoch_s"] = nullptr;
    object["time_source"] = "unsynced";
    object["time_sync_state"] = "unsynced";
  }
}

int main() {
 JsonDocument old; old["can_messages"]=JsonArray();
 JsonArray oldArray=old["can_messages"]; assert(oldArray.isNull());
 JsonDocument doc; doc["can_messages"].to<JsonArray>();
 JsonArray messages=doc["can_messages"];
 JsonObject early=messages.add<JsonObject>();early["timestamp"]=0;
 addUtcFields(early,0);assert(early["utc_epoch_s"].isNull());
 assert(early["time_sync_state"]=="unsynced");
 fakeMillis=100; NtpUptime.update(fakeMillis);
 assert(NtpClock.begin(100));NtpClock.accepted(100);
 NtpClock.urc("+QNTP: 0,\"2099/12/31,23:59:59+00\"",100);
 JsonObject current=messages.add<JsonObject>();current["timestamp"]=100;
 addUtcFields(current,100);
 assert(current["utc_epoch_s"].as<int64_t>()==4102444799LL);
 assert(current["timestamp"].as<uint32_t>()==100);
 assert(current["time_source"]=="ntp");
 assert(early["utc_epoch_s"].isNull());
 std::string encoded;serializeJson(doc,encoded);
 assert(encoded.find("4102444799")!=std::string::npos);
 assert(messages.size()==2);
 std::cout<<"Actual ArduinoJson timestamp integration passed; null-array regression reproduced\n";
}
