#ifndef OBD_LOGIC_H
#define OBD_LOGIC_H

#include <stddef.h>
#include <stdint.h>

struct ObdSample {
  bool valid;
  float value;
  const char* unit;
};

bool obdExtractPidBytes(const uint8_t* data, size_t len, uint8_t pid, uint8_t* a, uint8_t* b, uint8_t* c);
ObdSample obdParseEngineRpm(const uint8_t* data, size_t len);
ObdSample obdParseVehicleSpeed(const uint8_t* data, size_t len);
ObdSample obdParseCoolantTemp(const uint8_t* data, size_t len);
ObdSample obdParseEngineLoad(const uint8_t* data, size_t len);
ObdSample obdParseIntakeAirTemp(const uint8_t* data, size_t len);
ObdSample obdParseThrottlePosition(const uint8_t* data, size_t len);
ObdSample obdParseDistanceTraveled(const uint8_t* data, size_t len);
ObdSample obdParseControlModuleVoltage(const uint8_t* data, size_t len);
ObdSample obdParseAmbientAirTemp(const uint8_t* data, size_t len);

#endif // OBD_LOGIC_H
