#include "obd-logic.h"

namespace {
ObdSample invalidSample(const char* unit) {
  return {false, 0.0f, unit};
}
} // namespace

bool obdExtractPidBytes(const uint8_t* data, size_t len, uint8_t pid, uint8_t* a, uint8_t* b, uint8_t* c) {
  if (len < 4 || !data || !a || !b || !c) {
    return false;
  }
  // Expected OBD-II response frame format:
  // [0] = number of additional bytes (e.g. 0x04)
  // [1] = 0x41 (response to mode 0x01)
  // [2] = PID
  // [3] = A, [4] = B, [5] = C ...
  if (data[1] != 0x41 || data[2] != pid) {
    return false;
  }
  *a = (len > 3) ? data[3] : 0;
  *b = (len > 4) ? data[4] : 0;
  *c = (len > 5) ? data[5] : 0;
  return true;
}

ObdSample obdParseEngineRpm(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x0C, &a, &b, &c)) {
    return invalidSample("rpm");
  }
  float rpm = ((static_cast<uint16_t>(a) << 8) | b) / 4.0f;
  return {true, rpm, "rpm"};
}

ObdSample obdParseVehicleSpeed(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x0D, &a, &b, &c)) {
    return invalidSample("km/h");
  }
  return {true, static_cast<float>(a), "km/h"};
}

ObdSample obdParseCoolantTemp(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x05, &a, &b, &c)) {
    return invalidSample("C");
  }
  return {true, static_cast<float>(a) - 40.0f, "C"};
}

ObdSample obdParseEngineLoad(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x04, &a, &b, &c)) {
    return invalidSample("%");
  }
  return {true, (static_cast<float>(a) * 100.0f) / 255.0f, "%"};
}

ObdSample obdParseIntakeAirTemp(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x0F, &a, &b, &c)) {
    return invalidSample("C");
  }
  return {true, static_cast<float>(a) - 40.0f, "C"};
}

ObdSample obdParseThrottlePosition(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x11, &a, &b, &c)) {
    return invalidSample("%");
  }
  return {true, (static_cast<float>(a) * 100.0f) / 255.0f, "%"};
}

ObdSample obdParseDistanceTraveled(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x31, &a, &b, &c)) {
    return invalidSample("km");
  }
  float km = static_cast<float>((static_cast<uint16_t>(a) << 8) | b);
  return {true, km, "km"};
}

ObdSample obdParseControlModuleVoltage(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x42, &a, &b, &c)) {
    return invalidSample("V");
  }
  float volts = ((static_cast<uint16_t>(a) << 8) | b) / 1000.0f;
  return {true, volts, "V"};
}

ObdSample obdParseAmbientAirTemp(const uint8_t* data, size_t len) {
  uint8_t a = 0;
  uint8_t b = 0;
  uint8_t c = 0;
  if (!obdExtractPidBytes(data, len, 0x46, &a, &b, &c)) {
    return invalidSample("C");
  }
  return {true, static_cast<float>(a) - 40.0f, "C"};
}
