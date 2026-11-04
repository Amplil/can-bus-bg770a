// PlatformIO only preprocesses .ino files at src_dir's root, not nested examples.
// Keep the Arduino IDE sketch as the single source of truth and compile it here.
#include <Arduino.h>
#include <ArduinoJson.h>

void addCANMessageToJSON(unsigned long id, unsigned char* data, unsigned long timestamp);
void updateVehicleData(unsigned char pid, unsigned char* data, unsigned long timestamp);
void cleanupOldMessages();
static bool cellularSend(const JsonDocument& doc);
template<typename T>
void printData(T& stream, const void* data, size_t size);

#include "../examples/grove-can-bus-cellular/grove-can-bus-cellular.ino"
