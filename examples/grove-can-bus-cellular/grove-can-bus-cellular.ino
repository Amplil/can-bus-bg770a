/*
 * grove-can-bus.ino
 * Grove - CAN BUS Module based on GD32E103 for Wio BG770A
 * Copyright (C) Seeed K.K.
 * MIT License
 */

#include <Adafruit_TinyUSB.h>
#include <csignal>
#include <WioCellular.h>
#include <grove-can-bus.h>
#include <ArduinoJson.h>
#include <obd-session.h>

// Set only from verified vehicle specifications; never probe live bus rates.
#ifndef OBD_CAN_RATE
#define OBD_CAN_RATE CAN_RATE_500
#endif
#ifndef OBD_CAN_EXTENDED
#define OBD_CAN_EXTENDED 0
#endif
static_assert(OBD_CAN_RATE == CAN_RATE_250 || OBD_CAN_RATE == CAN_RATE_500, "OBD CAN rate must be 250 or 500 kbps");
static_assert(OBD_CAN_EXTENDED == 0 || OBD_CAN_EXTENDED == 1, "OBD ID width must be 0 or 1");
static bg770a::ObdSession Obd(OBD_CAN_EXTENDED);
void addCANMessageToJSON(unsigned long, unsigned char*, unsigned long);
void updateVehicleData(unsigned char, unsigned char*, unsigned long);
void cleanupOldMessages();
static bool cellularSend(const JsonDocument&);
template<typename T> void printData(T&, const void*, size_t);

#define SEARCH_ACCESS_TECHNOLOGY (WioCellularNetwork::SearchAccessTechnology::LTEM)  // https://seeedjp.github.io/Wiki/Wio_BG770A/kb/kb4.html
#define LTEM_BAND (WioCellularNetwork::NTTDOCOMO_LTEM_BAND)        
                  // https://seeedjp.github.io/Wiki/Wio_BG770A/kb/kb4.html
static const char APN[] = "soracom.io";
static const char HOST[] = "uni.soracom.io";
static constexpr int PORT = 23080;
static constexpr int POWER_ON_TIMEOUT = 1000 * 20;     // [ms]
static constexpr int NETWORK_TIMEOUT = 1000 * 60 * 3;  // [ms]
static constexpr int RECEIVE_TIMEOUT = 1000 * 10;      // [ms]
static JsonDocument JsonDoc;

WioCAN can;
static bool CanReady = false;

static void abortHandler(int sig) {
  Serial.printf("ABORT: Signal %d received\n", sig);
  yield();

  vTaskSuspendAll();  // FreeRTOS
  while (true) {
    ledOn(LED_BUILTIN);
    nrfx_coredep_delay_us(100000);  // Spin
    ledOff(LED_BUILTIN);
    nrfx_coredep_delay_us(100000);  // Spin
  }
}

void setup() {
  signal(SIGABRT, abortHandler);
  Serial.begin(115200);
  {
    const auto start = millis();
    while (!Serial && millis() - start < 5000) {
      delay(2);
    }
  }
  Serial.println();
  Serial.println();
  Serial.println("Grove CAN Bus Module for Wio BG770A");
  Serial.println("=====================================");

  digitalWrite(LED_BUILTIN, HIGH);

  // Network configuration
  WioNetwork.config.searchAccessTechnology = SEARCH_ACCESS_TECHNOLOGY;
  WioNetwork.config.ltemBand = LTEM_BAND;
  WioNetwork.config.apn = APN;

  WioCellular.begin();
  // Power on the cellular module
  if (WioCellular.powerOn(POWER_ON_TIMEOUT) != WioCellularResult::Ok) abort();
  WioNetwork.begin();

  // Wait for communication available
  if (!WioNetwork.waitUntilCommunicationAvailable(NETWORK_TIMEOUT)) abort();
  Serial.println("initialized");
  {
    const auto start = millis();
    while (!Serial && millis() - start < 5000) {
      delay(2);
    }
  }

  digitalWrite(PIN_VGROVE_ENABLE, VGROVE_ENABLE_ON);
  delay(1000);
  // Initialize CAN module
  can.begin();
  // Set CAN bus rate to 500kbps (commonly used)
  if((CanReady = can.setCanRate(OBD_CAN_RATE))) {
    Serial.println("Configured CAN bus rate: OK");
  } else {
    Serial.println("Configured CAN bus rate: FAILED");
  }

  // --- set mask and filter (standard frame) ---
  // Mask設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setMask(0, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Mask0 set: OK (All pass)");
  } else {
    Serial.println("Mask0 set: FAILED");
  }
  if (can.setMask(1, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Mask1 set: OK (All pass)");
  } else {
    Serial.println("Mask1 set: FAILED");
  }
  
  // Filter設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setFilt(0, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt0 set: OK (All pass)");
  } else {
    Serial.println("Filt0 set: FAILED");
  }
  if (can.setFilt(1, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt1 set: OK (All pass)");
  } else {
    Serial.println("Filt1 set: FAILED");
  }
  if (can.setFilt(2, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt2 set: OK (All pass)");
  } else {
    Serial.println("Filt2 set: FAILED");
  }
  if (can.setFilt(3, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt3 set: OK (All pass)");
  } else {
    Serial.println("Filt3 set: FAILED");
  }
  if (can.setFilt(4, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt4 set: OK (All pass)");
  } else {
    Serial.println("Filt4 set: FAILED");
  }
  if (can.setFilt(5, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt5 set: OK (All pass)");
  } else {
    Serial.println("Filt5 set: FAILED");
  }
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize JSON document structure
  JsonDoc["session_start"] = millis();
  JsonDoc["device_id"] = "WIO_BG770A_001";
  JsonDoc["can_messages"].to<JsonArray>();
  JsonDoc["vehicle_data"].to<JsonObject>();
  
  Serial.println();
  Serial.println("=== CAN Bus Initialization Complete ===");
  Serial.println("Listening for ALL CAN messages...");
  Serial.println("Format: [timestamp] CAN ID: 0xXXX | Data: XX XX XX XX XX XX XX XX | Description");
  Serial.println("========================================");
}

void loop() {
  static unsigned long lastCellularSend = 0;
  static const unsigned long cellularSendInterval = 10000;
  const uint32_t now = millis();
  Obd.tick(now);
  unsigned long id = 0;
  unsigned char data[8] = {};
  // Bound receive work per loop; unrelated traffic never completes a request.
  for (unsigned n=0; n<8 && can.receive(&id, data); ++n) {
    addCANMessageToJSON(id, data, millis());
    if (Obd.accept(id, data, sizeof(data), millis()))
      updateVehicleData(data[2], data, millis());
  }
  unsigned char pid;
  if (CanReady && Obd.next(millis(), pid)) {
    unsigned char request[8] = {2, 1, pid, 0, 0, 0, 0, 0};
    can.send(Obd.requestId(), OBD_CAN_EXTENDED, 0, 8, request);
  }
  JsonObject diagnostics = JsonDoc["obd"].to<JsonObject>();
  diagnostics["coverage"] = "vehicle_unverified";
  if (Obd.selected()) diagnostics["response_id"] = Obd.ecu();
  const unsigned char pids[] = {0x0C,0x0D,0x05,0x04,0x0F,0x11,0x31,0x42,0x46};
  for (auto p : pids) {
    const auto state = Obd.support(p);
    diagnostics["supported_pids"][String(p, HEX)] =
      state == bg770a::Support::Supported ? "supported" :
      state == bg770a::Support::Unsupported ? "unsupported" : "unknown";
  }

  if(millis() - lastCellularSend > cellularSendInterval) {
    if (cellularSend(JsonDoc)) cleanupOldMessages();
    // 送信後、古いメッセージデータをクリア（最新のvehicle_dataは保持）
    lastCellularSend = millis();
  }
  
  // Optional: Enable debug mode by sending commands through Serial monitor
  // Uncomment the following line to enable debug mode
  //can.debugMode();
  // The normal receiver owns UART bytes; debugPID() would drain replies.

  delay(10);
}

// CANメッセージをタイムスタンプ付きでJSONに追加
void addCANMessageToJSON(unsigned long id, unsigned char* data, unsigned long timestamp) {
  JsonArray messages = JsonDoc["can_messages"];
  
  // 新しいメッセージオブジェクトを作成
  JsonObject message = messages.add<JsonObject>();
  message["timestamp"] = timestamp;
  message["can_id"] = "0x" + String(id, HEX);
  
  // データ配列を追加
  JsonArray dataArray = message["data"].to<JsonArray>();
  for(int i = 0; i < 8; i++) {
    dataArray.add(data[i]);
  }
  
  // No Suzuki DBC or frame metadata is available: never infer ECU role.
  message["type"] = "Unknown";

  // メッセージ数が多すぎる場合は古いものを削除（メモリ管理）
  if(messages.size() > 50) {
    messages.remove(0);  // 最古のメッセージを削除
  }
}

// 車両データを時系列で更新
void updateVehicleData(unsigned char pid, unsigned char* data, unsigned long timestamp) {
  JsonObject vehicleData = JsonDoc["vehicle_data"].as<JsonObject>();
  
  // raw_data オブジェクトが存在しない場合は作成
  if (!vehicleData["raw_data"].is<JsonObject>()) {
    vehicleData["raw_data"].to<JsonObject>();
  }
  
  JsonArray rawDataArray = vehicleData["raw_data"]["data"].to<JsonArray>();
  // 既存のデータをクリア
  rawDataArray.clear();
  for(int i = 0; i < 8; i++) {
    rawDataArray.add(data[i]);
  }
  vehicleData["raw_data"]["pid"] = pid;
  vehicleData["raw_data"]["timestamp"] = timestamp;

  switch(pid) {
    case PID_ENGIN_PRM: // Engine RPM
      if(data[0] >= 4) {
        unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
        JsonObject rpmObj = vehicleData["engine_rpm"].to<JsonObject>();
        rpmObj["value"] = rpm;
        rpmObj["unit"] = "rpm";
        rpmObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_VEHICLE_SPEED: // Vehicle Speed
      {
        JsonObject speedObj = vehicleData["vehicle_speed"].to<JsonObject>();
        speedObj["value"] = data[3];
        speedObj["unit"] = "km/h";
        speedObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_COOLANT_TEMP: // Coolant Temperature
      {
        JsonObject coolantObj = vehicleData["coolant_temp"].to<JsonObject>();
        coolantObj["value"] = data[3] - 40;
        coolantObj["unit"] = "°C";
        coolantObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_ENGINE_LOAD: // Engine Load
      {
        JsonObject loadObj = vehicleData["engine_load"].to<JsonObject>();
        loadObj["value"] = data[3] * 100.0 / 255.0;
        loadObj["unit"] = "%";
        loadObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_INTAKE_AIR_TEMP: // Intake Air Temperature
      {
        JsonObject intakeObj = vehicleData["intake_air_temp"].to<JsonObject>();
        intakeObj["value"] = data[3] - 40;
        intakeObj["unit"] = "°C";
        intakeObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_THROTTLE_POS: // Throttle Position
      {
        JsonObject throttleObj = vehicleData["throttle_position"].to<JsonObject>();
        throttleObj["value"] = data[3] * 100.0 / 255.0;
        throttleObj["unit"] = "%";
        throttleObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_DISTANCE_TRAVELED: // Distance Since DTC Clear
      if(data[0] >= 4) {
        unsigned int distance = (data[3] * 256) + data[4];
        JsonObject distanceObj = vehicleData["distance_traveled"].to<JsonObject>();
        distanceObj["value"] = distance;
        distanceObj["unit"] = "km";
        distanceObj["meaning"] = "distance_since_dtc_clear";
        distanceObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_CONTROL_MODULE_VOLTAGE: // Control Module Voltage
      if(data[0] >= 4) {
        float voltage = ((data[3] * 256) + data[4]) / 1000.0;
        JsonObject voltageObj = vehicleData["control_module_voltage"].to<JsonObject>();
        voltageObj["value"] = voltage;
        voltageObj["unit"] = "V";
        voltageObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_AMBIENT_AIR_TEMP: // Ambient Air Temperature
      {
        JsonObject ambientObj = vehicleData["ambient_air_temp"].to<JsonObject>();
        ambientObj["value"] = data[3] - 40;
        ambientObj["unit"] = "°C";  
        ambientObj["timestamp"] = timestamp;
      }
      break;
  }
}

// 古いメッセージをクリーンアップ（メモリ管理）
void cleanupOldMessages() {
  JsonArray messages = JsonDoc["can_messages"];
  
  // メッセージ配列をクリア（送信後なので）
  while(messages.size() > 0) {
    messages.remove(0);
  }
  
  Serial.println("Cleaned up old CAN messages after transmission");
}

static bool cellularSend(const JsonDocument &doc) {
  Serial.println("### Sending Time-Series JSON Data");
  Serial.print("Messages in queue: ");
  Serial.println(doc["can_messages"].size());
  Serial.print("Vehicle data entries: ");
  Serial.println(doc["vehicle_data"].size());

  Serial.print("Connecting ");
  Serial.print(HOST);
  Serial.print(":");
  Serial.println(PORT);

  {
    WioCellularTcpClient2<WioCellularModule> client{ WioCellular };
    if (!client.open(WioNetwork.config.pdpContextId, HOST, PORT)) {
      Serial.printf("ERROR: Failed to open %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    if (!client.waitForConnect()) {
      Serial.printf("ERROR: Failed to connect %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    Serial.print("Sending ");
    std::string str;
    serializeJson(doc, str);
    printData(Serial, str.data(), str.size());
    Serial.println();
    if (!client.send(str.data(), str.size())) {
      Serial.printf("ERROR: Failed to send socket %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    Serial.println("Receiving");
    static uint8_t recvData[WioCellular.RECEIVE_SOCKET_SIZE_MAX];
    size_t recvSize;
    if (!client.receive(recvData, sizeof(recvData), &recvSize, RECEIVE_TIMEOUT)) {
      Serial.printf("ERROR: Failed to receive socket %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    printData(Serial, recvData, recvSize);
    Serial.println();
  }

  Serial.println("### Completed");

  return true;
}

template<typename T>
void printData(T &stream, const void *data, size_t size) {
  auto p = static_cast<const char *>(data);

  for (; size > 0; --size, ++p)
    stream.write(0x20 <= *p && *p <= 0x7f ? *p : '.');
}