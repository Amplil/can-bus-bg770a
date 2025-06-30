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
  if(can.setCanRate(CAN_RATE_500)) {
    Serial.println("CAN bus rate set to 500kbps: OK");
  } else {
    Serial.println("CAN bus rate set to 500kbps: FAILED");
  }

  // --- set mask and filter (standard frame) ---
  // Mask設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setMask(0, 0, 0x00000000)) {
    Serial.println("Mask0 set: OK (All pass)");
  } else {
    Serial.println("Mask0 set: FAILED");
  }
  if (can.setMask(1, 0, 0x00000000)) {
    Serial.println("Mask1 set: OK (All pass)");
  } else {
    Serial.println("Mask1 set: FAILED");
  }
  
  // Filter設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setFilt(0, 0, 0x00000000)) {
    Serial.println("Filt0 set: OK (All pass)");
  } else {
    Serial.println("Filt0 set: FAILED");
  }
  if (can.setFilt(1, 0, 0x00000000)) {
    Serial.println("Filt1 set: OK (All pass)");
  } else {
    Serial.println("Filt1 set: FAILED");
  }
  if (can.setFilt(2, 0, 0x00000000)) {
    Serial.println("Filt2 set: OK (All pass)");
  } else {
    Serial.println("Filt2 set: FAILED");
  }
  if (can.setFilt(3, 0, 0x00000000)) {
    Serial.println("Filt3 set: OK (All pass)");
  } else {
    Serial.println("Filt3 set: FAILED");
  }
  if (can.setFilt(4, 0, 0x00000000)) {
    Serial.println("Filt4 set: OK (All pass)");
  } else {
    Serial.println("Filt4 set: FAILED");
  }
  if (can.setFilt(5, 0, 0x00000000)) {
    Serial.println("Filt5 set: OK (All pass)");
  } else {
    Serial.println("Filt5 set: FAILED");
  }
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize JSON document structure
  JsonDoc["session_start"] = millis();
  JsonDoc["device_id"] = "WIO_BG770A_001";
  JsonDoc["can_messages"] = JsonArray();
  JsonDoc["vehicle_data"].to<JsonObject>();
  
  Serial.println();
  Serial.println("=== CAN Bus Initialization Complete ===");
  Serial.println("Listening for ALL CAN messages...");
  Serial.println("Format: [timestamp] CAN ID: 0xXXX | Data: XX XX XX XX XX XX XX XX | Description");
  Serial.println("========================================");
}

void loop() {
  // 統計情報用の変数
  static unsigned long totalMessages = 0;
  static unsigned long lastStatsTime = 0;
  
  // OBD-II PID要求を複数種類順番に送信
  static unsigned long lastSend = 0;
  static int sendInterval = 1000;
  static unsigned long lastCellularSend = 0;
  static int cellularSendInterval = 10000;
  
  // 送信するPIDの配列
  static unsigned char obdPids[] = {0x0C, 0x0D, 0x05, 0x04, 0x0F, 0x11, 0x42, 0x46};
  static const char* obdPidNames[] = {
    "Engine RPM", "Vehicle Speed", "Coolant Temperature", "Engine Load",
    "Intake Air Temperature", "Throttle Position", "Control Module Voltage", "Ambient Air Temperature"
  };
  static int currentPidIndex = 0;
  static int numPids = sizeof(obdPids) / sizeof(obdPids[0]);

  if(millis() - lastSend > sendInterval) {
    // 現在のPIDを送信
    unsigned char cmd = obdPids[currentPidIndex];
    can.sendPid(cmd);
    Serial.print("Sent OBD-II ");
    Serial.print(obdPidNames[currentPidIndex]);
    Serial.print(" request (PID: 0x");
    if(cmd < 0x10) Serial.print("0");
    Serial.print(cmd, HEX);
    Serial.println(")");
    
    // 次のPIDに移動（循環）
    currentPidIndex = (currentPidIndex + 1) % numPids;
    lastSend = millis();
  }
  // すべてのCAN信号を受信・表示
  unsigned long id = 0;
  unsigned char data[8];
  if(can.receive(&id, data)) {
    totalMessages++;
    
    // タイムスタンプごとにJSONに蓄積
    addCANMessageToJSON(id, data, millis());
    // OBD2データを車両データとしても記録
    updateVehicleData(data[2], data, millis());
    
    // 受信メッセージの詳細表示
    Serial.print("[");
    Serial.print(millis());
    Serial.print("ms] CAN ID: 0x");
    if(id < 0x100) Serial.print("0");
    if(id < 0x10) Serial.print("0");
    Serial.print(id, HEX);
    Serial.print(" | Data: ");
    
    for(int i = 0; i < 8; i++) {
      if(data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      if(i < 7) Serial.print(" ");
    }
    
    // データの解釈を試行
    Serial.print(" | ");
    
    // OBD-II応答の場合
    if((id >= 0x7E8 && id <= 0x7EF)) {
      Serial.print("OBD-II Response from ECU ");
      Serial.print(id - 0x7E8);
      if(data[0] >= 2 && data[1] == 0x41) {
        Serial.print(" | PID: 0x");
        Serial.print(data[2], HEX);
        
        
        // 特定のPIDの解釈
        switch(data[2]) {
          case 0x0C: // Engine RPM
            if(data[0] >= 4) {
              unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
              Serial.print(" | Engine RPM: ");
              Serial.print(rpm);
              Serial.print(" rpm");
            }
            break;
          case 0x0D: // Vehicle Speed
            Serial.print(" | Vehicle Speed: ");
            Serial.print(data[3]);
            Serial.print(" km/h");
            break;
          case 0x05: // Coolant Temperature
            Serial.print(" | Coolant Temp: ");
            Serial.print(data[3] - 40);
            Serial.print(" °C");
            break;
          case 0x04: // Engine Load
            Serial.print(" | Engine Load: ");
            Serial.print(data[3] * 100.0 / 255.0);
            Serial.print(" %");
            break;
          case 0x0F: // Intake Air Temperature
            Serial.print(" | Intake Air Temp: ");
            Serial.print(data[3] - 40);
            Serial.print(" °C");
            break;
          case 0x11: // Throttle Position
            Serial.print(" | Throttle Position: ");
            Serial.print(data[3] * 100.0 / 255.0);
            Serial.print(" %");
            break;
          case 0x42: // Control Module Voltage
            if(data[0] >= 4) {
              float voltage = ((data[3] * 256) + data[4]) / 1000.0;
              Serial.print(" | Control Module Voltage: ");
              Serial.print(voltage);
              Serial.print(" V");
            }
            break;
          case 0x46: // Ambient Air Temperature
            Serial.print(" | Ambient Air Temp: ");
            Serial.print(data[3] - 40);
            Serial.print(" °C");
            break;
        }
      }
    }
    // その他の一般的なCAN IDの場合
    else if(id >= 0x100 && id <= 0x7FF) {
      Serial.print("Standard CAN Frame");
    }
    else if(id >= 0x18000000) {
      Serial.print("Extended CAN Frame");
    }
    else {
      Serial.print("Unknown Frame Type");
    }    
    Serial.println();
  }

  if(millis() - lastCellularSend > cellularSendInterval) {
    cellularSend(JsonDoc);
    // 送信後、古いメッセージデータをクリア（最新のvehicle_dataは保持）
    cleanupOldMessages();
    lastCellularSend = millis();
  }
  
  // Optional: Enable debug mode by sending commands through Serial monitor
  // Uncomment the following line to enable debug mode
  //can.debugMode();
  can.debugPID();

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
  
  // メッセージタイプの判定
  if(id >= 0x7E8 && id <= 0x7EF) {
    message["type"] = "OBD2_Response";
    message["ecu_id"] = id - 0x7E8;
    
    if(data[0] >= 2 && data[1] == 0x41) {
      message["pid"] = "0x" + String(data[2], HEX);
    }
  } else if(id >= 0x100 && id <= 0x1FF) {
    message["type"] = "Engine_Control";
  } else if(id >= 0x200 && id <= 0x2FF) {
    message["type"] = "Transmission_Control";
  } else if(id >= 0x300 && id <= 0x3FF) {
    message["type"] = "Body_Control";
  } else {
    message["type"] = "Unknown";
  }
  
  // メッセージ数が多すぎる場合は古いものを削除（メモリ管理）
  if(messages.size() > 50) {
    messages.remove(0);  // 最古のメッセージを削除
  }
}

// 車両データを時系列で更新
void updateVehicleData(unsigned char pid, unsigned char* data, unsigned long timestamp) {
  JsonObject vehicleData = JsonDoc["vehicle_data"].to<JsonObject>();
  
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
    case 0x0C: // Engine RPM
      if(data[0] >= 4) {
        unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
        JsonObject rpmObj = vehicleData["engine_rpm"].to<JsonObject>();
        rpmObj["value"] = rpm;
        rpmObj["unit"] = "rpm";
        rpmObj["timestamp"] = timestamp;
      }
      break;
      
    case 0x0D: // Vehicle Speed
      {
        JsonObject speedObj = vehicleData["vehicle_speed"].to<JsonObject>();
        speedObj["value"] = data[3];
        speedObj["unit"] = "km/h";
        speedObj["timestamp"] = timestamp;
      }
      break;
      
    case 0x05: // Coolant Temperature
      {
        JsonObject coolantObj = vehicleData["coolant_temp"].to<JsonObject>();
        coolantObj["value"] = data[3] - 40;
        coolantObj["unit"] = "°C";
        coolantObj["timestamp"] = timestamp;
      }
      break;
      
    case 0x04: // Engine Load
      {
        JsonObject loadObj = vehicleData["engine_load"].to<JsonObject>();
        loadObj["value"] = data[3] * 100.0 / 255.0;
        loadObj["unit"] = "%";
        loadObj["timestamp"] = timestamp;
      }
      break;
      
    case 0x0F: // Intake Air Temperature
      {
        JsonObject intakeObj = vehicleData["intake_air_temp"].to<JsonObject>();
        intakeObj["value"] = data[3] - 40;
        intakeObj["unit"] = "°C";
        intakeObj["timestamp"] = timestamp;
      }
      break;
      
    case 0x11: // Throttle Position
      {
        JsonObject throttleObj = vehicleData["throttle_position"].to<JsonObject>();
        throttleObj["value"] = data[3] * 100.0 / 255.0;
        throttleObj["unit"] = "%";
        throttleObj["timestamp"] = timestamp;
      }
      break;
      
    case 0x42: // Control Module Voltage
      if(data[0] >= 4) {
        float voltage = ((data[3] * 256) + data[4]) / 1000.0;
        JsonObject voltageObj = vehicleData["control_module_voltage"].to<JsonObject>();
        voltageObj["value"] = voltage;
        voltageObj["unit"] = "V";
        voltageObj["timestamp"] = timestamp;
      }
      break;
      
    case 0x46: // Ambient Air Temperature
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