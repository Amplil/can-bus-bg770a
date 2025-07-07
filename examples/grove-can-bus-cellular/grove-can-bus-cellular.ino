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
JsonArray rootArray = JsonDoc.to<JsonArray>();
// Initialize JSON document structure
//JsonDoc["session_start"] = millis();
//JsonDoc["device_id"] = "WIO_BG770A_001";
//JsonArray canMessages = JsonDoc["can_messages"].to<JsonArray>();
//JsonArray vehicleData = JsonDoc["vehicle_data"].to<JsonArray>();

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
    
  Serial.println();
  Serial.println("=== CAN Bus Initialization Complete ===");
  Serial.println("Listening for ALL CAN messages...");
  Serial.println("Format: [timestamp] CAN ID: 0xXXX | Data: XX XX XX XX XX XX XX XX | Description");
  Serial.println("========================================");
}

void loop() {
  // 統計情報用の変数
  //static unsigned long totalMessages = 0;
  static unsigned long lastStatsTime = 0;
  
  // OBD-II PID要求を複数種類順番に送信
  static unsigned long lastSend = 0;
  static int sendInterval = 50;
  static unsigned long lastCellularSend = 0;
  static int cellularSendInterval = 20000;
  static unsigned long lastRotateSend = 0;
  static int rotateSendInterval = 2000;
  // 送信するPIDの配列
  static unsigned char obdPids[] = {
    PID_ENGIN_PRM, PID_VEHICLE_SPEED, PID_COOLANT_TEMP, PID_ENGINE_LOAD, PID_INTAKE_AIR_TEMP,
    PID_THROTTLE_POS, PID_DISTANCE_TRAVELED, PID_CONTROL_MODULE_VOLTAGE, PID_AMBIENT_AIR_TEMP
  };
  static const char* obdPidNames[] = {
    "Engine RPM", "Vehicle Speed", "Coolant Temperature", "Engine Load",
    "Intake Air Temperature", "Throttle Position", "Distance Traveled", "Control Module Voltage", "Ambient Air Temperature"
  };
  static int currentPidIndex = 0;
  static int numPids = sizeof(obdPids) / sizeof(obdPids[0]);

  if(millis() - lastRotateSend > rotateSendInterval) {
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
    if(currentPidIndex == 0) {
      lastRotateSend = millis();
      Serial.println("--------------------------------");
    }
  }
  // すべてのCAN信号を受信・表示
  unsigned long id = 0;
  unsigned char data[8];
  if(can.receive(&id, data)) {
    //totalMessages++;
    
    // タイムスタンプごとにJSONに蓄積
    //addCANMessageToJSON(id, data, millis());
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
          case PID_ENGIN_PRM: // Engine RPM
            if(data[0] >= 4) {
              unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
              Serial.print(" | Engine RPM: ");
              Serial.print(rpm);
              Serial.print(" rpm");
            }
            break;
          case PID_VEHICLE_SPEED: // Vehicle Speed
            Serial.print(" | Vehicle Speed: ");
            Serial.print(data[3]);
            Serial.print(" km/h");
            break;
          case PID_COOLANT_TEMP: // Coolant Temperature
            Serial.print(" | Coolant Temp: ");
            Serial.print(data[3] - 40);
            Serial.print(" °C");
            break;
          case PID_ENGINE_LOAD: // Engine Load
            Serial.print(" | Engine Load: ");
            Serial.print(data[3] * 100.0 / 255.0);
            Serial.print(" %");
            break;
          case PID_INTAKE_AIR_TEMP: // Intake Air Temperature
            Serial.print(" | Intake Air Temp: ");
            Serial.print(data[3] - 40);
            Serial.print(" °C");
            break;
          case PID_THROTTLE_POS: // Throttle Position
            Serial.print(" | Throttle Position: ");
            Serial.print(data[3] * 100.0 / 255.0);
            Serial.print(" %");
            break;
          case PID_DISTANCE_TRAVELED: // Distance Traveled
            if(data[0] >= 4) {
              unsigned int distance = (data[3] * 256) + data[4];
              Serial.print(" | Distance Traveled: ");
              Serial.print(distance);
              Serial.print(" km");
            }
            break;
          case PID_CONTROL_MODULE_VOLTAGE: // Control Module Voltage
            if(data[0] >= 4) {
              float voltage = ((data[3] * 256) + data[4]) / 1000.0;
              Serial.print(" | Control Module Voltage: ");
              Serial.print(voltage);
              Serial.print(" V");
            }
            break;
          case PID_AMBIENT_AIR_TEMP: // Ambient Air Temperature
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
/*
// CANメッセージをタイムスタンプ付きでJSONに追加
void addCANMessageToJSON(unsigned long id, unsigned char* data, unsigned long timestamp) {
  //JsonArray messages = JsonDoc["can_messages"];
  JsonObject message = rootArray.add<JsonObject>();
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
}
*/

// 車両データを時系列で更新
void updateVehicleData(unsigned char pid, unsigned char* data, unsigned long timestamp) {
  JsonObject dataObj = rootArray.add<JsonObject>();
  JsonArray rawDataArray = dataObj["raw_data"].to<JsonArray>();
  // 既存のデータをクリア
  //rawDataArray.clear();
  for(int i = 0; i < 8; i++) {
    rawDataArray.add(data[i]);
  }
  dataObj["pid"] = pid;
  dataObj["timestamp"] = timestamp;

  switch(pid) {
    case PID_ENGIN_PRM: // Engine RPM
      if(data[0] >= 4) {
        unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
        dataObj["engine_rpm"] = rpm;
      }
      break;
      
    case PID_VEHICLE_SPEED: // Vehicle Speed
      {
        dataObj["vehicle_speed"] = data[3];
      }
      break;
      
    case PID_COOLANT_TEMP: // Coolant Temperature
      {
        dataObj["coolant_temp"] = data[3] - 40;
        //coolantObj["value"] = data[3] - 40;
        //coolantObj["unit"] = "°C";
        //coolantObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_ENGINE_LOAD: // Engine Load
      {
        dataObj["engine_load"] = data[3] * 100.0 / 255.0;
        //loadObj["value"] = data[3] * 100.0 / 255.0;
        //loadObj["unit"] = "%";
        //loadObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_INTAKE_AIR_TEMP: // Intake Air Temperature
      {
        dataObj["intake_air_temp"] = data[3] - 40;
        //intakeObj["value"] = data[3] - 40;
        //intakeObj["unit"] = "°C";
        //intakeObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_THROTTLE_POS: // Throttle Position
      {
        dataObj["throttle_position"] = data[3] * 100.0 / 255.0;
        //throttleObj["value"] = data[3] * 100.0 / 255.0;
        //throttleObj["unit"] = "%";
        //throttleObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_DISTANCE_TRAVELED: // Distance Traveled
      if(data[0] >= 4) {
        unsigned int distance = (data[3] * 256) + data[4];
        dataObj["distance_traveled"] = distance;
        //distanceObj["value"] = distance;
        //distanceObj["unit"] = "km";
        //distanceObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_CONTROL_MODULE_VOLTAGE: // Control Module Voltage
      if(data[0] >= 4) {
        float voltage = ((data[3] * 256) + data[4]) / 1000.0;
        dataObj["control_module_voltage"] = voltage;
        //voltageObj["value"] = voltage;
        //voltageObj["unit"] = "V";
        //voltageObj["timestamp"] = timestamp;
      }
      break;
      
    case PID_AMBIENT_AIR_TEMP: // Ambient Air Temperature
      {
        dataObj["ambient_air_temp"] = data[3] - 40;
        //ambientObj["value"] = data[3] - 40;
        //ambientObj["unit"] = "°C";  
        //ambientObj["timestamp"] = timestamp;
      }
      break;
  }
}

// 古いメッセージをクリーンアップ（メモリ管理）
void cleanupOldMessages() {
  //canMessages.clear();
  //vehicleData.clear();
  rootArray.clear();
  Serial.println("Cleaned up CAN data messages after transmission");
}

static bool cellularSend(const JsonDocument &doc) {
  Serial.println("### Sending Time-Series JSON Data");
  //Serial.print("Messages in queue: ");
  //Serial.println(doc["can_messages"].size());
  //Serial.print("Vehicle data entries: ");
  //Serial.println(doc["vehicle_data"].size());

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