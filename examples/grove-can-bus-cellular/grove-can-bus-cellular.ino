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
#include <time.h>

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
//JsonObject vehicleData = JsonDoc.to<JsonObject>();
// Initialize JSON document structure
//JsonDoc["session_start"] = millis();
//JsonDoc["device_id"] = "WIO_BG770A_001";
//JsonArray canMessages = JsonDoc["can_messages"].to<JsonArray>();
JsonArray dataArray = JsonDoc.to<JsonArray>();
JsonObject vehicleData = dataArray.add<JsonObject>();
static constexpr int OBD_COMMAND_INTERVAL = 50; // [ms] 1つずつのOBD-IIコマンドを送信する間隔
static constexpr int OBD_INTERVAL = 5000; // [ms] OBD-IIのデータ取得のためのコマンド群を送信する間隔
//static constexpr int CELLULAR_INTERVAL = 60000; // [ms] セルラーデータの送信間隔
static constexpr int CELLULAR_INTERVAL = 35000; // [ms] セルラーデータの送信間隔
static constexpr int DTC_INTERVAL = 30000; // [ms] DTCのデータ取得のためのコマンドを送信する間隔

static constexpr int PSM_INTERVAL = 1000 * 60 * 5;        // [ms]
static constexpr int PSM_PERIOD = 60 * 6;                 // [s] モジュールがスリープ状態に入るまでの待機期間
static constexpr int PSM_ACTIVE = 2;                      // [s] モジュールがスリープから復帰して、通信可能な状態である時間
static constexpr int PSM_POWER_DOWN_TIMEOUT = 1000 * 60;  // [ms] PSMが電源OFFになるまでの時間

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
  //static int sendInterval = 50;
  static unsigned long lastCellularSend = 0;
  //static int cellularSendInterval = 20000;
  static unsigned long lastRotateSend = 0;
  //static int rotateSendInterval = 20000;
  // 送信するPIDの配列
  static unsigned char obdPids[] = {
    PID_ENGIN_PRM, PID_VEHICLE_SPEED, PID_COOLANT_TEMP, PID_ENGINE_LOAD, PID_INTAKE_AIR_TEMP,
    PID_THROTTLE_POS, PID_DISTANCE_TRAVELED, PID_ODOMETER, PID_CONTROL_MODULE_VOLTAGE, PID_AMBIENT_AIR_TEMP
  };
  static const char* obdPidNames[] = {
    "Engine RPM", "Vehicle Speed", "Coolant Temperature", "Engine Load",
    "Intake Air Temperature", "Throttle Position", "Distance Traveled", "Odometer", "Control Module Voltage", "Ambient Air Temperature"
  };
  static int currentPidIndex = 0;
  static int numPids = sizeof(obdPids) / sizeof(obdPids[0]);
  
  // DTC関連の変数
  static unsigned long lastDtcSend = 0;
  //static int dtcSendInterval = 30000; // 30秒ごとにDTCを取得

  if(millis() - lastRotateSend > OBD_INTERVAL) {
    if(millis() - lastSend > OBD_COMMAND_INTERVAL) {
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
      vehicleData = dataArray.add<JsonObject>(); // 配列に新しいデータを追加できるよう初期化
      Serial.println("--------------------------------");
    }
  }
  
  // DTC送信タイミング
  if(millis() - lastDtcSend > DTC_INTERVAL) {
    sendDtcRequest();
    lastDtcSend = millis();
  }
  
  // すべてのCAN信号を受信・表示
  unsigned long id = 0;
  unsigned char data[8];
  if(can.receive(&id, data)) {
    //totalMessages++;
    
    // DTCレスポンスかどうかをチェック
    if(id >= 0x7E8 && id <= 0x7EF && data[0] >= 3 && data[1] == 0x43) {
      processDtcData(data);
    } else {
      // OBD2データを車両データとしても記録
      updateVehicleData(data[2], data);
    }
    
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
      
      // DTCレスポンスの場合
      if(data[0] >= 3 && data[1] == 0x43) {
        Serial.print(" | DTC Response | Count: ");
        Serial.print(data[2]);
      }
      // 通常のPIDレスポンスの場合
      else if(data[0] >= 2 && data[1] == 0x41) {
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
          case PID_ODOMETER: // Odometer
            if(data[0] >= 6) {
              unsigned long odometer = ((unsigned long)data[3] << 24) + ((unsigned long)data[4] << 16) + ((unsigned long)data[5] << 8) + data[6];
              Serial.print(" | Odometer: ");
              Serial.print(odometer * 0.1);
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

  if(millis() - lastCellularSend > CELLULAR_INTERVAL) {
    cellularSend(JsonDoc);
    clearData(); // 送信後、データをクリア
    lastCellularSend = millis();
  }
  
  // Optional: Enable debug mode by sending commands through Serial monitor
  // Uncomment the following line to enable debug mode
  //can.debugMode();
  //can.debugPID();

  delay(10);
}

// DTC送信機能
void sendDtcRequest() {
  can.sendDtc(PID_DTC_READ);
  Serial.println("Sent DTC request");
}

// DTCコードを文字列に変換
String dtcToString(unsigned char highByte, unsigned char lowByte) {
  String dtcType = "";
  String dtcNumber = "";
  
  // DTCタイプの判定（上位2ビット）
  unsigned char type = (highByte >> 6) & 0x03;
  switch(type) {
    case 0x00: dtcType = "P"; break;
    case 0x01: dtcType = "C"; break;
    case 0x02: dtcType = "B"; break;
    case 0x03: dtcType = "U"; break;
  }
  
  // DTCナンバーの取得（14ビット、10進数）
  uint8_t d2 = (highByte >> 4) & 0x03; // only 2 bits valid per spec
  uint8_t d3 = highByte & 0x0F;
  uint8_t d4 = (lowByte >> 4) & 0x0F;
  uint8_t d5 = lowByte & 0x0F;
  dtcNumber = String(d2) + String(d3) + String(d4) + String(d5);
  
  return dtcType + dtcNumber;
}

// DTCデータを処理
void processDtcData(unsigned char* data) {
  // ISO-TP PCI 判定
  const uint8_t pciType = data[0] & 0xF0;   // 0x0? = SingleFrame, 0x1? = FirstFrame, 0x2? = Consecutive, 0x3? = Flow
  const uint8_t sfPayloadLen = data[0] & 0x0F; // SingleFrame のペイロード長

  // Single Frame 以外（マルチフレーム）は未対応（今後再組立てが必要）
  if (pciType == 0x10) {
    // First Frame of multi-frame response for DTCs
    Serial.println("DTC response is multi-frame (not reassembled). Skipping for now.");
    vehicleData["dtc_codes"] = "MULTI";  // マルチフレームは未対応のため FALSE を格納
    vehicleData["dtc_count"] = 0;
    return;
  }
  if (pciType != 0x00) {
    Serial.println("pciType != 0x00");
    return; // 未対応フレーム
  }

  // 期待: data[1] = 0x43 (Mode 03 Response)
  if (sfPayloadLen >= 3 && data[1] == 0x43) {
    // 以降はDTCの2バイトペアの列
    // このフレーム内で実際に載っているDTC数（Single Frameに収まる分のみ）
    const uint8_t bytesAfterService = sfPayloadLen - 1; // service(0x43) を除いたバイト数
    uint8_t dtcPairsInFrame = bytesAfterService / 2;
    // フレーム上の位置の上限（データは data[2]～data[7]）
    dtcPairsInFrame = min<uint8_t>(dtcPairsInFrame, 3);

    if (dtcPairsInFrame == 0) {
      vehicleData["dtc_codes"] = "";
      vehicleData["dtc_count"] = 0;
      Serial.println("No DTCs found in single-frame response");
      return;
    }

    String dtcCodes = "";
    for (uint8_t i = 0; i < dtcPairsInFrame; i++) {
      const uint8_t hi = data[2 + i * 2];
      const uint8_t lo = data[3 + i * 2];
      String dtcCode = dtcToString(hi, lo);
      Serial.print("DTC ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(dtcCode);
      if (dtcCodes.length() > 0) dtcCodes += ",";
      dtcCodes += dtcCode;
    }

    vehicleData["dtc_codes"] = dtcCodes;
    vehicleData["dtc_count"] = dtcPairsInFrame; // Single Frame内で取得できた件数
    //vehicleData["dtc_timestamp"] = timestamp;
  }
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

// 車両データを単一オブジェクトに更新
void updateVehicleData(unsigned char pid, unsigned char* data) {
  // 単一のオブジェクトに全データを格納
  
  // raw_dataを文字列として格納（最新のもので上書き）
  String rawDataString = "";
  for(int i = 0; i < 8; i++) {
    if(data[i] < 0x10) rawDataString += "0";
    rawDataString += String(data[i], HEX);
    if(i < 7) rawDataString += " ";
  }
  //vehicleData["raw_data"] = rawDataString;
  //vehicleData["pid"] = pid;
  if (!vehicleData["time"].is<String>()) {
    const auto start_time = millis();
    vehicleData["time"] = GetTime();
    const auto end_time = millis();
    Serial.print("時刻を取得するのにかかった時間(ms):");
    Serial.println(end_time - start_time);
  }

  switch(pid) {
    case PID_ENGIN_PRM: // Engine RPM
      if(data[0] >= 4) {
        unsigned int rpm = ((data[3] * 256) + data[4]) / 4;
        vehicleData["engine_rpm"] = rpm;
      }
      break;
      
    case PID_VEHICLE_SPEED: // Vehicle Speed
      {
        vehicleData["vehicle_speed"] = data[3];
      }
      break;
      
    case PID_COOLANT_TEMP: // Coolant Temperature
      {
        vehicleData["coolant_temp"] = data[3] - 40;
      }
      break;
      
    case PID_ENGINE_LOAD: // Engine Load
      {
        vehicleData["engine_load"] = data[3] * 100.0 / 255.0;
      }
      break;
      
    case PID_INTAKE_AIR_TEMP: // Intake Air Temperature
      {
        vehicleData["intake_air_temp"] = data[3] - 40;
      }
      break;
      
    case PID_THROTTLE_POS: // Throttle Position
      {
        vehicleData["throttle_position"] = data[3] * 100.0 / 255.0;
      }
      break;
      
    case PID_DISTANCE_TRAVELED: // Distance Traveled
      if(data[0] >= 4) {
        unsigned int distance = (data[3] * 256) + data[4];
        vehicleData["distance_traveled"] = distance;
      }
      break;
      
    case PID_ODOMETER: // Odometer
      if(data[0] >= 6) {
        unsigned long odometer = ((unsigned long)data[3] << 24) + ((unsigned long)data[4] << 16) + ((unsigned long)data[5] << 8) + data[6];
        vehicleData["odometer"] = odometer * 0.1;
      }
      break;
      
    case PID_CONTROL_MODULE_VOLTAGE: // Control Module Voltage
      if(data[0] >= 4) {
        float voltage = ((data[3] * 256) + data[4]) / 1000.0;
        vehicleData["control_module_voltage"] = voltage;
      }
      break;
      
    case PID_AMBIENT_AIR_TEMP: // Ambient Air Temperature
      {
        vehicleData["ambient_air_temp"] = data[3] - 40;
      }
      break;
  }
}

// 古いメッセージをクリーンアップ（メモリ管理）
void clearData() {
  // vehicleData.clear(); // 全データをクリアする場合
  dataArray.clear();
  Serial.println("dataArrayをクリアしました。");
}

static bool cellularSend(const JsonDocument &doc) {
  Serial.println("### Sending Combined Vehicle Data Object");
  Serial.print("Object keys count: ");
  Serial.println(doc.size());

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

// Get current JST time via worldtimeapi.org
String GetTime() {
  const char* host = "worldtimeapi.org";
  const int port = 80;

  WioCellularTcpClient2<WioCellularModule> client{ WioCellular };
  if (!client.open(WioNetwork.config.pdpContextId, host, port)) {
    Serial.printf("ERROR: Open time API %s\n", WioCellularResultToString(client.getLastResult()));
    return "";
  }
  if (!client.waitForConnect()) {
    Serial.printf("ERROR: Connect time API %s\n", WioCellularResultToString(client.getLastResult()));
    return "";
  }

  const char* req =
    "GET /api/timezone/Asia/Tokyo HTTP/1.1\r\n"
    "Host: worldtimeapi.org\r\n"
    "Connection: close\r\n\r\n";
  if (!client.send(req, strlen(req))) {
    Serial.printf("ERROR: Send time API %s\n", WioCellularResultToString(client.getLastResult()));
    return "";
  }

  static uint8_t buf[WioCellular.RECEIVE_SOCKET_SIZE_MAX];
  size_t recvSize;
  if (!client.receive(buf, sizeof(buf), &recvSize, RECEIVE_TIMEOUT)) {
    Serial.printf("ERROR: Receive time API %s\n", WioCellularResultToString(client.getLastResult()));
    return "";
  }

  // Parse HTTP response: find JSON body
  const char* resp = reinterpret_cast<const char*>(buf);
  const char* body = strstr(resp, "\r\n\r\n");
  if (!body) return "";
  body += 4;

  // JSON parse
  JsonDocument jd;
  auto err = deserializeJson(jd, body);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return "";
  }
  if (jd["datetime"].is<const char*>()) {
    // worldtimeapi datetime: 2024-08-11T10:23:45.123456+09:00
    const char* dt = jd["datetime"];
    int Y,M,D,h,m,s;
    if (sscanf(dt, "%d-%d-%dT%d:%d:%d", &Y,&M,&D,&h,&m,&s) == 6) {
      struct tm t{};
      t.tm_year = Y - 1900;
      t.tm_mon = M - 1;
      t.tm_mday = D;
      t.tm_hour = h;
      t.tm_min = m;
      t.tm_sec = s;
      return jd["datetime"];
    }
  }
  return "";
}
/*
String timestamp() {
  const auto start_time = millis();
  time_t t;
  GetTime(&t);
  struct tm *tm_info = localtime(&t);
  char jstTimeStr[25];
  snprintf(jstTimeStr, sizeof(jstTimeStr), "%04d-%02d-%02dT%02d:%02d:%02d+09:00",
           tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
           tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  const auto end_time = millis();
  Serial.print("時刻を取得するのにかかった時間(ms):");
  Serial.println(end_time - start_time);
  return jstTimeStr;
}
*/
