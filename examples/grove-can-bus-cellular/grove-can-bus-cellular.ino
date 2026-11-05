/*
 * grove-can-bus.ino
 * Grove - CAN BUS Module based on GD32E103 for Wio BG770A
 * Copyright (C) AutoSystems, Inc.
 * MIT License
 */

#include <Adafruit_TinyUSB.h>
#include <csignal>
#include <WioCellular.h>
#include <grove-can-bus.h>
#include <ArduinoJson.h>
#include <cmath>
#include <string>
#include <time.h>
#include <telemetry-services.h>
#include <telemetry-time.h>
#include <obd-session.h>
#include <endpoint-ack.h>
#include <functional>
#include <utility>
#ifndef TELEMETRY_NTP
#define TELEMETRY_NTP 1
#endif
#ifndef TELEMETRY_GNSS
#define TELEMETRY_GNSS 1
#endif
// Explicit compatibility policy: only the ten existing standard Mode 01 PIDs.
// Unknown remains unknown until a validated positive reply; exclusions are skipped.
#ifndef OBD_ALLOW_UNKNOWN_PIDS
#define OBD_ALLOW_UNKNOWN_PIDS 1
#endif
#ifndef OBD_CAN_RATE
#define OBD_CAN_RATE CAN_RATE_500
#endif
#ifndef OBD_CAN_EXTENDED
#define OBD_CAN_EXTENDED 0
#endif
static_assert(OBD_CAN_RATE==CAN_RATE_250 || OBD_CAN_RATE==CAN_RATE_500, "Choose a verified CAN rate");
static_assert(OBD_CAN_EXTENDED==0 || OBD_CAN_EXTENDED==1, "Choose CAN ID width");

#define SEARCH_ACCESS_TECHNOLOGY (WioCellularNetwork::SearchAccessTechnology::LTEM)  // https://seeedjp.github.io/Wiki/Wio_BG770A/kb/kb4.html
#define LTEM_BAND (WioCellularNetwork::NTTDOCOMO_LTEM_BAND) // https://seeedjp.github.io/Wiki/Wio_BG770A/kb/kb4.html
static const char APN[] = "soracom.io";
static const char HOST[] = "uni.soracom.io";
static constexpr int PORT = 23080;
static constexpr int POWER_ON_TIMEOUT = 1000 * 20;     // [ms]
static constexpr int NETWORK_TIMEOUT = 1000 * 60 * 3;  // [ms]
static constexpr int RECEIVE_TIMEOUT = 1000 * 10;      // [ms]
static JsonDocument JsonDoc;
//JsonObject vehicleData = JsonDoc.to<JsonObject>();
JsonArray dataArray = JsonDoc["data"].to<JsonArray>();
JsonObject vehicleData; // dataArrayに追加するJsonObject
char deviceIMSI[20]; // SIMカードのIMSI情報を保存するための変数

static constexpr int OBD_INTERVAL = 10000; // [ms] OBD-IIのデータ取得のためのコマンド群を送信する間隔
//static constexpr int CELLULAR_INTERVAL = 60000; // [ms] セルラーデータの送信間隔
static constexpr int CELLULAR_INTERVAL = 30000; // [ms] セルラーデータの送信間隔
static constexpr int DTC_INTERVAL = 60000; // [ms] DTCのデータ取得のためのコマンドを送信する間隔

static constexpr int POWER_OFF_INTERVAL = 1000 * 60 * 10; // スリープ時間[ms]（10分間）
//static constexpr int POWER_OFF_INTERVAL = 1000 * 60 * 1; // スリープ時間[ms]（1分間）
//static constexpr int POWER_OFF_INTERVAL = 1000 * 60 * 2; // スリープ時間[ms]（2分間）
static constexpr int POWER_OFF_DELAY_TIME = 1000 * 3;  // [ms]
static constexpr unsigned long DATA_EMPTY_TIMEOUT = 60 * 1000 * 1; // データの受信がなくなってからスリープに入るまでの時間 (ms)
static constexpr unsigned long ACK_INTERVAL = 60 * 60 * 1000; // スリープに入ってからも定期的にACKを送信する間隔 (ms)
//static constexpr unsigned long ACK_INTERVAL = 1000 * 60 * 5; // スリープに入ってからも定期的にACKを送信する間隔 (ms)
static constexpr unsigned long ABORT_TIME = 1000 * 60 * 5; // ABORTが発生してリセットがかかるまでの時間[ms]（5分間）

WioCAN can;
static bg770a::TelemetryServices Services;
static bg770a::ObdSession Obd(OBD_CAN_EXTENDED, OBD_ALLOW_UNKNOWN_PIDS);
static bool CanReady=false, TelemetryAttempted=false, HaveCanActivity=false, ModemSleeping=false, SynchronousModem=false;
static bool DtcWaiting=false;
static bool HeartbeatPending=false;
static uint32_t DtcStarted=0;
static uint32_t LastCanActivity=0;
using ServiceHandler = decltype(std::declval<WioCellularModule&>().registerUrcHandler2(
    std::declval<std::function<bool(const std::string&)>>()));
static ServiceHandler ServicesHandler;
void initCAN();
void sendDtcRequest();
void processDtcData(unsigned char*);
String dtcToString(unsigned char, unsigned char);
void updateVehicleData(unsigned char, unsigned char*);
void clearData();
static bool cellularSend(const JsonDocument&);
static void initializeVehicleDataSchema();
bool getIMSI(char*);
bool getTime(char*);
String formatTime(char*);
String addMillisTime();
bool arrayEmptyCheck(JsonArrayConst);
void netPowerOnRestart();
void netPowerOffWait();
template<typename T> void printData(T&, const void*, size_t);

static void serviceModem(bool allowNtp=true, bool allowGnss=false) {
  Services.now(millis());
  if(ModemSleeping)return;
  for(int i=0;i<4;++i) WioCellular.doWork(0);
  if(SynchronousModem)return;
  Services.tick(millis());
  if(Services.gnss.state()==bg770a::GnssSession::State::ResetNeeded) {
    WioCellular.getInterface().reset();
    Services.gnss.resetIssued(millis());
  }
  const bool online=WioNetwork.getNetworkState()==WioCellularNetwork::NetworkState::Connected;
  const auto command=Services.command(millis(),WioNetwork.config.pdpContextId,online,
                                      allowNtp,allowGnss && TelemetryAttempted);
  if(!command.empty())WioCellular.writeBinary(command.data(),command.size());
}

static void serviceCan() {
  Obd.tick(millis());
  if(DtcWaiting && uint32_t(millis()-DtcStarted)>=200)DtcWaiting=false;
  unsigned long id=0; unsigned char bytes[8]={};
  for(unsigned n=0;n<8 && can.receive(&id,bytes);++n) {
    const uint32_t receivedAt=millis();
    // receive() may block: enforce the deadline after every returned frame.
    if(DtcWaiting && uint32_t(receivedAt-DtcStarted)>=200)DtcWaiting=false;
    const bool telemetry=Obd.accept(id,bytes,sizeof(bytes),receivedAt);
    const bool dtc=DtcWaiting && Obd.selected() && id==Obd.ecu() &&
      ((bytes[0]>=1 && bytes[0]<=7 && bytes[1]==0x43 && bytes[0]%2==1) ||
       ((bytes[0]&0xF0)==0x10 && ((bytes[0]&15)*256U+bytes[1])>7 && bytes[2]==0x43));
    // Valid activity wakes the modem even when UTC is unavailable. Capability
    // replies are activity too, but never create a measurement record.
    if(Obd.acceptedFrame() || dtc) {
      HaveCanActivity=true; LastCanActivity=receivedAt;
    }
    if(dtc)DtcWaiting=false;
    if(!telemetry && !dtc)continue;
    int64_t epoch;
    if(!Services.clock.utc(Services.now(millis()),epoch))continue;
    if(vehicleData.isNull())initializeVehicleDataSchema();
    if(telemetry)updateVehicleData(bytes[2],bytes);
    if(dtc)processDtcData(bytes);
  }
}
static void cooperativeDelay(uint32_t duration) {
  const uint32_t start=millis();
  do { serviceModem();serviceCan();delay(1); }
  while(uint32_t(millis()-start)<duration);
}
static void attachFreshLocation() {
  JsonDoc.remove("location");
  const auto& fix=Services.gnss.fix();
  if(!bg770a::isFresh(fix,millis()))return;
  JsonObject location=JsonDoc["location"].to<JsonObject>();
  location["latitude"]=fix.latitude;location["longitude"]=fix.longitude;
  location["altitude_m"]=fix.altitude;location["hdop"]=fix.hdop;
  location["fix_type"]=fix.fix;location["satellites"]=fix.satellites;
  location["utc_hhmmss"]=fix.utc;location["date_ddmmyy"]=fix.date;
  location["age_ms"]=uint32_t(millis()-fix.receivedMillis);
  location["source"]="gnss";
}


static void abortHandler(int sig) {
  Serial.printf("ABORT: Signal %d received\n", sig);
  //yield();

  //vTaskSuspendAll();  // FreeRTOS
  Serial.print(ABORT_TIME/1000);Serial.println("秒待ってリセットします。");
  const auto start = millis();
  while (millis()-start<ABORT_TIME) {
    ledOn(LED_BUILTIN);
    nrfx_coredep_delay_us(100000);  // Spin
    ledOff(LED_BUILTIN);
    nrfx_coredep_delay_us(100000);  // Spin
  }
  NVIC_SystemReset();
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

  Serial.println("Cellular bus initialized");
  // Network configuration
  WioNetwork.config.searchAccessTechnology = SEARCH_ACCESS_TECHNOLOGY;
  WioNetwork.config.ltemBand = LTEM_BAND;
  WioNetwork.config.apn = APN;
  WioCellular.begin();
  // Power on the cellular module
  if (WioCellular.powerOn(POWER_ON_TIMEOUT) != WioCellularResult::Ok) abort();
  Services.ntpEnabled=TELEMETRY_NTP;
  Services.gnssEnabled=TELEMETRY_GNSS;
  ServicesHandler=WioCellular.registerUrcHandler2([](const std::string& line) {
    return Services.line(line,millis());
  });
  WioNetwork.begin();
  // Wait for communication available
  if (!WioNetwork.waitUntilCommunicationAvailable(NETWORK_TIMEOUT)) abort();

  Serial.println("CAN bus initialized");
  initCAN();

  char startupTime[64];
  // Preserve bounded HTTP fallback when NTP cannot provide a validated sample.
  // If both fail, continue without fabricating a timestamp or reboot loop.
  if(!getTime(startupTime))Serial.println("Clock unsynced: defer timestamped records until synchronization");

  // Get IMSI for device identification
  if(Services.idle() && getIMSI(deviceIMSI)) {
    Serial.println("Device identity ready (redacted)");
    // IMSIをJSONドキュメントのルートレベルに追加
    JsonDoc["imsi"] = deviceIMSI;
  } else {
    Serial.println("IMSIを取得できませんでした。");
    // Retry after asynchronous modem recovery; never overlap AT owners.
  }
  
  //initializeVehicleDataSchema();
  digitalWrite(LED_BUILTIN, LOW);
    
  Serial.println();
  Serial.println("Listening for ALL CAN messages...");
  Serial.println("Format: [timestamp] CAN ID: 0xXXX | Data: XX XX XX XX XX XX XX XX | Description");
  Serial.println("========================================");
}

void loop() {
  serviceModem();
  serviceCan();
  if(HaveCanActivity && ModemSleeping && Services.atIdle())netPowerOnRestart();
  if(Services.idle() && !ModemSleeping && !deviceIMSI[0] && getIMSI(deviceIMSI))JsonDoc["imsi"]=deviceIMSI;
  static uint32_t lastClockRetry=millis();
  if(Services.idle() && !ModemSleeping && uint32_t(millis()-lastClockRetry)>=60000 &&
      strcmp(Services.clock.source(),"unsynced")==0) {
    char time[64]; getTime(time);lastClockRetry=millis();
  }
  static uint32_t lastCellularSend=millis(), lastRotateSend=0, batchStarted=0;
  static uint32_t lastDtcSend=millis(), lastDataReceivedTime=millis(), lastACKTime=millis();
  static bool batch=false;
  const uint32_t now=millis();
  if(HaveCanActivity) { lastDataReceivedTime=LastCanActivity; HaveCanActivity=false; }
  if(uint32_t(now-lastRotateSend)>=OBD_INTERVAL) {
    if(!batch) {
      vehicleData=JsonObject(); batch=true; batchStarted=now;
      if(!CanReady)Serial.println("CAN polling disabled: module configuration unconfirmed");
    }
    unsigned char pid;
    if(CanReady && !DtcWaiting && Obd.next(now,pid)) {
      const unsigned char request[8]={2,1,pid,0,0,0,0,0};
      const bool sent=can.send(Obd.requestId(),OBD_CAN_EXTENDED,0,8,request);
      Serial.print(sent ? "Queued Mode 01 PID 0x" : "Failed Mode 01 PID 0x");
      Serial.print(pid,HEX);
      Serial.println(Obd.support(pid)==bg770a::Support::Unknown ? " (capability unknown)" : "");
    }
    // Allow discovery through PID A0 and all current telemetry within the batch.
    if(uint32_t(now-batchStarted)>=4000 && !Obd.waiting()) {
      lastRotateSend=now;batch=false;
    }
  }
  if(uint32_t(now-lastDtcSend)>=DTC_INTERVAL && !Obd.waiting() && !DtcWaiting) {
    if(CanReady && Obd.selected())sendDtcRequest();
    lastDtcSend=now;
  }
  JsonObject diagnostics=JsonDoc["obd"].to<JsonObject>();
  diagnostics["coverage"]="vehicle_unverified";
  diagnostics["can_ready"]=CanReady;
  diagnostics["unknown_pid_fallback"]=Obd.compatibilityEnabled();
  diagnostics["distance_traveled_meaning"]="distance_since_dtc_clear";
  if(Obd.selected())diagnostics["response_id"]=Obd.ecu();
  const unsigned char pids[]={0x0C,0x0D,0x05,0x04,0x0F,0x11,0x31,0xA6,0x42,0x46};
  for(auto pid:pids) {
    const auto state=Obd.support(pid);
    diagnostics["supported_pids"][String(pid,HEX)]=state==bg770a::Support::Supported ? "supported" :
      state==bg770a::Support::Unsupported ? "unsupported" : "unknown";
  }
  JsonDoc["time_source"]=Services.clock.source();
  JsonDoc["time_sync_state"]=Services.clock.quality(Services.now(now));

  if(millis() - lastCellularSend > CELLULAR_INTERVAL && Services.atIdle()) {
    if(arrayEmptyCheck(JsonDoc["data"])) {
      Serial.println("doc data is empty.");
      unsigned long emptyDuration = millis() - lastDataReceivedTime;
      if(emptyDuration > DATA_EMPTY_TIMEOUT) { // DATA_EMPTY_TIMEOUT時間経過チェック
        unsigned long lastACKDuration = millis() - lastACKTime;
        if(lastACKDuration > ACK_INTERVAL && !HeartbeatPending) {
          HeartbeatPending=true;
          Serial.println("Heartbeat pending: ingest requires measured records; empty send deferred");
        }
        Serial.print("OBD2の受信データが");Serial.print(emptyDuration / 1000);Serial.println("秒間なかったためWioCellularとCAN BUSをOFFにします。");
        digitalWrite(PIN_VGROVE_ENABLE, VGROVE_ENABLE_OFF);
        netPowerOffWait();
        Serial.println("もう一度OBD2の受信データがあるかどうかを確かめ、なければスリープ状態を継続します。");
        initCAN();
        clearData(); // 送信後、データをクリア
        lastCellularSend = millis();
        //return false; // データは送信しないまま終わらす
        return; // データは送信しないまま終わらす
      }
    }else {
      Serial.println("doc data is ok.");
      // DATA_EMPTY_TIMEOUT以上経過したあとで、そのあと受診データが確認された場合はpowerOnを開始する
      if(ModemSleeping) {
        netPowerOnRestart();
      }
      lastDataReceivedTime = millis(); // 最後にデータを受信した時刻を更新
      //lastDataReceivedDuration=0;
    }
    // The ingest contract rejects data:[]; retain heartbeat intent locally.
    if(dataArray.size()==0) { lastCellularSend=millis();cooperativeDelay(10);return; }
    TelemetryAttempted=true;
    if(cellularSend(JsonDoc)) { clearData();lastACKTime=millis();HeartbeatPending=false; } // 送信後、データをクリア
    lastCellularSend = millis();
  }
  
  // Optional: Enable debug mode by sending commands through Serial monitor
  // Uncomment the following line to enable debug mode
  //can.debugMode();
  //can.debugPID();

  serviceModem(true,true);
  cooperativeDelay(10);
}

// DTC送信機能
void sendDtcRequest() {
  const unsigned char request[8]={1,3,0,0,0,0,0,0};
  can.send(Obd.requestId(),OBD_CAN_EXTENDED,0,8,request);
  DtcWaiting=true; DtcStarted=millis();
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
  dtcNumber = String(d2, HEX) + String(d3, HEX) + String(d4, HEX) + String(d5, HEX);
  dtcNumber.toUpperCase();
  
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
  if (sfPayloadLen >= 1 && sfPayloadLen<=7 && sfPayloadLen%2==1 && data[1] == 0x43) {
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
    unsigned count=0;
    for (uint8_t i = 0; i < dtcPairsInFrame; i++) {
      const uint8_t hi = data[2 + i * 2];
      const uint8_t lo = data[3 + i * 2];
      if(hi==0 && lo==0)continue;
      ++count;
      String dtcCode = dtcToString(hi, lo);
      Serial.print("DTC ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(dtcCode);
      if (dtcCodes.length() > 0) dtcCodes += " ";
      dtcCodes += dtcCode;
    }

    vehicleData["dtc_codes"] = dtcCodes;
    vehicleData["dtc_count"] = count; // Single Frame内で取得できた件数
    //vehicleData["dtc_timestamp"] = timestamp;
  }
}

// 車両データを単一オブジェクトに更新
void updateVehicleData(unsigned char pid, unsigned char* data) {
  // 単一のオブジェクトに全データを格納
  /*
  // raw_dataを文字列として格納（最新のもので上書き）
  String rawDataString = "";
  for(int i = 0; i < 8; i++) {
    if(data[i] < 0x10) rawDataString += "0";
    rawDataString += String(data[i], HEX);
    if(i < 7) rawDataString += " ";
  }
  vehicleData["raw_data"] = rawDataString;
  vehicleData["pid"] = pid;
  */

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
        vehicleData["engine_load"] = std::lround(static_cast<double>(data[3]) * 100.0 / 255.0);
      }
      break;
      
    case PID_INTAKE_AIR_TEMP: // Intake Air Temperature
      {
        vehicleData["intake_air_temp"] = data[3] - 40;
      }
      break;
      
    case PID_THROTTLE_POS: // Throttle Position
      {
        vehicleData["throttle_position"] = std::lround(static_cast<double>(data[3]) * 100.0 / 255.0);
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
        vehicleData["odometer"] = std::lround(static_cast<double>(odometer) * 0.1);
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
  dataArray.clear();
  vehicleData=JsonObject();
  Serial.println("dataArrayをクリアしました。");
}

static bool cellularSend(const JsonDocument &doc) {
  if(!Services.idle() || !deviceIMSI[0] || ModemSleeping || doc["data"].size()==0)return false;
  Serial.println("### Sending Combined Vehicle Data Object");
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
    attachFreshLocation(); // A slow TCP connection may outlive fix freshness.
    serializeJson(doc, str);
    Serial.print(str.size());Serial.print(" bytes (payload redacted)");
    Serial.println();
    if (!client.send(str.data(), str.size())) {
      Serial.printf("ERROR: Failed to send socket %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    Serial.println("Receiving");
    static uint8_t recvData[WioCellular.RECEIVE_SOCKET_SIZE_MAX];
    bg770a::EndpointAck ack;
    const uint32_t began=millis();
    for(unsigned part=0;part<16 && uint32_t(millis()-began)<RECEIVE_TIMEOUT;++part) {
      size_t recvSize=0;
      const int remaining=RECEIVE_TIMEOUT-int(uint32_t(millis()-began));
      if(!client.receive(recvData,sizeof(recvData),&recvSize,remaining))return false;
      if(recvSize==0) { ack.finish();break; }
      if(ack.feed(recvData,recvSize)!=bg770a::EndpointAck::Result::Pending)break;
    }
    Serial.print("Endpoint status: ");Serial.println(ack.status());
    if(ack.result()!=bg770a::EndpointAck::Result::Accepted) {
      Serial.println("Endpoint rejected/unconfirmed; retaining records");return false;
    }

  }

  Serial.println("### Completed");

  return true;
}

static void initializeVehicleDataSchema() {
  if(dataArray.size()>=12)dataArray.remove(0);
  vehicleData = dataArray.add<JsonObject>(); // Preserve bounded latest samples
  vehicleData["time_source"]=Services.clock.source();
  vehicleData["time_sync_state"]=Services.clock.quality(Services.now(millis()));
  int64_t epoch;
  if(Services.clock.utc(Services.now(millis()),epoch))vehicleData["utc_epoch_s"]=epoch;
  vehicleData["time"] = addMillisTime(); // 経過時間を計算して現在時刻を生成, String (ISO8601)
  // Ingest API (Zod) expects every key present on each record; OBD updates overwrite below.
  vehicleData["engine_rpm"] = 0;
  vehicleData["vehicle_speed"] = 0;
  vehicleData["coolant_temp"] = 0;
  vehicleData["engine_load"] = 0;
  vehicleData["intake_air_temp"] = 0;
  vehicleData["throttle_position"] = 0;
  vehicleData["distance_traveled"] = 0;
  vehicleData["odometer"] = 0;
  vehicleData["control_module_voltage"] = 0;
  vehicleData["ambient_air_temp"] = 0;
  vehicleData["dtc_codes"] = "";
  vehicleData["dtc_count"] = 0;
}

template<typename T>
void printData(T &stream, const void *data, size_t size) {
  auto p = static_cast<const char *>(data);

  for (; size > 0; --size, ++p)
    stream.write(0x20 <= *p && *p <= 0x7f ? *p : '.');
}

// Get IMSI from SIM card
bool getIMSI(char* imsi) {
  Serial.println("Getting IMSI...");
  
  // WioCellularライブラリのgetIMSI APIを使用してIMSIを取得
  std::string imsiStr;
  const auto result = WioCellular.getIMSI(&imsiStr);
  
  if (result == WioCellularResult::Ok && !imsiStr.empty()) {
    strncpy(imsi, imsiStr.c_str(), 19);
    imsi[19] = '\0';
    Serial.println("Device identity ready (redacted)");
    return true;
  } else {
    Serial.println("ERROR: Failed to get IMSI");
    return false;
  }
}

// worldtimeapi: 1 回の receive では JSON が切れることがあるため複数回読み集める
static bool getTimeFromWorldTimeApiHttp(char* time) {
  const char* host = "worldtimeapi.org";
  const int port = 80;
  const int maxRetries = 2;
  
  for (int retry = 0; retry < maxRetries; retry++) {
    if (retry > 0) {
      Serial.printf("Retrying time API request (%d/%d)\n", retry + 1, maxRetries);
      cooperativeDelay(5000); // 5秒待機してからリトライ
    }
    
    WioCellularTcpClient2<WioCellularModule> client{ WioCellular };
    if (!client.open(WioNetwork.config.pdpContextId, host, port)) {
      Serial.printf("ERROR: Open time API %s\n", WioCellularResultToString(client.getLastResult()));
      continue; // 次のリトライへ
    }
    if (!client.waitForConnect()) {
      Serial.printf("ERROR: Connect time API %s\n", WioCellularResultToString(client.getLastResult()));
      continue; // 次のリトライへ
    }

    const char* req =
      "GET /api/timezone/Asia/Tokyo HTTP/1.1\r\n"
      "Host: worldtimeapi.org\r\n"
      "Connection: close\r\n\r\n";
    if (!client.send(req, strlen(req))) {
      Serial.printf("ERROR: Send time API %s\n", WioCellularResultToString(client.getLastResult()));
      continue; // 次のリトライへ
    }

    static uint8_t buf[WioCellular.RECEIVE_SOCKET_SIZE_MAX];
    size_t total = 0;
    const uint32_t began = millis();
    while (total + 1 < sizeof(buf) && uint32_t(millis()-began) < 45000) {
      size_t recvSize = 0;
      if (!client.receive(buf + total, sizeof(buf) - 1 - total, &recvSize, RECEIVE_TIMEOUT)) {
        break;
      }
      if (recvSize == 0) break;
      total += recvSize;
      buf[total] = '\0';
      const char* hdrEnd = strstr(reinterpret_cast<const char*>(buf), "\r\n\r\n");
      if (hdrEnd) {
        JsonDocument probe;
        int64_t epoch;
        if(!deserializeJson(probe,hdrEnd+4) && probe["datetime"].is<const char*>() &&
           bg770a::parseHttpTime(probe["datetime"].as<const char*>(),epoch))break;
      }
    }

    if (total == 0) {
      Serial.printf("ERROR: Receive time API (no data) %s\n", WioCellularResultToString(client.getLastResult()));
      continue;
    }

    const char* resp = reinterpret_cast<const char*>(buf);
    const char* body = strstr(resp, "\r\n\r\n");
    if (!body) {
      Serial.println("ERROR: HTTP response body not found");
      continue; // 次のリトライへ
    }
    body += 4;

    JsonDocument jd;
    const auto err = deserializeJson(jd, body);
    if (err) {
      Serial.print("JSON parse error: ");
      Serial.println(err.c_str());
      continue; // 次のリトライへ
    }
    if (jd["datetime"].is<const char*>()) {
      const char* dt = jd["datetime"];
      int Y, M, D, h, m, s;
      if (sscanf(dt, "%d-%d-%dT%d:%d:%d", &Y, &M, &D, &h, &m, &s) == 6) {
        strncpy(time, dt, 63);
        time[63] = '\0';
        return true;
      }
      Serial.println("ERROR: Failed to parse datetime format");
      continue;
    }
    Serial.println("ERROR: datetime field not found in JSON");
  }

  Serial.printf("ERROR: Failed to get time after %d HTTP retries\n", maxRetries);
  return false;
}

bool getTime(char* time) {
  if(WioNetwork.getNetworkState()!=WioCellularNetwork::NetworkState::Connected)return false;
  const uint32_t start=millis();
  while(TELEMETRY_NTP && uint32_t(millis()-start)<135000) {
    serviceModem();serviceCan();
    int64_t epoch;
    if(Services.clock.utc(Services.now(millis()),epoch)) {
      const auto iso=bg770a::isoUtc(epoch);strncpy(time,iso.c_str(),63);time[63]=0;return true;
    }
    if(Services.ntp.modemIdle() && !Services.clock.ready(Services.now(millis())))break;
    delay(1);
  }
  // Never run synchronous fallback in an uncertain/resetting AT session.
  if(!Services.idle())return false;
  SynchronousModem=true;
  const bool fallback=getTimeFromWorldTimeApiHttp(time);
  SynchronousModem=false;
  if(!fallback)return false;
  int64_t epoch;
  if(!bg770a::parseHttpTime(time,epoch))return false;
  Services.clock.seedFallback(epoch,Services.now(millis()));
  return true;
}
String formatTime(char* time) { return String(time); }
String addMillisTime() {
  int64_t epoch;
  if(!Services.clock.utc(Services.now(millis()),epoch))return String();
  return String(bg770a::isoUtc(epoch).c_str());
}

bool arrayEmptyCheck(JsonArrayConst arr) { // doc["data"]のすべての要素がtimeしかないかどうかを調べる
  for (JsonObjectConst obj : arr) {
    Serial.print("obj size: ");Serial.println(obj.size());
    if (obj.size() > 1) { // 要素が1より多くある場合はtime以外の要素もあることを示す
      return false;
    }
  }
  return true;
}

void netPowerOffWait() {
  if(!Services.atIdle())return;
  ModemSleeping=true;
  // Power off the cellular module
  // WioCellularがOFFになっているかどうか確かめる
  if (!WioCellular.getInterface().isActive()) {
    Serial.println("WioCellularはすでにOFFになっていたため、delay関数でPOWER_OFF_INTERVAL時間待ち、OFFを継続します。");
    delay(POWER_OFF_INTERVAL);
    return;
  }
  WioCellular.doWorkUntil(POWER_OFF_DELAY_TIME);
  auto start = millis();
  Serial.print("WioNetwork.end()が開始されます。");Serial.println(start);
  WioNetwork.end();
  Serial.print("WioNetwork.end()完了にかかった時間(ms)");Serial.println(millis() - start);

  start = millis();
  Serial.print("WioCellular.powerOff()が開始されます。");Serial.println(start);
  if (WioCellular.powerOff() != WioCellularResult::Ok) abort();
  Serial.print("WioCellular.powerOff()完了にかかった時間(ms)");Serial.println(millis() - start);

  start = millis();
  Serial.print("WioCellular.doWorkUntil(POWER_OFF_INTERVAL)が開始されます。");Serial.println(start);
  WioCellular.doWorkUntil(POWER_OFF_INTERVAL);  
  Serial.print("WioCellular.doWorkUntil(POWER_OFF_INTERVAL)完了にかかった時間(ms)");Serial.println(millis() - start);
}

void netPowerOnRestart() { // powerOnを開始する
  // Power on the cellular module
  auto start = millis(); // 計測用
  Serial.print("powerOn開始");Serial.println(start);
  if (WioCellular.powerOn(POWER_ON_TIMEOUT) != WioCellularResult::Ok) abort();
  Serial.print("powerOn完了にかかった時間(ms)");Serial.println(millis() - start);
  
  Services.powerCycle();
  WioNetwork.begin();
  start = millis();
  Serial.print("ネットワーク再開を開始します。");Serial.println(start);
  const bool networkFlag=WioNetwork.waitUntilCommunicationAvailable(NETWORK_TIMEOUT);
  if (networkFlag) {
    ModemSleeping=false;
    Serial.print("ネットワークが再開できるまでの時間(ms)");Serial.println(millis() - start);
  }
  else{
    Serial.print("ERROR: ネットワークが再開できませんでした。");Serial.println(millis());
  }
}

void initCAN() {
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
  Obd=bg770a::ObdSession(OBD_CAN_EXTENDED, OBD_ALLOW_UNKNOWN_PIDS);
  DtcWaiting=false;
  // Inherited default; verify the actual vehicle before selecting rate/ID width.
  if((CanReady=can.setCanRate(OBD_CAN_RATE))) {
    Serial.println("Configured CAN bus rate: OK");
  } else {
    Serial.println("Configured CAN bus rate: FAILED");
  }

  // --- set mask and filter (standard frame) ---
  // Mask設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setMask(0, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Mask0 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Mask0 set: FAILED");
  }
  if (can.setMask(1, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Mask1 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Mask1 set: FAILED");
  }

  // Filter設定: 0x00000000で全てのIDを通す（全通し）
  if (can.setFilt(0, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt0 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Filt0 set: FAILED");
  }
  if (can.setFilt(1, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt1 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Filt1 set: FAILED");
  }
  if (can.setFilt(2, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt2 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Filt2 set: FAILED");
  }
  if (can.setFilt(3, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt3 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Filt3 set: FAILED");
  }
  if (can.setFilt(4, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt4 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Filt4 set: FAILED");
  }
  if (can.setFilt(5, OBD_CAN_EXTENDED, 0x00000000)) {
    Serial.println("Filt5 set: OK (All pass)");
  } else {
    CanReady=false;Serial.println("Filt5 set: FAILED");
  }
}
