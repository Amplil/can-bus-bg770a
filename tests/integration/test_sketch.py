#!/usr/bin/env python3
"""Compile current sketch functions and loop against real JSON/AT dispatcher."""
import os, pathlib, re, subprocess, tempfile, json
root=pathlib.Path(__file__).resolve().parents[2]
sketch=(root/'examples/grove-can-bus-cellular/grove-can-bus-cellular.ino').read_text()
field=json.loads((root/'tests/fixtures/field-pid00-utc.json').read_text())
observed_rx=next(e for e in field['events'] if e['kind']=='rx')
def function(signature):
    start=sketch.index(signature)
    while ';' in sketch[start:sketch.index('{',start)]:
        start=sketch.index(signature,start+len(signature))
    brace=sketch.index('{',start); depth=1; end=brace+1
    # These functions contain no unmatched braces in strings/comments.
    while depth:
        if sketch[end]=='{': depth+=1
        if sketch[end]=='}': depth-=1
        end+=1
    return sketch[start:end]
headers=(root/'src/grove-can-bus.h').read_text()
defines='\n'.join(re.findall(r'^#define (?:PID_|CAN_RATE_).*',headers,re.M))
constants=sketch[sketch.index('static constexpr int OBD_INTERVAL'):sketch.index('WioCAN can;')]
source=r'''
#include <ArduinoJson.h>
#include <telemetry-services.h>
#include <telemetry-time.h>
#include <obd-session.h>
#include <endpoint-ack.h>
#include <telemetry-batch.h>
#include <telemetry-record.h>
#include <can-diagnostics.h>
#include <obd-dtc.h>
#include <cassert>
#include <array>
#include <deque>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <cctype>
#include <cstring>
#include <new>
static uint32_t nowMs=0;
static bool advanceMillis=false;
static bool allowStartupReadWait=false;
uint32_t millis(){const auto value=nowMs;if(advanceMillis)++nowMs;return value;}
void delay(unsigned ms){nowMs+=ms;}
#include "module/at_client/AtClient.hpp"
struct Uart {
 std::deque<unsigned char> rx;std::string tx;unsigned resets=0;
 void waitReadAvailable(int t){assert(t==0 || (allowStartupReadWait && t>0 && t<=10));nowMs+=t;}
 int read(){if(rx.empty())return -1;auto c=rx.front();rx.pop_front();return c;}
 void write(int c){tx+=char(c);}
 void reset(){++resets;}
};
struct Module:wiocellular::module::at_client::AtClient<Module>{static constexpr size_t RECEIVE_SOCKET_SIZE_MAX=256;Uart uart;Uart& getInterface(){return uart;}} WioCellular;
struct Network {
 enum class NetworkState{Connected,Offline};
 struct Config{int pdpContextId=1;}config;
 NetworkState state=NetworkState::Connected;
 NetworkState getNetworkState(){return state;}
 bool canCommunicate(){return state==NetworkState::Connected;}
 static const char* networkStateToString(NetworkState state){return state==NetworkState::Connected?"Connected":"Offline";}
} WioNetwork;
using WioCellularNetwork=Network;
struct String:std::string {
 using std::string::string;
 String(const std::string& s):std::string(s){}
 String(unsigned long n,int base=10){std::ostringstream s;if(base==16)s<<std::hex;s<<n;assign(s.str());}
 void toUpperCase(){for(auto& c:*this)c=char(std::toupper(static_cast<unsigned char>(c)));}
};
constexpr int HEX=16;
using std::min;
struct SerialType {
 std::string output;
 explicit operator bool()const{return true;}
 template<class T>void print(const T& value){std::ostringstream stream;stream<<value;output+=stream.str();}
 template<class T>void print(T value,int base){std::ostringstream stream;if(base==16)stream<<std::hex;stream<<static_cast<unsigned long>(value);output+=stream.str();}
 void println(){output+="\n";}
 template<class T>void println(const T& value){print(value);println();}
 template<class... T>void printf(const char* format,T... values){char line[1024];std::snprintf(line,sizeof(line),format,values...);output+=line;}
} Serial;
struct Frame{unsigned long id;std::array<unsigned char,8> bytes;uint32_t delayMs=0;};
struct Can {
 std::deque<Frame> rx;std::deque<std::array<unsigned char,8>> sent;
 bool rateOK=true,maskOK=true,filterOK=true,sendOK=true;
 void begin(){} bool setCanRate(unsigned){return rateOK;}
 bool setMask(unsigned,unsigned,unsigned long){return maskOK;}
 bool setFilt(unsigned,unsigned,unsigned long){return filterOK;}
 bool receive(unsigned long* id,unsigned char* data){if(rx.empty())return false;auto f=rx.front();rx.pop_front();nowMs+=f.delayMs;*id=f.id;std::copy(f.bytes.begin(),f.bytes.end(),data);return true;}
 bool send(unsigned long,unsigned char,unsigned char,unsigned char size,const unsigned char* bytes){assert(size==8);assert(bytes[1]==1||bytes[1]==3);std::array<unsigned char,8> f;std::copy(bytes,bytes+8,f.begin());sent.push_back(f);return sendOK;}
}can;
static JsonDocument JsonDoc;
static JsonArray dataArray=JsonDoc["data"].to<JsonArray>();
static JsonObject vehicleData;
static char deviceIMSI[20]={};
static bg770a::TelemetryServices Services;
static bg770a::ObdSession Obd(false,true);
static bool CanReady=true,HaveCanActivity=false,ModemSleeping=false,DtcWaiting=false,SynchronousModem=false,HeartbeatPending=false;
enum class StartupLocationState {Pending,Acquiring,Upload,Complete};
static StartupLocationState StartupLocation=StartupLocationState::Pending;
static JsonDocument StartupLocationDoc;
static bool StartupLocationAttempted=false;
static uint32_t StartupLocationLastAttempt=0;
static bool CellularReady=true;
static uint32_t LastCellularAttempt=0;
static constexpr uint32_t CELLULAR_RETRY_INTERVAL=60000;
static constexpr size_t CELLULAR_PAYLOAD_LIMIT=1024;
static size_t LastSentRecords=0;
static unsigned cellularInitCalls=0;
static bool initializeCellular(){++cellularInitCalls;LastCellularAttempt=millis();return false;}
static bg770a::CanDiagnostics CanStats;
static bool DtcExpired=false;
static uint32_t DtcStarted=0;
static uint32_t LastCanActivity=0;
constexpr unsigned CAN_BAUDRATE=9600;
constexpr unsigned OBD_CAN_EXTENDED=0,OBD_ALLOW_UNKNOWN_PIDS=1,OBD_CAN_RATE=16;
constexpr unsigned PIN_VGROVE_ENABLE=0,VGROVE_ENABLE_OFF=0,VGROVE_ENABLE_ON=1;
static bool sendResult=false;static unsigned sendCalls=0,powerOffs=0,powerOns=0;
void digitalWrite(unsigned,unsigned){}
constexpr unsigned TELEMETRY_NTP=0;
static void serviceModem(bool,bool);
bool getTime(char*);
static std::string httpTime;
static unsigned identityCalls=0,httpCalls=0;
static bool getTimeFromWorldTimeApiHttp(char* out){
 ++httpCalls;assert(SynchronousModem);const auto before=WioCellular.uart.tx;
 serviceModem(true,true);assert(WioCellular.uart.tx==before);
 std::strncpy(out,httpTime.c_str(),63);out[63]=0;return !httpTime.empty();
}
bool getIMSI(char* out){++identityCalls;std::strcpy(out,"fixture-device");return true;}
void initCAN();
void netPowerOffWait(){assert(Services.atIdle());++powerOffs;ModemSleeping=true;}
void netPowerOnRestart(){assert(Services.atIdle());++powerOns;Services.powerCycle();ModemSleeping=false;WioNetwork.state=Network::NetworkState::Connected;}
void clearData();
static void clearSentData(size_t);
static void discardEmptyVehicleRecords();
static bool cellularSend(const JsonDocument&);
static bool queueStartupLocation();
static bool serviceStartupLocation();
static unsigned socketOpens=0;
static uint32_t socketDelay=0;
static bool socketOpenResult=true,socketConnectResult=true;
static std::deque<std::string> SentPayloads;
static std::deque<std::string> ReplyChunks={"200 {}\r\n"};
using WioCellularModule=Module;
static constexpr const char* HOST="fixture.invalid";
static constexpr int PORT=23080,RECEIVE_TIMEOUT=10000;
static const char* WioCellularResultToString(int){return "fixture-status";}
template<class T>struct WioCellularTcpClient2 {
 std::deque<std::string> chunks=ReplyChunks;
 explicit WioCellularTcpClient2(T&){}
 bool open(int,const char*,int){++socketOpens;nowMs+=socketDelay;return socketOpenResult;}
 bool waitForConnect(){return socketConnectResult;}
 bool send(const char* bytes,size_t size){++sendCalls;SentPayloads.emplace_back(bytes,size);return sendResult;}
 int getLastResult(){return 0;}
 bool receive(uint8_t* out,size_t cap,size_t* size,int remaining){assert(remaining>0);nowMs+=1;
  if(chunks.empty()){*size=0;return true;}
  auto part=chunks.front();chunks.pop_front();assert(part.size()<=cap);
  std::copy(part.begin(),part.end(),out);*size=part.size();return true;
 }
};
static void initializeVehicleDataSchema();
void updateVehicleData(unsigned char,unsigned char*);
void processDtcData(unsigned char*);
String addMillisTime();
void sendDtcRequest();
bool arrayEmptyCheck(JsonArrayConst);
'''+defines+'\n'+constants+'\n'
for signature in ['static bool canIdle()', 'static bool cellularOnline()', 'static void trace(', 'static bool hasGpsPosition(', 'static size_t gpsRowCount(', 'static std::string gpsPayloadTime(', 'static void logGpsPosition(', 'static void logGpsPayload(', 'static void logGpsDiagnostic()', 'static const char* rejectionName(', 'static void traceFrame(', 'static void tickCan(', 'static void canSummary()', 'static void serviceModem(', 'static void serviceCan()', 'static void cooperativeDelay(', 'static bool waitForCellularNetwork(',
 'static void attachFreshLocation()', 'static bool queueStartupLocation()', 'static bool dispatchServiceLine(', 'static bool serviceStartupLocation()', 'static bool cellularSend(const JsonDocument &doc)', 'void initCAN() {', 'String dtcToString(', 'void processDtcData(unsigned char* data)',
 'void updateVehicleData(unsigned char pid,', 'void clearData() {', 'static void clearSentData(', 'static void discardEmptyVehicleRecords() {', 'static void initializeVehicleDataSchema() {',
 'String addMillisTime() {', 'bool getTime(char* time) {', 'bool arrayEmptyCheck(', 'void sendDtcRequest() {','void loop() {']:
    source+=function(signature)+'\n'
source+=r'''
static void pollUntilSend(unsigned expected) {
 for(unsigned n=0;n<400 && sendCalls<expected;++n){nowMs+=10;loop();}
 assert(sendCalls==expected);
}
static void startupReply(const char* command) {
 const std::string sent=std::string(command)+"\r";
 assert(WioCellular.uart.tx.size()>=sent.size());
 assert(WioCellular.uart.tx.compare(WioCellular.uart.tx.size()-sent.size(),sent.size(),sent)==0);
 assert(dispatchServiceLine(command));assert(dispatchServiceLine("OK"));loop();
}
static void completeStartupGnss(bool hasFix=true,bool registered=true) {
 loop();assert(StartupLocation==StartupLocationState::Acquiring);
 assert(sendCalls==0 && can.sent.empty());
 startupReply("AT+CFUN=4");startupReply("AT+QGPS=1");
 if(hasFix) {
  nowMs+=1000;loop();
  assert(dispatchServiceLine("AT+QGPSLOC=2"));
  assert(dispatchServiceLine("+QGPSLOC: 123456.789,35.12345,139.12345,1.2,12.0,3,0.00,0.0,0.0,061126,08"));
  assert(dispatchServiceLine("OK"));loop();
  assert(StartupLocationDoc["data"].size()==1);
 } else {
  nowMs+=Services.gnss.acquisitionTimeoutMs();loop();
  assert(!Services.gnss.fix().valid);
 }
 startupReply("AT+QGPSEND");startupReply("AT+CFUN=1");startupReply("AT+CEREG=1");
 assert(dispatchServiceLine("AT+CEREG?"));
 assert(dispatchServiceLine(registered?"+CEREG: 1,1":"+CEREG: 1,0"));assert(dispatchServiceLine("OK"));loop();
}
static size_t logCount(const std::string& token) {
 size_t result=0,position=0;
 while((position=Serial.output.find(token,position))!=std::string::npos){++result;position+=token.size();}
 return result;
}
int main(int argc,char** argv){
 if(argc>1 && std::string(argv[1])=="gps-observability") {
  // Synthetic modem replies: coordinates are published only after final OK.
  Services.ntpEnabled=false;Services.gnssEnabled=true;sendResult=true;CanReady=false;
  loop();assert(StartupLocation==StartupLocationState::Acquiring);
  assert(Serial.output.find("[GPS-ACQUIRE]")!=std::string::npos);
  assert(logCount("[GPS-FIX]")==0 && logCount("[GPS-ACK]")==0);
  startupReply("AT+CFUN=4");startupReply("AT+QGPS=1");
  nowMs+=1000;loop();assert(dispatchServiceLine("AT+QGPSLOC=2"));
  assert(dispatchServiceLine("+CME ERROR: 516"));loop();
  assert(Serial.output.find("status=no_fix cme_error=516")!=std::string::npos);
  assert(logCount("[GPS-FIX]")==0 && sendCalls==0);
  nowMs+=1000;loop();assert(dispatchServiceLine("AT+QGPSLOC=2"));
  assert(dispatchServiceLine("+QGPSLOC: fixture-private-invalid-location"));
  assert(Serial.output.find("status=invalid_response")!=std::string::npos);
  assert(Serial.output.find("fixture-private-invalid-location")==std::string::npos);
  assert(dispatchServiceLine("OK"));loop();assert(!Services.gnss.fix().valid);
  nowMs+=1000;loop();assert(dispatchServiceLine("AT+QGPSLOC=2"));
  assert(dispatchServiceLine("+QGPSLOC: 123456.789,0.00000,0.00000,1.2,0.0,3,0.00,0.0,0.0,061126,08"));
  assert(!Services.gnss.fix().valid && logCount("[GPS-FIX]")==0);
  assert(dispatchServiceLine("OK"));
  assert(Services.gnss.fix().valid && logCount("[GPS-FIX]")==1);
  assert(Serial.output.find("status=acquired gps_utc=2026-11-06T12:34:56.789+00:00")!=std::string::npos);
  assert(Serial.output.find("latitude=0.000000 longitude=0.000000 altitude_m=0.0")!=std::string::npos);
  assert(StartupLocation==StartupLocationState::Upload && StartupLocationDoc["data"].size()==1);
  assert(logCount("[GPS-QUEUE]")==1 && logCount("status=awaiting_ack")==0);
  loop();startupReply("AT+QGPSEND");startupReply("AT+CFUN=1");startupReply("AT+CEREG=1");
  assert(dispatchServiceLine("AT+CEREG?"));assert(dispatchServiceLine("+CEREG: 1,1"));
  assert(dispatchServiceLine("OK"));loop();
  assert(sendCalls==1 && StartupLocation==StartupLocationState::Complete);
  assert(Serial.output.find("direct_gps_rows=1 root_location=0 records=1")!=std::string::npos);
  assert(Serial.output.find("source=record")!=std::string::npos);
  assert(logCount("status=awaiting_ack")==1);
  assert(logCount("status=accepted server_status=200 direct_gps_rows=1 root_location=0")==1);
  assert(logCount("[GPS-FIX]")==1);
  std::cout<<"GPS diagnostics distinguish polling/errors/candidates from a confirmed zero-coordinate fix, queued payload and positive upload acknowledgment\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="gps-upload-failures") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;sendResult=true;CanReady=false;
  socketOpenResult=false;completeStartupGnss();
  assert(sendCalls==0 && StartupLocationDoc["data"].size()==1);
  assert(Serial.output.find("status=open_failed")!=std::string::npos);
  assert(logCount("status=awaiting_ack")==0 && logCount("status=accepted server_status=")==0);
  socketOpenResult=true;socketConnectResult=false;nowMs+=31000;serviceCan();serviceStartupLocation();
  assert(sendCalls==0 && StartupLocationDoc["data"].size()==1);
  assert(Serial.output.find("status=connect_failed")!=std::string::npos);
  socketConnectResult=true;sendResult=false;nowMs+=31000;serviceCan();serviceStartupLocation();
  assert(sendCalls==1 && StartupLocationDoc["data"].size()==1);
  assert(Serial.output.find("status=send_failed")!=std::string::npos);
  assert(logCount("status=awaiting_ack")==0);
  sendResult=true;ReplyChunks={"400 {\"private\":\"fixture-private-response\"}"};
  nowMs+=31000;serviceCan();serviceStartupLocation();
  assert(sendCalls==2 && StartupLocationDoc["data"].size()==1);
  assert(logCount("status=awaiting_ack")==1 && logCount("status=accepted server_status=")==0);
  assert(Serial.output.find("status=unconfirmed server_status=400 direct_gps_rows=1 root_location=0 retained=1")!=std::string::npos);
  assert(Serial.output.find("fixture-private-response")==std::string::npos);
  ReplyChunks={"200 {\"ok\":true}"};nowMs+=31000;serviceCan();serviceStartupLocation();
  assert(sendCalls==3 && StartupLocation==StartupLocationState::Complete && StartupLocationDoc["data"].size()==0);
  assert(logCount("status=awaiting_ack")==2 && logCount("status=accepted server_status=200 direct_gps_rows=1 root_location=0")==1);
  std::cout<<"GPS upload diagnostics report open/connect/send/ACK failures and retain the fix; only a positive endpoint ACK reports accepted\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="gps-payload-prefix") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;sendResult=true;CanReady=false;
  Services.clock.seedFallback(1793968496LL,Services.now(0));loop();Serial.output.clear();
  for(unsigned n=0;n<6;++n){initializeVehicleDataSchema();vehicleData["engine_rpm"]=1000+n;}
  vehicleData["latitude"]=35.12345;vehicleData["longitude"]=139.12345;
  vehicleData["gps_time"]="2026-11-06T12:34:56.789+00:00";
  assert(cellularSend(JsonDoc) && LastSentRecords<6);
  assert(Serial.output.find("direct_gps_rows=0 root_location=0 records=")!=std::string::npos);
  assert(logCount("status=accepted server_status=200 direct_gps_rows=0 root_location=0")==1);
  assert(logCount("[GPS-POSITION]")==0);
  clearSentData(LastSentRecords);bool sawGps=false;
  while(dataArray.size()) {
   Serial.output.clear();assert(cellularSend(JsonDoc));
   JsonDocument sent;assert(!deserializeJson(sent,SentPayloads.back()));
   bool containsGps=false;for(JsonObjectConst row:sent["data"].as<JsonArrayConst>())if(!row["latitude"].isNull())containsGps=true;
   assert((logCount("status=accepted server_status=200 direct_gps_rows=1 root_location=0")==1)==containsGps);
   sawGps=sawGps||containsGps;clearSentData(LastSentRecords);
  }
  assert(sawGps);
  // A root-level fix is a separate, explicitly counted GPS representation.
  JsonDocument envelope;envelope["imsi"]="fixture-device";
  JsonObject record=envelope["data"].to<JsonArray>().add<JsonObject>();
  bg770a::initializeObdFields(record);record["time"]="2026-11-06T12:34:56+00:00";record["engine_rpm"]=1000;
  JsonObject location=envelope["location"].to<JsonObject>();
  location["latitude"]=35.12345;location["longitude"]=139.12345;location["altitude_m"]=0;
  location["utc_hhmmss"]="123456.789";location["date_ddmmyy"]="061126";
  location["hdop"]=1.2;location["fix_type"]=3;location["satellites"]=8;location["age_ms"]=0;location["source"]="gnss";
  Serial.output.clear();assert(cellularSend(envelope));
  assert(Serial.output.find("direct_gps_rows=0 root_location=1 records=1")!=std::string::npos);
  assert(Serial.output.find("source=root_location")!=std::string::npos);
  assert(Serial.output.find("gps_utc=2026-11-06T12:34:56.789+00:00")!=std::string::npos);
  assert(logCount("status=accepted server_status=200 direct_gps_rows=0 root_location=1")==1);
  std::cout<<"GPS payload/ACK counts describe only the actual transmitted prefix and distinguish direct GPS rows from root location metadata\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="gps-command-timeout-log") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;CanReady=false;loop();
  nowMs+=20000;loop();
  assert(Services.gnss.disabled() && WioCellular.uart.resets==1);
  assert(Serial.output.find("status=command_timeout")!=std::string::npos);
  assert(Serial.output.find("disabled=1")!=std::string::npos);
  assert(logCount("[GPS-FIX]")==0 && logCount("[GPS-ACK]")==0 && sendCalls==0);
  std::cout<<"GNSS command timeout and disabled recovery are visible without falsely claiming a fix or upload\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="qntp-utc-metadata") {
  Services.ntpEnabled=true;Services.gnssEnabled=false;sendResult=true;
  loop();assert(StartupLocation==StartupLocationState::Complete);
  assert(dispatchServiceLine(Services.ntp.command()));assert(dispatchServiceLine("OK"));
  // Synthetic QNTP input based on documented UTC semantics, not a claimed
  // raw response captured from the user's nine-hour-offset field excerpt.
  assert(dispatchServiceLine("+QNTP: 0,\"2026/11/06,13:01:57+36\""));
  loop();assert(deviceIMSI[0]); // Identity retrieval resumes after NTP releases AT ownership.
  assert(addMillisTime()=="2026-11-06T13:01:57+00:00");
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=1234;
  assert(vehicleData["time"]=="2026-11-06T13:01:57+00:00");
  assert(vehicleData["time_source"]=="ntp");
  assert(cellularSend(JsonDoc) && LastSentRecords==1);
  JsonDocument payload;assert(!deserializeJson(payload,SentPayloads.back()));
  assert(payload["data"][0]["time"]=="2026-11-06T13:01:57+00:00");
  assert(Serial.output.find("[LTE-SEND] 2026-11-06T13:01:57+00:00")!=std::string::npos);
  assert(Serial.output.find("response_time=2026/11/06,13:01:57+36 interpreted_utc=2026-11-06T13:01:57+00:00")!=std::string::npos);
  assert(Serial.output.find("2026-11-06T04:01:57+00:00")==std::string::npos);
  assert(dispatchServiceLine("+QNTP: 0,\"fixture-private-invalid-time\""));
  assert(Serial.output.find("fixture-private-invalid-time")==std::string::npos);
  // HTTP offset syntax remains local-time based and subtracts its offset once.
  int64_t epoch;assert(bg770a::parseHttpTime("2026-11-06T22:01:57+09:00",epoch));
  assert(bg770a::isoUtc(epoch)=="2026-11-06T13:01:57+00:00");
  std::cout<<"Synthetic QNTP UTC digits survive nonzero suffix metadata through clock, diagnostics, record and actual sender; HTTP offsets convert once\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="zero-records") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  uint8_t pid;assert(Obd.next(0,pid));
  const uint8_t bitmap[]={6,0x41,0,0,0x18,0,0,0};
  assert(!Obd.accept(0x7E8,bitmap,8,1));
  const char* fields[]={"engine_rpm","vehicle_speed","coolant_temp","engine_load",
   "intake_air_temp","throttle_position","distance_traveled","odometer",
   "control_module_voltage","ambient_air_temp","dtc_count","dtc_codes"};
  initializeVehicleDataSchema();
  for(const char* field:fields)assert(vehicleData.containsKey(field) && vehicleData[field].isNull());
  vehicleData["latitude"]=35;vehicleData["battery_level"]=99;
  assert(!bg770a::hasVehicleMeasurement(vehicleData));
  assert(!cellularSend(JsonDoc) && socketOpens==0);
  discardEmptyVehicleRecords();assert(dataArray.size()==0);
  // A complete no-fault reply is an acquired zero, unlike an unanswered query.
  sendDtcRequest();can.rx.push_back({0x7E8,{2,0x43,0,0,0,0,0,0}});
  serviceCan();assert(dataArray.size()==1 && !DtcWaiting);
  assert(vehicleData["dtc_count"]==0 && vehicleData["dtc_codes"]=="");
  assert(vehicleData["engine_rpm"].isNull() && bg770a::hasVehicleMeasurement(vehicleData));
  sendResult=true;assert(cellularSend(JsonDoc));
  JsonDocument noFault;assert(!deserializeJson(noFault,SentPayloads.back()));
  assert(noFault["data"][0]["dtc_count"]==0 && noFault["data"][0]["dtc_codes"]=="");
  assert(noFault["data"][0]["vehicle_speed"].isNull());
  clearSentData(LastSentRecords);
  // Malformed and timed-out RPM replies leave the field unmeasured.
  assert(!Obd.next(101,pid));assert(Obd.next(102,pid) && pid==0x0C);
  nowMs=103;can.rx.push_back({0x7E8,{3,0x41,0x0C,0,0,0,0,0}});serviceCan();
  assert(Obd.waiting() && dataArray.size()==0);
  nowMs=302;serviceCan();assert(!Obd.waiting() && dataArray.size()==0);
  // A validated stopped-speed response is a real zero and may be sent alone.
  assert(Obd.next(402,pid) && pid==0x0D);
  nowMs=403;can.rx.push_back({0x7E8,{3,0x41,0x0D,0,0,0,0,0}});serviceCan();
  assert(dataArray.size()==1 && vehicleData["vehicle_speed"]==0);
  assert(vehicleData["engine_rpm"].isNull() && vehicleData["dtc_count"].isNull());
  discardEmptyVehicleRecords();assert(dataArray.size()==1);
  assert(cellularSend(JsonDoc));
  JsonDocument stopped;assert(!deserializeJson(stopped,SentPayloads.back()));
  const JsonObjectConst stoppedRow=stopped["data"][0].as<JsonObjectConst>();
  assert(stoppedRow["vehicle_speed"]==0);
  for(const char* field:fields)if(std::strcmp(field,"vehicle_speed"))assert(stoppedRow[field].isNull());
  clearSentData(LastSentRecords);
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=0;vehicleData["vehicle_speed"]=0;
  initializeVehicleDataSchema();vehicleData["coolant_temp"]=0;
  initializeVehicleDataSchema();vehicleData["intake_air_temp"]=-40;
  initializeVehicleDataSchema();vehicleData["control_module_voltage"]=12.4;
  initializeVehicleDataSchema();vehicleData["dtc_count"]=1;vehicleData["dtc_codes"]="P01AF";
  initializeVehicleDataSchema();vehicleData["dtc_codes"]=" \t";
  initializeVehicleDataSchema();
  discardEmptyVehicleRecords();assert(dataArray.size()==5 && vehicleData.isNull());
  assert(dataArray[0]["engine_rpm"]==0 && dataArray[0]["vehicle_speed"]==0);
  assert(dataArray[1]["coolant_temp"]==0 && dataArray[2]["intake_air_temp"]==-40);
  while(dataArray.size()) {
   const auto queued=dataArray.size();assert(cellularSend(JsonDoc));
   JsonDocument sent;assert(!deserializeJson(sent,SentPayloads.back()));
   assert(sent["data"].size()==LastSentRecords && LastSentRecords>0);
   for(JsonObjectConst row:sent["data"].as<JsonArrayConst>())assert(bg770a::hasVehicleMeasurement(row));
   const auto acknowledged=LastSentRecords;clearSentData(acknowledged);
   assert(dataArray.size()==queued-acknowledged);
  }
  std::cout<<"Unavailable OBD values serialize as null; validated zero speed, zero temperature and no-fault DTC remain measured; empty rows excluded\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-gps-delayed-fix") {
  // The real sketch must keep searching past 30s and deliver a later first
  // fix independently of CAN, rather than merely extending a session constant.
  Services.ntpEnabled=false;Services.gnssEnabled=true;sendResult=true;CanReady=false;
  loop();assert(StartupLocation==StartupLocationState::Acquiring);
  assert(Serial.output.find("search_timeout_ms=180000")!=std::string::npos);
  startupReply("AT+CFUN=4");startupReply("AT+QGPS=1");
  nowMs=30000;loop();
  assert(StartupLocation==StartupLocationState::Acquiring && sendCalls==0);
  assert(dispatchServiceLine("AT+QGPSLOC=2"));
  assert(dispatchServiceLine("+CME ERROR: 516"));loop();
  assert(logCount("status=no_fix_timeout")==0 && logCount("[GPS-FIX]")==0);
  nowMs=40000;loop();assert(dispatchServiceLine("AT+QGPSLOC=2"));
  assert(dispatchServiceLine("+QGPSLOC: 123456.789,35.12345,139.12345,1.2,12.0,3,0.00,0.0,0.0,061126,08"));
  assert(logCount("[GPS-FIX]")==0);
  assert(dispatchServiceLine("OK"));loop();
  assert(logCount("[GPS-FIX]")==1 && sendCalls==0);
  const auto elapsed=Services.gnss.diagnostic().elapsedMs;
  assert(elapsed>bg770a::GnssSession::AcquisitionMs && elapsed<=40000);
  assert(Serial.output.find("elapsed_ms="+std::to_string(elapsed))!=std::string::npos);
  startupReply("AT+QGPSEND");startupReply("AT+CFUN=1");startupReply("AT+CEREG=1");
  assert(dispatchServiceLine("AT+CEREG?"));
  assert(dispatchServiceLine("+CEREG: 1,1"));assert(dispatchServiceLine("OK"));loop();
  assert(sendCalls==1 && StartupLocation==StartupLocationState::Complete);
  assert(logCount("[GPS-ACK]")==1 && dataArray.size()==0 && can.sent.empty());
  JsonDocument sent;assert(!deserializeJson(sent,SentPayloads.back()));
  JsonObjectConst row=sent["data"][0].as<JsonObjectConst>();
  assert(row["gps_time"]=="2026-11-06T12:34:56.789+00:00");
  assert(std::abs(row["latitude"].as<double>()-35.12345)<0.00001);
  assert(row["engine_rpm"].isNull() && row["vehicle_speed"].isNull());
  std::cout<<"First GPS fix acquired at40s survives former30s cutoff and is acknowledged without an ECU response\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-gps-first") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;
  // A queued OBD row must not precede the first acquired GPS upload.
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=2000;sendResult=true;
  ReplyChunks={"400 ","{\"error\":\"Invalid payload: decode or JSON failed\",\"private\":\"fixture-private-response\"}"};
  completeStartupGnss();assert(sendCalls==1);
  assert(dataArray.size()==1 && StartupLocation==StartupLocationState::Upload);
  assert(StartupLocationDoc["data"].size()==1 && Services.gnss.modemIdle());
  assert(Serial.output.find("reason=payload_decode")!=std::string::npos);
  assert(Serial.output.find("fixture-private-response")==std::string::npos);
  JsonDocument sent;assert(!deserializeJson(sent,SentPayloads.back()));
  JsonObjectConst row=sent["data"][0].as<JsonObjectConst>();
  assert(row["time"]=="2026-11-06T12:34:56+00:00");
  assert(row["gps_time"]=="2026-11-06T12:34:56.789+00:00");
  assert(std::abs(row["latitude"].as<double>()-35.12345)<0.00001);
  assert(std::abs(row["longitude"].as<double>()-139.12345)<0.00001);
  const char* fields[]={"engine_rpm","vehicle_speed","coolant_temp","engine_load",
   "intake_air_temp","throttle_position","distance_traveled","odometer",
   "control_module_voltage","ambient_air_temp","dtc_count","dtc_codes"};
  for(const char* field:fields)assert(row.containsKey(field) && row[field].isNull());
  nowMs+=1000;loop();assert(sendCalls==1 && dataArray.size()==1);
  // Keep the initial fix and its acquisition time even after the fresh-fix window.
  nowMs+=bg770a::GnssSession::RetryMs+1;loop();assert(sendCalls==2);
  assert(StartupLocation==StartupLocationState::Upload && dataArray.size()==1);
  JsonDocument retry;assert(!deserializeJson(retry,SentPayloads.back()));
  assert(retry["data"][0]["gps_time"]==row["gps_time"]);
  assert(retry["data"][0]["latitude"]==row["latitude"]);
  ReplyChunks={"200 {\"ok\":true}"};
  nowMs+=31000;serviceCan();assert(!serviceStartupLocation());
  assert(sendCalls==3 && StartupLocation==StartupLocationState::Complete);
  assert(dataArray.size()==1 && StartupLocationDoc["data"].size()==0);
  std::cout<<"Startup GPS precedes queued OBD without a prior clock; failed ACK retains the acquired fix for bounded retries and success releases OBD\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-gps-only") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;sendResult=true;
  completeStartupGnss();assert(sendCalls==1 && dataArray.size()==0);
  assert(StartupLocation==StartupLocationState::Complete && socketOpens==1);
  assert(can.sent.empty() && powerOffs==0);
  // The regular cadence is measured from the acquisition start, after completion.
  nowMs=bg770a::GnssSession::RetryMs-1;serviceModem(true,true);
  assert(Services.gnss.modemIdle());
  nowMs=bg770a::GnssSession::RetryMs;serviceModem(true,true);
  assert(Services.gnss.state()==bg770a::GnssSession::State::Offline);
  std::cout<<"Startup GPS-only upload needs no OBD response; periodic GNSS waits ten minutes from initial acquisition\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-gps-no-fix") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;
  completeStartupGnss(false);
  assert(StartupLocation==StartupLocationState::Complete && sendCalls==0);
  assert(StartupLocationDoc["data"].size()==0 && dataArray.size()==0);
  assert(nowMs<bg770a::GnssSession::InitialAcquisitionMs+1000);
  assert(can.sent.size()>0 && Obd.waiting());
  assert(Serial.output.find("status=no_fix_timeout")!=std::string::npos);
  assert(logCount("[GPS-FIX]")==0 && logCount("[GPS-ACK]")==0);
  std::cout<<"Startup no-fix acquisition ends at its bound, restores LTE and resumes OBD without fabricated GPS\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-no-fix-periodic-gps") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;
  completeStartupGnss(false);
  assert(sendCalls==0 && StartupLocation==StartupLocationState::Complete);
  // No ECU ever replies and no upload has succeeded. The ten-minute attempt
  // must nevertheless acquire and send GPS through the independent document.
  CanReady=false;can.sent.clear();nowMs=bg770a::GnssSession::RetryMs;serviceCan();
  serviceModem(true,true);assert(StartupLocation==StartupLocationState::Acquiring);
  sendResult=true;completeStartupGnss();
  assert(sendCalls==1 && StartupLocation==StartupLocationState::Complete);
  assert(dataArray.size()==0 && StartupLocationDoc["data"].size()==0);
  JsonDocument periodic;assert(!deserializeJson(periodic,SentPayloads.back()));
  assert(periodic["data"][0]["gps_time"]=="2026-11-06T12:34:56.789+00:00");
  assert(periodic["data"][0]["engine_rpm"].isNull());
  std::cout<<"A bounded no-fix startup still permits ten-minute GPS-only acquisition/upload with no ECU reply or prior successful upload\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-no-fix-no-network") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;
  completeStartupGnss(false,false);
  assert(StartupLocation==StartupLocationState::Complete && sendCalls==0);
  assert(Services.atIdle() && !Services.idle());
  assert(Obd.waiting() && can.sent.size()>0);
  assert(dataArray.size()==0 && StartupLocationDoc["data"].size()==0);
  std::cout<<"No-fix startup releases CAN collection even while LTE registration is unavailable\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-gps-recovery") {
  Services.ntpEnabled=false;Services.gnssEnabled=true;sendResult=true;
  loop();startupReply("AT+CFUN=4");startupReply("AT+QGPS=1");
  nowMs+=1000;loop();
  assert(dispatchServiceLine("AT+QGPSLOC=2"));
  assert(dispatchServiceLine("+QGPSLOC: 123456.789,35.12345,139.12345,1.2,12.0,3,0.00,0.0,0.0,061126,08"));
  assert(dispatchServiceLine("OK"));
  assert(StartupLocation==StartupLocationState::Upload && StartupLocationDoc["data"].size()==1);
  // Dispatch and fail the stop before the outer loop can snapshot a live fix.
  serviceModem(true,false);
  // A failed GNSS stop clears the session fix and requires physical recovery.
  assert(dispatchServiceLine("AT+QGPSEND"));assert(dispatchServiceLine("ERROR"));loop();
  assert(WioCellular.uart.resets==1 && !Services.gnss.fix().valid && sendCalls==0);
  assert(StartupLocation==StartupLocationState::Upload && StartupLocationDoc["data"].size()==1);
  nowMs+=2000;assert(dispatchServiceLine("APP RDY"));nowMs+=2200;loop();
  startupReply("AT+IFC=2,2");nowMs+=2200;loop();
  startupReply("AT+CFUN=1");startupReply("AT+CEREG=1");
  assert(dispatchServiceLine("AT+CEREG?"));
  assert(dispatchServiceLine("+CEREG: 1,1"));assert(dispatchServiceLine("OK"));loop();
  assert(sendCalls==1 && StartupLocation==StartupLocationState::Complete);
  JsonDocument recovered;assert(!deserializeJson(recovered,SentPayloads.back()));
  assert(recovered["data"][0]["gps_time"]=="2026-11-06T12:34:56.789+00:00");
  assert(recovered["data"][0]["engine_rpm"].isNull());
  std::cout<<"Startup GPS snapshot survives GNSS stop failure and modem reset, then sends its original acquisition time after LTE recovery\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="startup-ntp-first") {
  Services.ntpEnabled=true;Services.gnssEnabled=true;
  loop();assert(StartupLocation==StartupLocationState::Pending);
  assert(WioCellular.uart.tx.find("AT+QNTP=")!=std::string::npos);
  assert(WioCellular.uart.tx.find("AT+CFUN=4")==std::string::npos);
  assert(dispatchServiceLine(Services.ntp.command()));assert(dispatchServiceLine("OK"));loop();
  assert(WioCellular.uart.tx.find("AT+CFUN=4")==std::string::npos);
  assert(dispatchServiceLine("+QNTP: 0,\"2026/11/06,12:34:50+00\""));loop();
  assert(StartupLocation==StartupLocationState::Acquiring);
  assert(WioCellular.uart.tx.find("AT+CFUN=4")!=std::string::npos && sendCalls==0);
  std::cout<<"Startup GNSS waits for validated NTP ownership to finish before taking the modem\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="bounded-send-prefix") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  for(unsigned n=0;n<8;++n){initializeVehicleDataSchema();vehicleData["engine_rpm"]=1000+n;}
  sendResult=true;ReplyChunks={"400 {\"error\":\"Invalid payload shape\"}"};
  assert(!cellularSend(JsonDoc) && LastSentRecords==0 && dataArray.size()==8);
  JsonDocument failed;assert(!deserializeJson(failed,SentPayloads.back()));
  assert(SentPayloads.back().size()<=CELLULAR_PAYLOAD_LIMIT && failed["data"].size()<8);
  ReplyChunks={"200 {\"ok\":true}"};
  unsigned remaining=8,expected=1000;
  while(remaining) {
   assert(cellularSend(JsonDoc) && LastSentRecords>0 && LastSentRecords<=remaining);
   JsonDocument batch;assert(!deserializeJson(batch,SentPayloads.back()));
   assert(SentPayloads.back().size()<=CELLULAR_PAYLOAD_LIMIT);
   for(JsonObjectConst row:batch["data"].as<JsonArrayConst>())assert(row["engine_rpm"]==expected++);
   remaining-=LastSentRecords;clearSentData(LastSentRecords);
   assert(dataArray.size()==remaining && vehicleData.isNull());
  }
  assert(expected==1008);
  std::cout<<"Actual sender bounds each complete JSON batch and removes only accepted record prefixes; 400 retains every record\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="bounded-backlog-drain") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  for(unsigned n=0;n<8;++n){initializeVehicleDataSchema();vehicleData["engine_rpm"]=1000+n;}
  sendResult=true;nowMs=30001;loop();pollUntilSend(1);
  assert(dataArray.size()>0 && dataArray.size()<8);
  const auto firstSentAt=nowMs;
  for(unsigned n=0;n<400 && sendCalls<2;++n){nowMs+=10;loop();}
  assert(sendCalls==2 && uint32_t(nowMs-firstSentAt)<5000);
  assert(dataArray.size()>0 && dataArray.size()<7);
  std::cout<<"Accepted partial batch schedules the next prefix promptly instead of waiting 30s per queued record\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="registration-resumes-send") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=2000;sendResult=true;
  Services.line("+CEREG: 0",1000);nowMs=30000;Services.tick(nowMs);
  assert(Services.gnss.takeCommand()=="AT+CEREG?\r");
  // Complete a negative query before the send deadline, leaving an idle AT
  // boundary but registration still pending.
  Services.line("AT+CEREG?",nowMs);Services.line("+CEREG: 1, 2",nowMs);Services.line("OK",nowMs);
  nowMs=30001;
  for(unsigned n=0;n<50 && Serial.output.find("registration_pending")==std::string::npos;++n){loop();nowMs+=10;}
  assert(sendCalls==0 && dataArray.size()==1 && !Services.idle());
  assert(Serial.output.find("registration_pending")!=std::string::npos);
  // Recovery can be confirmed by a query alone, without a new network URC.
  nowMs+=5000;Services.tick(nowMs);assert(Services.gnss.takeCommand()=="AT+CEREG?\r");
  Services.line("AT+CEREG?",nowMs);Services.line("+CEREG: 1, 5",nowMs);Services.line("OK",nowMs);
  WioNetwork.state=Network::NetworkState::Offline;
  assert(Services.idle() && cellularOnline());
  for(unsigned n=0;n<50 && sendCalls==0;++n){loop();nowMs+=10;}
  assert(sendCalls==1 && dataArray.size()==0);
  std::cout<<"Registration deferral stays visible and due; whitespace-tolerant confirmed query resumes sending without network URC\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="cellular-network-wait") {
  allowStartupReadWait=true;
  WioNetwork.state=Network::NetworkState::Offline;
  can.rx.push_back({0x123,{2,1,0x0C,0,0,0,0,0}});
  assert(!waitForCellularNetwork(11000));
  assert(nowMs>=11000 && nowMs<=11010 && CellularReady && CanStats.received==1);
  size_t count=0,pos=0;while((pos=Serial.output.find("[LTE-WAIT]",pos))!=std::string::npos){++count;++pos;}
  assert(count==3 && Serial.output.find("[LTE-TIMEOUT]")!=std::string::npos);
  assert(WioCellular.uart.tx.empty() && httpCalls==0 && identityCalls==0);
  WioNetwork.state=Network::NetworkState::Connected;
  assert(waitForCellularNetwork(11000));
  assert(Serial.output.find("[LTE-READY]")!=std::string::npos);
  std::cout<<"Actual LTE wait reports every 5s, services CAN, returns on timeout without abort, and accepts later connectivity\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="cellular-unavailable") {
  Services.ntpEnabled=true;Services.gnssEnabled=true;CellularReady=false;
  const auto before=WioCellular.uart.tx;
  char value[64];assert(!getTime(value) && httpCalls==0);
  initializeVehicleDataSchema();assert(!cellularSend(JsonDoc) && socketOpens==0);
  nowMs=10000;loop();assert(Obd.waiting() && !can.sent.empty());
  assert(identityCalls==0 && WioCellular.uart.tx==before);
  nowMs=60000;Obd=bg770a::ObdSession(false,true);uint8_t pid;assert(Obd.next(nowMs,pid));
  assert(Obd.waiting());loop();assert(cellularInitCalls==0);
  nowMs+=201;loop();assert(cellularInitCalls==1);
  nowMs+=10;loop();assert(cellularInitCalls==1);
  assert(powerOffs==0 && identityCalls==0 && httpCalls==0);
  assert(WioCellular.uart.tx==before);
  std::cout<<"Unavailable cellular leaves CAN polling active, suppresses modem work and sleep, and retries only in an idle gap after 60s\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="synthetic-padded-values") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));
  uint8_t pid;assert(Obd.next(0,pid));
  const uint8_t map[]={6,0x41,0,0,0x10,0,0,0};Obd.accept(0x7E8,map,8,1);
  assert(!Obd.next(101,pid));assert(Obd.next(102,pid) && pid==0x0C);
  // Synthetic extension tests, not responses from the supplied vehicle log.
  can.rx.push_back({0x7E8,{7,0x41,0x0C,0x1F,0x40,0xFF,0xAA,0x55}});
  nowMs=109;serviceCan();assert(vehicleData["engine_rpm"]==2000);
  assert(vehicleData["vehicle_speed"].isNull() && CanStats.telemetry==1);
  sendDtcRequest();can.rx.push_back({0x7E8,{7,0x43,1,0x01,0xAF,0xFF,0xFF,0xFF}});
  serviceCan();assert(vehicleData["dtc_codes"]=="P01AF" && vehicleData["dtc_count"]==1);
  std::cout<<"Synthetic padded RPM/DTC decode only required data; padding never adds values/codes\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="field-padded") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  nowMs=19750;loop();assert(Obd.waiting() && Obd.pendingPid()==0);
  // Exact supplied field response; no other vehicle bytes are inferred.
  can.rx.push_back({FIELD_ID,{FIELD_BYTES}});
  nowMs=FIELD_MILLIS;loop();assert(CanStats.bitmaps==1 && Obd.selected());
  assert(Obd.support(0x0C)==bg770a::Support::Supported);
  assert(dataArray.size()==0); // bitmap/padding never becomes a measurement
  assert(Serial.output.find("ACCEPT bitmap")!=std::string::npos);
  std::cout<<"Exact fixed-length field bitmap accepted within deadline; padding not recorded\n";return 0;
 }
 if(argc>1 && (std::string(argv[1])=="utc-continuity" || std::string(argv[1])=="utc-true-wrap")) {
  const bool wrap=std::string(argv[1])=="utc-true-wrap";
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  nowMs=wrap?0xFFFFFF00U:19740;advanceMillis=true;
  int64_t anchor;assert(bg770a::parseHttpTime("2026-11-05T06:38:43+00:00",anchor));
  const uint32_t start=millis();Services.clock.seedFallback(anchor,Services.now(start));
  for(unsigned n=0;n<40;++n)loop();
  const uint32_t current=millis();int64_t epoch;
  assert(Services.clock.utc(Services.now(current),epoch));
  const int64_t wall=uint32_t(current-start)/1000;
  assert(epoch-anchor>=wall-1 && epoch-anchor<=wall+1);
  if(wrap)assert(current<start);
  assert(Serial.output.find("2026-12-")==std::string::npos);
  std::cout<<"Actual verbose sketch UTC follows advancing millis, with true wrap="<<wrap<<"\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="verbose-diagnostics") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  for(unsigned i=0;i<80;++i)can.rx.push_back({0x123,{2,1,0x0C,0,0,0,0,0}});
  for(unsigned i=0;i<10;++i)serviceCan();
  assert(CanStats.received==80 && CanStats.dropped[2]==80 && CanStats.suppressed==16);
  size_t count=0,pos=0;while((pos=Serial.output.find("[CAN-RX]",pos))!=std::string::npos){++count;++pos;}
  assert(count==64);canSummary();
  const uint8_t request[]={2,1,0x0C,0,0,0,0,0};
  traceFrame("CAN-TX-UART",0x7DF,request,"enqueue only");
  traceFrame("CAN-RX",0x7E8,request,"ACCEPT synthetic formatter fixture");
  assert(Serial.output.find("[0 ms]")!=std::string::npos);
  assert(Serial.output.find("raw_lines_suppressed=16")!=std::string::npos);
  assert(Serial.output.find("id=0x000007DF bytes=02 01 0C 00 00 00 00 00")!=std::string::npos);
  assert(Serial.output.find("ACCEPT synthetic formatter fixture")!=std::string::npos);
  assert(Serial.output.find("fixture-device")==std::string::npos);
  std::cout<<"Default verbose log timestamps/raw bytes/drop reasons/totals and overload suppression verified\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="uart-failure") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;loop();can.sendOK=false;
  nowMs=10000;loop();assert(!CanReady && CanStats.queueFailures==1);
  const auto count=can.sent.size();nowMs=11000;loop();assert(can.sent.size()==count);
  assert(Serial.output.find("CAN-UART-FAILED")!=std::string::npos);
  std::cout<<"Partial/failed UART queueing stops further requests and reports unconfirmed framing\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="pending-socket") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=1234;sendResult=true;socketDelay=500;
  nowMs=30001;loop();assert(Obd.waiting() && socketOpens==0);
  assert(!cellularSend(JsonDoc) && socketOpens==0);
  can.rx.push_back({0x7E8,{6,0x41,0,0,0x18,0,0,0}});
  nowMs=30060;loop();assert(Obd.selected() && socketOpens==1);
  assert(!Obd.waiting());
  assert(Serial.output.find("LTE-DEFER")!=std::string::npos);
  assert(Serial.output.find("id=0x000007E8 bytes=06 41 00")!=std::string::npos);
  assert(Serial.output.find("ACCEPT bitmap")!=std::string::npos);
  assert(Serial.output.find("fixture-device")==std::string::npos);
  std::cout<<"Pending OBD response defers synchronous TCP until reply processed, including direct sender guard\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="pending-clock-identity") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;loop();
  const auto identities=identityCalls;deviceIMSI[0]=0;
  nowMs=60000;uint8_t pid;assert(Obd.next(nowMs,pid));
  nowMs=60001;Services.ntpEnabled=true;const auto writes=WioCellular.uart.tx;
  loop();assert(WioCellular.uart.tx==writes);Services.ntpEnabled=false;
  assert(identityCalls==identities && httpCalls==0);
  char value[64];assert(!getTime(value) && httpCalls==0);
  can.rx.push_back({0x7E8,{6,0x41,0,0,0x18,0,0,0}});
  nowMs=60050;loop();assert(identityCalls==identities+1 && httpCalls==1);
  std::cout<<"Pending OBD response defers identity/time synchronous calls, which resume after reply\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="missing-bitmap") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  nowMs=9900;loop();assert(can.sent.empty());
  bool rpmRequested=false;
  for(nowMs=10000;nowMs<14000 && dataArray.size()==0;nowMs+=10) {
   loop();if(!can.sent.empty() && can.sent.back()[2]==0x0C && Obd.waiting()) {
    rpmRequested=true;assert(Obd.support(0x0C)==bg770a::Support::Unknown);
    can.rx.push_back({0x7E8,{4,0x41,0x0C,0x1F,0x40,0,0,0}});
   }
  }
  assert(rpmRequested);
  assert(dataArray.size()>0 && vehicleData["engine_rpm"]==2000);
  assert(Obd.support(0x0C)==bg770a::Support::Supported);
  std::cout<<"Missing bitmap recovers validated legacy telemetry after bounded retries\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="empty-and-400") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  sendResult=true;nowMs=30001;loop();assert(sendCalls==0 && socketOpens==0);
  assert(!cellularSend(JsonDoc));assert(socketOpens==0);
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=2000;
  ReplyChunks={"4","00 {\"error\":\"Invalid payload shape\",\"details\":{\"formErrors\":[],\"fieldErrors\":{\"data\":[\"data must contain at least one record\"]}}}"};
  nowMs=61020;loop();pollUntilSend(1);assert(dataArray.size()==1 && sendCalls==1);
  ReplyChunks={"20","7 {\"status\":\"partial\"}"};
  nowMs=92040;loop();pollUntilSend(2);assert(dataArray.size()==1 && sendCalls==2);
  ReplyChunks={"20","0 {\"result\":\"ok\"}\r\n"};
  nowMs=123060;loop();pollUntilSend(3);assert(dataArray.size()==0 && sendCalls==3);
  std::cout<<"Actual sender skips empty data; fragmented 400/207 retain queue and 200 clears it\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="heartbeat-deferred") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  sendResult=true;nowMs=30001;loop();assert(sendCalls==0);
  nowMs=ACK_INTERVAL+61000;loop();
  for(unsigned n=0;n<400 && !powerOffs;++n){nowMs+=10;loop();}
  assert(HeartbeatPending && socketOpens==0 && powerOns==0 && powerOffs==1);
  assert(dataArray.size()==0);ModemSleeping=false;
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=2000;
  nowMs+=31000;loop();pollUntilSend(1);assert(!HeartbeatPending && sendCalls==1);
  std::cout<<"Empty heartbeat remains pending locally until a measured payload succeeds\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="config-failure") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  can.rateOK=false;initCAN();assert(!CanReady);loop();nowMs=10000;loop();assert(can.sent.empty());
  can.rateOK=true;can.maskOK=false;initCAN();assert(!CanReady);
  can.maskOK=true;can.filterOK=false;initCAN();assert(!CanReady);
  can.filterOK=true;initCAN();assert(CanReady);
  std::cout<<"Actual initCAN propagates rate/mask/filter failures; no unconfigured transmission\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="unsynced-wake") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;loop();
  ModemSleeping=true;WioNetwork.state=Network::NetworkState::Offline;
  nowMs=10000;loop();
  can.rx.push_back({0x7E8,{6,0x41,0,0,0x18,0,0,0}});
  nowMs=10020;loop();
  assert(powerOns==1 && !ModemSleeping);assert(dataArray.size()==0);
  ModemSleeping=true;nowMs=10200;loop();nowMs=10350;loop();
  can.rx.push_back({0x123,{4,0x41,0x0C,0x1F,0x40,0,0,0}});
  nowMs=10360;loop();assert(ModemSleeping && powerOns==1);
  can.rx.push_back({0x7E8,{4,0x41,0x0C,0x1F,0x40,0,0,0}});
  nowMs=10380;loop();assert(!ModemSleeping && powerOns==2);
  assert(dataArray.size()==0);assert(std::string(Services.clock.source())=="unsynced");
  nowMs=11000;sendDtcRequest();ModemSleeping=true;
  can.rx.push_back({0x7E8,{4,0x43,1,0x01,0xAF,0,0,0}});
  serviceCan();assert(!DtcWaiting);loop();assert(powerOns==3);
  assert(dataArray.size()==0);
  std::cout<<"Unsynced capability/telemetry/DTC activity wakes modem without fabricated records\n";
  return 0;
 }
 if(argc>1 && std::string(argv[1])=="dtc-deadline") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));
  uint8_t pid;assert(Obd.next(0,pid));
  const uint8_t bitmap[]={6,0x41,0,0,0x18,0,0,0};
  assert(!Obd.accept(0x7E8,bitmap,8,1));
  sendDtcRequest();
  can.rx.push_back({0x123,{0,0,0,0,0,0,0,0},100});
  can.rx.push_back({0x124,{0,0,0,0,0,0,0,0},100});
  can.rx.push_back({0x7E8,{4,0x43,1,0x01,0xAF,0,0,0},1});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting);
  sendDtcRequest();can.rx.push_back({0x7E8,{4,0x43,1,0x01,0xAF,0,0,0},199});
  serviceCan();assert(dataArray.size()==1 && !DtcWaiting);clearData();HaveCanActivity=false;
  sendDtcRequest();can.rx.push_back({0x7E8,{4,0x43,1,0x01,0xAF,0,0,0},200});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting && !HaveCanActivity);
  sendDtcRequest();can.rx.push_back({0x7E8,{0x10,10,0x43,0,0,0,0,0},200});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting);
  nowMs=0xFFFFFFF0U;sendDtcRequest();
  can.rx.push_back({0x7E8,{4,0x43,1,0x01,0xAF,0,0,0},200});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting);
  std::cout<<"DTC receive-time deadline, blocking batches, SF/FF boundary and millis wrap passed\n";
  return 0;
 }

 Services.ntpEnabled=false;Services.gnssEnabled=false;
 Services.clock.seedFallback(1735689599LL,Services.now(0));
 auto handler=WioCellular.registerUrcHandler2(dispatchServiceLine);
 loop();assert(JsonDoc["imsi"]=="fixture-device");assert(dataArray.size()==0);
 nowMs=10000;loop();assert(can.sent.back()[2]==0);
 can.rx.push_back({0x123,{6,0x41,0,0,0x18,0,0,0}});
 nowMs=10020;loop();assert(!Obd.selected());assert(dataArray.size()==0);
 can.rx.push_back({0x7E8,{6,0x41,0,0,0x18,0,0,0}});
 nowMs=10040;loop();assert(Obd.selected());assert(dataArray.size()==0);
 nowMs=10200;loop();nowMs=10350;loop();assert(can.sent.back()[2]==0x0C);
 can.rx.push_back({0x7E9,{4,0x41,0x0C,0x1F,0x40,0,0,0}});
 can.rx.push_back({0x7E8,{3,0x41,0x0C,0x1F,0x40,0,0,0}});
 nowMs=10360;loop();assert(dataArray.size()==0);
 can.rx.push_back({0x7E8,{4,0x41,0x0C,0x1F,0x40,0,0,0}});
 nowMs=10380;loop();assert(dataArray.size()==1);
 assert(vehicleData["time_source"]=="http_fallback");
 assert(vehicleData["engine_rpm"]==2000);assert(vehicleData["time"]=="2025-01-01T00:00:09+00:00");
 nowMs=10500;loop();assert(can.sent.back()[2]==0x0D);
 can.rx.push_back({0x7E8,{3,0x41,0x0D,42,0,0,0,0}});
 nowMs=10520;loop();assert(vehicleData["vehicle_speed"]==42);assert(vehicleData["engine_rpm"]==2000);
 assert(vehicleData.containsKey("odometer"));assert(vehicleData.containsKey("dtc_codes"));
 Obd.tick(11000);nowMs=11000;sendDtcRequest();
 can.rx.push_back({0x7E9,{4,0x43,1,0x01,0xAF,0,0,0}});serviceCan();assert(DtcWaiting);
 can.rx.push_back({0x7E8,{4,0x43,1,0x01,0xAF,0,0,0}});serviceCan();assert(!DtcWaiting);
 assert(vehicleData["dtc_codes"]=="P01AF");assert(vehicleData["dtc_count"]==1);
 sendDtcRequest();can.rx.push_back({0x7E8,{8,0x43,0x01,0xAF,0,0,0,0}});serviceCan();assert(DtcWaiting);
 nowMs+=200;serviceCan();assert(!DtcWaiting);
 sendDtcRequest();can.rx.push_back({0x7E8,{2,0x43,0,0,0,0,0,0}});serviceCan();assert(vehicleData["dtc_count"]==0);
 sendDtcRequest();can.rx.push_back({0x7E8,{0x10,10,0x43,0,0,0,0,0}});serviceCan();assert(vehicleData["dtc_codes"]=="MULTI");assert(vehicleData["dtc_count"].isNull());
 nowMs=30001;loop();pollUntilSend(1);assert(sendCalls==1);assert(dataArray.size()==1); // failure retains samples
 sendResult=true;nowMs=61000;loop();pollUntilSend(2);assert(sendCalls==2);assert(dataArray.size()==0);assert(vehicleData.isNull());
 // Actual reset dispatch and off-state AT exclusion.
 Services.gnss.recover();serviceModem();assert(WioCellular.uart.resets==1);
 auto before=WioCellular.uart.tx;ModemSleeping=true;nowMs+=40000;serviceModem();assert(WioCellular.uart.tx==before);
 ModemSleeping=false;Services.gnss=bg770a::GnssSession{};
 SynchronousModem=true;Services.ntpEnabled=true;
 serviceModem();assert(WioCellular.uart.tx==before);SynchronousModem=false;Services.ntpEnabled=false;
 // Sample bound and UTC leap/year formatting are preserved on actual schema.
 nowMs=70000;
 for(unsigned n=0;n<20;++n) { initializeVehicleDataSchema(); }
 assert(dataArray.size()==12);
 clearData();assert(dataArray.size()==0 && vehicleData.isNull());
 attachFreshLocation();assert(JsonDoc["location"].isNull());
 assert(JsonDoc["obd"]["distance_traveled_meaning"]=="distance_since_dtc_clear");
 Services.~TelemetryServices();new(&Services)bg770a::TelemetryServices;
 Services.ntpEnabled=false;Services.gnssEnabled=false;
 char fetched[64];httpTime="2023-02-29T00:00:00+09:00";
 assert(!getTime(fetched));assert(std::string(Services.clock.source())=="unsynced");
 httpTime="2024-02-29T23:59:59.123+09:00";assert(getTime(fetched));
 int64_t epoch;assert(Services.clock.utc(Services.now(millis()),epoch));
 assert(bg770a::isoUtc(epoch)=="2024-02-29T14:59:59+00:00");
 assert(std::string(Services.clock.source())=="http_fallback");assert(!SynchronousModem);
 WioNetwork.state=Network::NetworkState::Offline;assert(!getTime(fetched));
 std::cout<<"Actual sketch loop, AT recovery, CAN validation, nullable schema, DTC, queue, HTTP fallback tests passed\n";
}
'''
source=source.replace('FIELD_ID',str(observed_rx['id'])).replace('FIELD_BYTES',','.join(str(n) for n in observed_rx['bytes'])).replace('FIELD_MILLIS',str(observed_rx['millis']))
include=pathlib.Path(os.environ.get('ARDUINOJSON_INCLUDE',root/'.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src'))
with tempfile.TemporaryDirectory() as tmp:
    cpp=pathlib.Path(tmp)/'sketch.cpp';exe=pathlib.Path(tmp)/'sketch';cpp.write_text(source)
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-Wno-deprecated-declarations','-pedantic','-fsanitize=undefined','-I'+str(root/'src'),'-I'+str(include),'-I'+str(root/'tests/gnss/fixtures/test-support'),'-I'+str(root/'tests/gnss/fixtures/upstream/src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
    for scenario in os.environ.get('SKETCH_SCENARIOS','gps-observability gps-upload-failures gps-payload-prefix gps-command-timeout-log qntp-utc-metadata zero-records startup-gps-delayed-fix startup-gps-first startup-gps-only startup-gps-no-fix startup-no-fix-periodic-gps startup-no-fix-no-network startup-gps-recovery startup-ntp-first bounded-send-prefix bounded-backlog-drain registration-resumes-send cellular-network-wait cellular-unavailable unsynced-wake dtc-deadline missing-bitmap empty-and-400 heartbeat-deferred config-failure pending-socket pending-clock-identity uart-failure verbose-diagnostics field-padded synthetic-padded-values utc-continuity utc-true-wrap').split():
        subprocess.run([str(exe),scenario],check=True)
