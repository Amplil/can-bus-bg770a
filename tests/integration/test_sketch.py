#!/usr/bin/env python3
"""Compile current sketch functions and loop against real JSON/AT dispatcher."""
import os, pathlib, re, subprocess, tempfile
root=pathlib.Path(__file__).resolve().parents[2]
sketch=(root/'examples/grove-can-bus-cellular/grove-can-bus-cellular.ino').read_text()
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
defines='\n'.join(re.findall(r'^#define PID_.*',headers,re.M))
constants=sketch[sketch.index('static constexpr int OBD_INTERVAL'):sketch.index('WioCAN can;')]
source=r'''
#include <ArduinoJson.h>
#include <telemetry-services.h>
#include <telemetry-time.h>
#include <obd-session.h>
#include <endpoint-ack.h>
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
uint32_t millis(){return nowMs;}
void delay(unsigned ms){nowMs+=ms;}
#include "module/at_client/AtClient.hpp"
struct Uart {
 std::deque<unsigned char> rx;std::string tx;unsigned resets=0;
 void waitReadAvailable(int t){assert(t==0);}
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
struct SerialType {explicit operator bool()const{return true;} template<class... T>void print(T...){} template<class... T>void println(T...){} template<class... T>void printf(T...){} } Serial;
struct Frame{unsigned long id;std::array<unsigned char,8> bytes;uint32_t delayMs=0;};
struct Can {
 std::deque<Frame> rx;std::deque<std::array<unsigned char,8>> sent;
 bool rateOK=true,maskOK=true,filterOK=true;
 void begin(){} bool setCanRate(unsigned){return rateOK;}
 bool setMask(unsigned,unsigned,unsigned long){return maskOK;}
 bool setFilt(unsigned,unsigned,unsigned long){return filterOK;}
 bool receive(unsigned long* id,unsigned char* data){if(rx.empty())return false;auto f=rx.front();rx.pop_front();nowMs+=f.delayMs;*id=f.id;std::copy(f.bytes.begin(),f.bytes.end(),data);return true;}
 bool send(unsigned long,unsigned char,unsigned char,unsigned char size,const unsigned char* bytes){assert(size==8);assert(bytes[1]==1||bytes[1]==3);std::array<unsigned char,8> f;std::copy(bytes,bytes+8,f.begin());sent.push_back(f);return true;}
}can;
static JsonDocument JsonDoc;
static JsonArray dataArray=JsonDoc["data"].to<JsonArray>();
static JsonObject vehicleData;
static char deviceIMSI[20]={};
static bg770a::TelemetryServices Services;
static bg770a::ObdSession Obd(false,true);
static bool CanReady=true,TelemetryAttempted=false,HaveCanActivity=false,ModemSleeping=false,DtcWaiting=false,SynchronousModem=false,HeartbeatPending=false;
static uint32_t DtcStarted=0;
static uint32_t LastCanActivity=0;
constexpr unsigned OBD_CAN_EXTENDED=0,OBD_ALLOW_UNKNOWN_PIDS=1,OBD_CAN_RATE=16;
constexpr unsigned PIN_VGROVE_ENABLE=0,VGROVE_ENABLE_OFF=0,VGROVE_ENABLE_ON=1;
static bool sendResult=false;static unsigned sendCalls=0,powerOffs=0,powerOns=0;
void digitalWrite(unsigned,unsigned){}
constexpr unsigned TELEMETRY_NTP=0;
static void serviceModem(bool,bool);
bool getTime(char*);
static std::string httpTime;
static bool getTimeFromWorldTimeApiHttp(char* out){
 assert(SynchronousModem);const auto before=WioCellular.uart.tx;
 serviceModem(true,true);assert(WioCellular.uart.tx==before);
 std::strncpy(out,httpTime.c_str(),63);out[63]=0;return !httpTime.empty();
}
bool getIMSI(char* out){std::strcpy(out,"fixture-device");return true;}
void initCAN();
void netPowerOffWait(){assert(Services.atIdle());++powerOffs;ModemSleeping=true;}
void netPowerOnRestart(){assert(Services.atIdle());++powerOns;Services.powerCycle();ModemSleeping=false;WioNetwork.state=Network::NetworkState::Connected;}
void clearData();
static bool cellularSend(const JsonDocument&);
static unsigned socketOpens=0;
static std::deque<std::string> ReplyChunks={"200 {}\r\n"};
using WioCellularModule=Module;
static constexpr const char* HOST="fixture.invalid";
static constexpr int PORT=23080,RECEIVE_TIMEOUT=10000;
static const char* WioCellularResultToString(int){return "fixture-status";}
template<class T>struct WioCellularTcpClient2 {
 std::deque<std::string> chunks=ReplyChunks;
 explicit WioCellularTcpClient2(T&){}
 bool open(int,const char*,int){++socketOpens;return true;}
 bool waitForConnect(){return true;}
 bool send(const char*,size_t){++sendCalls;return sendResult;}
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
for signature in ['static void serviceModem(', 'static void serviceCan()', 'static void cooperativeDelay(',
 'static void attachFreshLocation()', 'static bool cellularSend(const JsonDocument &doc)', 'void initCAN() {', 'String dtcToString(', 'void processDtcData(unsigned char* data)',
 'void updateVehicleData(unsigned char pid,', 'void clearData() {', 'static void initializeVehicleDataSchema() {',
 'String addMillisTime() {', 'bool getTime(char* time) {', 'bool arrayEmptyCheck(', 'void sendDtcRequest() {','void loop() {']:
    source+=function(signature)+'\n'
source+=r'''
int main(int argc,char** argv){
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
  nowMs=61020;loop();assert(dataArray.size()==1 && sendCalls==1);
  ReplyChunks={"20","7 {\"status\":\"partial\"}"};
  nowMs=92040;loop();assert(dataArray.size()==1 && sendCalls==2);
  ReplyChunks={"20","0 {\"result\":\"ok\"}\r\n"};
  nowMs=123060;loop();assert(dataArray.size()==0 && sendCalls==3);
  std::cout<<"Actual sender skips empty data; fragmented 400/207 retain queue and 200 clears it\n";return 0;
 }
 if(argc>1 && std::string(argv[1])=="heartbeat-deferred") {
  Services.ntpEnabled=false;Services.gnssEnabled=false;
  Services.clock.seedFallback(1735689599LL,Services.now(0));loop();
  sendResult=true;nowMs=30001;loop();assert(sendCalls==0);
  nowMs=ACK_INTERVAL+61000;loop();
  assert(HeartbeatPending && socketOpens==0 && powerOns==0 && powerOffs==1);
  assert(dataArray.size()==0);ModemSleeping=false;
  initializeVehicleDataSchema();vehicleData["engine_rpm"]=2000;
  nowMs+=31000;loop();assert(!HeartbeatPending && sendCalls==1);
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
  can.rx.push_back({0x7E8,{3,0x43,0x01,0xAF,0,0,0,0}});
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
  can.rx.push_back({0x7E8,{3,0x43,0x01,0xAF,0,0,0,0},1});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting);
  sendDtcRequest();can.rx.push_back({0x7E8,{3,0x43,0x01,0xAF,0,0,0,0},199});
  serviceCan();assert(dataArray.size()==1 && !DtcWaiting);clearData();HaveCanActivity=false;
  sendDtcRequest();can.rx.push_back({0x7E8,{3,0x43,0x01,0xAF,0,0,0,0},200});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting && !HaveCanActivity);
  sendDtcRequest();can.rx.push_back({0x7E8,{0x10,10,0x43,0,0,0,0,0},200});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting);
  nowMs=0xFFFFFFF0U;sendDtcRequest();
  can.rx.push_back({0x7E8,{3,0x43,0x01,0xAF,0,0,0,0},200});
  serviceCan();assert(dataArray.size()==0 && !DtcWaiting);
  std::cout<<"DTC receive-time deadline, blocking batches, SF/FF boundary and millis wrap passed\n";
  return 0;
 }

 Services.ntpEnabled=false;Services.gnssEnabled=false;
 Services.clock.seedFallback(1735689599LL,Services.now(0));
 auto handler=WioCellular.registerUrcHandler2([](const std::string& s){return Services.line(s,millis());});
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
 can.rx.push_back({0x7E9,{3,0x43,0x01,0xAF,0,0,0,0}});serviceCan();assert(DtcWaiting);
 can.rx.push_back({0x7E8,{3,0x43,0x01,0xAF,0,0,0,0}});serviceCan();assert(!DtcWaiting);
 assert(vehicleData["dtc_codes"]=="P01AF");assert(vehicleData["dtc_count"]==1);
 sendDtcRequest();can.rx.push_back({0x7E8,{8,0x43,0x01,0xAF,0,0,0,0}});serviceCan();assert(DtcWaiting);
 nowMs+=200;serviceCan();assert(!DtcWaiting);
 sendDtcRequest();can.rx.push_back({0x7E8,{7,0x43,0,0,0,0,0,0}});serviceCan();assert(vehicleData["dtc_count"]==0);
 sendDtcRequest();can.rx.push_back({0x7E8,{0x10,10,0x43,0,0,0,0,0}});serviceCan();assert(vehicleData["dtc_codes"]=="MULTI");
 nowMs=30001;loop();assert(sendCalls==1);assert(dataArray.size()==1); // failure retains samples
 sendResult=true;nowMs=61000;loop();assert(sendCalls==2);assert(dataArray.size()==0);assert(vehicleData.isNull());
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
 std::cout<<"Actual sketch loop, AT recovery, CAN validation, legacy schema, DTC, queue, HTTP fallback tests passed\n";
}
'''
include=pathlib.Path(os.environ.get('ARDUINOJSON_INCLUDE',root/'.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src'))
with tempfile.TemporaryDirectory() as tmp:
    cpp=pathlib.Path(tmp)/'sketch.cpp';exe=pathlib.Path(tmp)/'sketch';cpp.write_text(source)
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-Wno-deprecated-declarations','-pedantic','-fsanitize=undefined','-I'+str(root/'src'),'-I'+str(include),'-I'+str(root/'tests/gnss/fixtures/test-support'),'-I'+str(root/'tests/gnss/fixtures/upstream/src'),str(cpp),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
    for scenario in os.environ.get('SKETCH_SCENARIOS','unsynced-wake dtc-deadline missing-bitmap empty-and-400 heartbeat-deferred config-failure').split():
        subprocess.run([str(exe),scenario],check=True)
