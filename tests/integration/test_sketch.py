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
struct Module:wiocellular::module::at_client::AtClient<Module>{Uart uart;Uart& getInterface(){return uart;}} WioCellular;
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
struct SerialType {template<class... T>void print(T...){} template<class... T>void println(T...){} } Serial;
struct Frame{unsigned long id;std::array<unsigned char,8> bytes;};
struct Can {
 std::deque<Frame> rx;std::deque<std::array<unsigned char,8>> sent;
 bool receive(unsigned long* id,unsigned char* data){if(rx.empty())return false;auto f=rx.front();rx.pop_front();*id=f.id;std::copy(f.bytes.begin(),f.bytes.end(),data);return true;}
 bool send(unsigned long,unsigned char,unsigned char,unsigned char size,const unsigned char* bytes){assert(size==8);assert(bytes[1]==1||bytes[1]==3);std::array<unsigned char,8> f;std::copy(bytes,bytes+8,f.begin());sent.push_back(f);return true;}
}can;
static JsonDocument JsonDoc;
static JsonArray dataArray=JsonDoc["data"].to<JsonArray>();
static JsonObject vehicleData;
static char deviceIMSI[20]={};
static bg770a::TelemetryServices Services;
static bg770a::ObdSession Obd;
static bool CanReady=true,TelemetryAttempted=false,HaveTelemetry=false,ModemSleeping=false,DtcWaiting=false,SynchronousModem=false;
static uint32_t DtcStarted=0;
constexpr unsigned OBD_CAN_EXTENDED=0;
constexpr unsigned PIN_VGROVE_ENABLE=0,VGROVE_ENABLE_OFF=0;
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
bool getIMSI(char* out){std::strcpy(out,"test-imsi");return true;}
void initCAN(){Obd=bg770a::ObdSession();}
void netPowerOffWait(){assert(Services.atIdle());++powerOffs;ModemSleeping=true;}
void netPowerOnRestart(){assert(Services.idle());++powerOns;Services.powerCycle();ModemSleeping=false;}
void clearData();
static bool cellularSend(const JsonDocument&){assert(Services.idle());++sendCalls;return sendResult;}
static void initializeVehicleDataSchema();
void updateVehicleData(unsigned char,unsigned char*);
void processDtcData(unsigned char*);
String addMillisTime();
void sendDtcRequest();
bool arrayEmptyCheck(JsonArrayConst);
'''+defines+'\n'+constants+'\n'
for signature in ['static void serviceModem(', 'static void serviceCan()', 'static void cooperativeDelay(',
 'static void attachFreshLocation()', 'String dtcToString(', 'void processDtcData(unsigned char* data)',
 'void updateVehicleData(unsigned char pid,', 'void clearData() {', 'static void initializeVehicleDataSchema() {',
 'String addMillisTime() {', 'bool getTime(char* time) {', 'bool arrayEmptyCheck(', 'void sendDtcRequest() {','void loop() {']:
    source+=function(signature)+'\n'
source+=r'''
int main(){
 Services.ntpEnabled=false;Services.gnssEnabled=false;
 Services.clock.seedFallback(1735689599LL,Services.now(0));
 auto handler=WioCellular.registerUrcHandler2([](const std::string& s){return Services.line(s,millis());});
 loop();assert(JsonDoc["imsi"]=="test-imsi");assert(dataArray.size()==0);
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
