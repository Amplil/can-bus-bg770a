#!/usr/bin/env python3
"""Compile actual pinned-main/current drivers; compare bytes and required JSON.
All responses/values here are synthetic host fixtures, never vehicle captures.
"""
import json, os, pathlib, re, subprocess, tempfile
root=pathlib.Path(__file__).resolve().parents[2]
BASE='93af60465fbfec11a852343c302180161a5ae251'
def baseline(path):
    return subprocess.check_output(['git','show',BASE+':'+path],cwd=root,text=True)
def function(text,signature):
    start=text.index(signature)
    while ';' in text[start:text.index('{',start)]:start=text.index(signature,start+len(signature))
    begin=text.index('{',start);depth=1;end=begin+1
    while depth:
        if text[end]=='{':depth+=1
        if text[end]=='}':depth-=1
        end+=1
    return text[start:end]
main_header=baseline('src/grove-can-bus.h')
main_driver=baseline('src/grove-can-bus.cpp')
main_sketch=baseline('examples/grove-can-bus-cellular/grove-can-bus-cellular.ino')
current_sketch=(root/'examples/grove-can-bus-cellular/grove-can-bus-cellular.ino').read_text()
constants={name:int(value,16) for name,value in re.findall(r'#define (PID_\w+)\s+(0x[0-9A-Fa-f]+)',main_header)}
names=re.search(r'obdPids\[\]\s*=\s*\{(.*?)\}',main_sketch,re.S).group(1)
names=[name.strip() for name in names.split(',') if name.strip()]
main_pids=[constants[name] for name in names]
current_pids=re.search(r'values\[\]\s*=\s*\{(.*?)\}',(root/'src/obd-session.h').read_text(),re.S).group(1)
assert [int(x,16) for x in current_pids.split(',')]==main_pids,'PID set/order differs from actual main'
loop=function(current_sketch,'void loop() {')
request=re.search(r'const unsigned char request\[8\]=(\{[^}]+\});',loop).group(1)
dtc_request=re.search(r'const unsigned char request\[8\]=(\{[^}]+\});',function(current_sketch,'void sendDtcRequest() {')).group(1)
defaults='\n'.join(re.search(r'#ifndef '+key+r'\n(.*?)#endif',current_sketch,re.S).group(1) for key in ['OBD_CAN_RATE','OBD_CAN_EXTENDED','OBD_ALLOW_UNKNOWN_PIDS'])
common=r'''
#include "grove-can-bus.h"
#include <obd-session.h>
#include <cassert>
#include <iostream>
#include <iomanip>
static uint32_t nowMs=0;
uint32_t millis(){return nowMs++;}
HardwareSerial Serial,Serial1;
static void dump(){for(auto byte:Serial1.tx)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(byte);std::cout<<"\n";Serial1.tx.clear();}
'''
include=pathlib.Path(os.environ.get('ARDUINOJSON_INCLUDE',root/'.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src'))
with tempfile.TemporaryDirectory() as folder:
    tmp=pathlib.Path(folder);base=tmp/'baseline';base.mkdir()
    (base/'grove-can-bus.h').write_text(main_header);(base/'grove-can-bus.cpp').write_text(main_driver)
    captures=[]
    for is_main,sketch,driver,header_path in [(True,main_sketch,base/'grove-can-bus.cpp',base),(False,current_sketch,root/'src/grove-can-bus.cpp',root/'src')]:
        source=common+'\n'+defaults+'\nint main(){WioCAN can;can.begin();\n'
        init=function(sketch,'void initCAN() {')
        for method,args in re.findall(r'can\.(setCanRate|setMask|setFilt)\(([^\n]*?)\)',init):source+='assert(can.'+method+'('+args+'));\n'
        source+='dump();\n'
        source+='const unsigned char pids[]={'+','.join(str(p) for p in [0,0x20,0x40,0x60,0x80,0xA0]+main_pids)+'};\n'
        if is_main:source+='for(auto pid:pids){can.sendPid(pid);dump();}\ncan.sendDtc(3);dump();\n'
        else:source+='bg770a::ObdSession session(OBD_CAN_EXTENDED);for(auto pid:pids){const unsigned char request[8]='+request+';can.send(session.requestId(),OBD_CAN_EXTENDED,0,8,request);dump();}\nconst unsigned char request[8]='+dtc_request+';can.send(session.requestId(),OBD_CAN_EXTENDED,0,8,request);dump();\n'
        source+='''// Synthetic host frame, not inferred from any vehicle log.
Serial1.rx={0,0,7,0xE8,4,0x41,0x0C,0x1F,0x40,0,0,0};unsigned long id=0;unsigned char data[8]={};
assert(can.receive(&id,data));for(int i=3;i>=0;--i)Serial1.tx.push_back((id>>(i*8))&255);for(auto byte:data)Serial1.tx.push_back(byte);dump();}
'''
        cpp=tmp/('main.cpp' if is_main else 'current.cpp');exe=cpp.with_suffix('');cpp.write_text(source)
        subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-fsanitize=undefined','-I'+str(header_path),'-I'+str(root/'src'),'-I'+str(root/'tests/driver/fixtures'),str(cpp),str(driver),'-o',str(exe)],check=True)
        captures.append(subprocess.check_output([str(exe)],text=True).splitlines())
    a,b=captures
    assert a[:-2]==b[:-2],'Default config commands or Mode 01 UART bytes differ from real main'
    assert a[-1]==b[-1] and len(bytes.fromhex(a[-1]))==12,'Actual driver receive framing differs'
    old,new=bytearray.fromhex(a[-2]),bytearray.fromhex(b[-2])
    assert old[6]==2 and new[6]==1 and old[:6]+old[7:]==new[:6]+new[7:],'Unexpected Mode 03 difference'
    schemas=[]
    for is_main,sketch in [(True,main_sketch),(False,current_sketch)]:
        source=r'''
#include <ArduinoJson.h>
#include <telemetry-services.h>
#include <cmath>
#include <iostream>
#include <string>
uint32_t millis(){return 1;}
static bg770a::TelemetryServices Services;
static JsonDocument JsonDoc;
static JsonArray dataArray=JsonDoc["data"].to<JsonArray>();
static JsonObject vehicleData;
static std::string addMillisTime(){return "2026-11-05T00:00:00+00:00";}
static void trace(const char*){}
struct Logger{template<class... T>void printf(T...){}} Serial;
'''
        source+='\n'.join(re.findall(r'^#define PID_.*',main_header,re.M))+'\n'
        source+=function(sketch,'static void initializeVehicleDataSchema() {')+'\n'+function(sketch,'void updateVehicleData(unsigned char pid,')
        source+='\nint main(){Services.clock.seedFallback(1793836800,1);JsonDoc["imsi"]="[synthetic-identity]";initializeVehicleDataSchema();\n'
        source+='const unsigned char pids[]={'+','.join(str(p) for p in main_pids)+'};\n'
        source+='for(auto pid:pids){unsigned char data[8]={static_cast<unsigned char>(2+bg770a::ObdSession::length(pid)),0x41,pid,0x20,0x40,0x60,0x80,0};updateVehicleData(pid,data);}\n'
        source+='std::string json;serializeJson(JsonDoc,json);std::cout<<json;}\n'
        # Main has no trace calls; keep shared mock declarations without warnings.
        source=source.replace('static void trace(const char*){}','void trace(const char*){}')
        source=source.replace('#include <telemetry-services.h>','#include <telemetry-services.h>\n#include <obd-session.h>')
        cpp=tmp/('schema-main.cpp' if is_main else 'schema-current.cpp');exe=cpp.with_suffix('');cpp.write_text(source)
        subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-fsanitize=undefined','-I'+str(root/'src'),'-I'+str(include),str(cpp),'-o',str(exe)],check=True)
        schemas.append(json.loads(subprocess.check_output([str(exe)],text=True)))
    a,b=schemas
    assert set(a)==set(b)=={'imsi','data'}
    old,new=a['data'][0],b['data'][0]
    assert all(key in new and new[key]==value and type(new[key]) is type(value) for key,value in old.items()),'Required schema/value/type drift against real main'
    assert set(new)-set(old)=={'time_source','time_sync_state','utc_epoch_s'}
print('Pinned main 93af604 versus current: all Mode01/config UART bytes, 12-byte RX, PID order and required JSON values/types match; sole Mode03 byte difference is PCI 02 -> 01. Synthetic host fixtures only.')
