#!/usr/bin/env python3
"""Compile the sketch's actual location serializer against real ArduinoJson."""
import os
import pathlib
import subprocess
import tempfile
root = pathlib.Path(__file__).resolve().parents[2]
include = pathlib.Path(os.environ.get('ARDUINOJSON_INCLUDE', root / '.pio/libdeps/seeed_wio_bg770a/ArduinoJson/src'))
if not (include / 'ArduinoJson.h').exists():
    raise SystemExit('ArduinoJson missing: run PlatformIO build or set ARDUINOJSON_INCLUDE')
sketch = (root / 'examples/grove-can-bus-cellular/grove-can-bus-cellular.ino').read_text()
function = sketch[sketch.index('static void attachFreshLocation() {'):sketch.index('\nstatic void abortHandler')]
source = r'''
#include <ArduinoJson.h>
#include <telemetry-services.h>
#include <cassert>
#include <string>
static uint32_t nowMs;
uint32_t millis() { return nowMs; }
static JsonDocument JsonDoc;
static bg770a::TelemetryServices Services;
auto& Gnss=Services.gnss;
''' + function + r'''
int main() {
  JsonDoc["imsi"]="test-imsi";
  JsonDoc["data"].to<JsonArray>().add<JsonObject>()["vehicle_speed"]=12;

  JsonDoc["location"]["latitude"]=123;
  attachFreshLocation(); assert(JsonDoc["location"].isNull());
  assert(Gnss.begin(0)); Gnss.takeCommand(); Gnss.line("AT+CFUN=4",0); Gnss.line("OK",0);
  Gnss.takeCommand(); Gnss.line("AT+QGPS=1",0); Gnss.line("OK",0);
  Gnss.tick(1000); Gnss.takeCommand(); Gnss.line("AT+QGPSLOC=2",1000);
  Gnss.line("+QGPSLOC: 123456.000,35.12345,139.12345,1.2,12.0,3,0.00,0.0,0.0,041126,08",1000);
  Gnss.line("OK",1000); nowMs=1000;
  attachFreshLocation(); assert(JsonDoc["location"]["latitude"].as<double>()==35.12345);
  assert(JsonDoc["location"]["source"]=="gnss");
  nowMs=61000; attachFreshLocation(); assert(!JsonDoc["location"].isNull());
  nowMs=121000; attachFreshLocation(); assert(!JsonDoc["location"].isNull());
  assert(JsonDoc["location"]["age_ms"]==120000);
  assert(JsonDoc["location"]["utc_hhmmss"]=="123456.000");
  assert(JsonDoc["location"]["date_ddmmyy"]=="041126");
  nowMs=601000; attachFreshLocation(); assert(!JsonDoc["location"].isNull());
  nowMs=601001; attachFreshLocation(); assert(JsonDoc["location"].isNull());
  std::string payload; serializeJson(JsonDoc,payload);
  assert(payload.find("location")==std::string::npos);
  assert(JsonDoc["data"].size()==1); assert(JsonDoc["data"][0]["vehicle_speed"]==12);
}
'''
with tempfile.TemporaryDirectory() as tmp:
    cpp=pathlib.Path(tmp)/'payload.cpp'; binary=pathlib.Path(tmp)/'payload'
    cpp.write_text(source)
    subprocess.run([os.environ.get('CXX','c++'),'-std=c++17','-Wall','-Wextra','-Werror','-fsanitize=undefined','-I'+str(root/'src'),'-I'+str(include),str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('Actual sketch location serializer / ArduinoJson tests passed')
