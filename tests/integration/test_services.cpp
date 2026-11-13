#include "telemetry-services.h"
#include "telemetry-time.h"
#include <cassert>
#include <iostream>
#include <deque>
static uint32_t nowMs=0;
uint32_t millis(){return nowMs;}
#include "module/at_client/AtClient.hpp"
struct Uart {
 std::deque<unsigned char> rx;std::string tx;
 void waitReadAvailable(int t){assert(t==0);}
 int read(){if(rx.empty())return -1;auto c=rx.front();rx.pop_front();return c;}
 void write(int c){tx+=char(c);}
};
struct Module:wiocellular::module::at_client::AtClient<Module>{Uart uart;Uart& getInterface(){return uart;}};
int main(){
 for(bool fragmented:{false,true}) {
  bg770a::TelemetryServices s;Module module;nowMs=0;unsigned other=0;
  auto services=module.registerUrcHandler2([&](const std::string& l){return s.line(l,nowMs);});
  auto network=module.registerUrcHandler2([&](const std::string& l){if(l=="+CEREG: 1"){++other;return true;}return false;});
  auto inject=[&](const std::string& text){for(auto c:text){module.uart.rx.push_back(c);if(fragmented)module.doWork(0);}for(int n=0;n<16;++n)module.doWork(0);};
  auto command=s.command(nowMs=0,1,true,true,true);assert(command=="AT+QNTP=1,\"ntp.nict.jp\",123,1\r");
  assert(!s.idle());assert(s.gnss.modemIdle());
  assert(s.command(nowMs=1,1,true,true,true).empty());
  inject(command+"\r\n+CEREG: 1\r\nOK\r\n");assert(other==1);assert(!s.idle());
  assert(s.command(nowMs=2,1,true,true,true).empty()); // wait for QNTP result before GNSS
  inject("+QNTP: 0,\"2026/11/05,10:00:00+36\"\r\n");assert(s.idle());
  int64_t epoch;assert(s.clock.utc(s.now(2),epoch));assert(bg770a::isoUtc(epoch)=="2026-11-05T10:00:00+00:00");
  nowMs=3;assert(s.command(nowMs,1,true,true,true)=="AT+CFUN=4\r");assert(!s.idle());
  assert(s.command(nowMs=4,1,true,true,true).empty());
  inject("AT+CFUN=4\r\r\nOK\r\n");assert(s.command(nowMs=4,1,false,true,true)=="AT+QGPS=1\r");
  inject("AT+QGPS=1\r\r\nOK\r\n");nowMs=1004;
  assert(s.command(nowMs,1,false,true,true)=="AT+QGPSLOC=2\r");
  inject("AT+QGPSLOC=2\r\r\n+QGPSLOC: 123456.000,35.12345,139.12345,1.2,12.0,3,0.00,0.0,0.0,051126,08\r\nOK\r\n");
  assert(s.gnss.fix().valid);assert(s.command(nowMs,1,false,true,true)=="AT+QGPSEND\r");
  inject("AT+QGPSEND\r\r\nOK\r\n");assert(s.command(nowMs,1,false,true,true)=="AT+CFUN=1\r");
  inject("AT+CFUN=1\r\r\nOK\r\n");assert(s.command(nowMs,1,true,true,true)=="AT+CEREG=1\r");
  inject("AT+CEREG=1\r\r\nOK\r\n");assert(s.command(nowMs,1,true,true,true)=="AT+CEREG?\r");
  inject("AT+CEREG?\r\r\n+CEREG: 1,1\r\nOK\r\n");assert(s.idle());
  assert(s.command(nowMs,1,true,true,true).empty());
  s.powerCycle();assert(!s.gnss.fix().valid);assert(s.idle());
 }
 {bg770a::TelemetryServices s;assert(!s.command(nowMs=0,1,true,true,false).empty());s.tick(10000);
  assert(s.gnss.state()==bg770a::GnssSession::State::ResetNeeded);assert(!s.idle());
  s.gnss.resetIssued(10000);assert(s.command(nowMs=10001,1,true,true,true).empty());
  assert(s.line("+QNTP: 0,\"2026/11/05,00:00:00+00\"",10002));
  int64_t e;assert(!s.clock.utc(s.now(10002),e));}
 {bg770a::TelemetryServices s;s.clock.seedFallback(1762300800LL,s.now(0));
  assert(!s.command(0,1,true,true,false).empty());s.tick(10000);
  s.gnss.resetIssued(10000);s.line("APP RDY",10010);
  assert(s.command(12210,1,false,true,true)=="AT+IFC=2,2\r");
  s.line("AT+IFC=2,2",12210);s.line("OK",12210);
  assert(s.command(14410,1,false,true,true)=="AT+CFUN=1\r");
  s.line("AT+CFUN=1",14410);s.line("OK",14410);
  assert(s.command(14411,1,true,true,true)=="AT+CEREG=1\r");
  s.line("AT+CEREG=1",14411);s.line("OK",14411);
  assert(s.command(14412,1,true,true,true)=="AT+CEREG?\r");
  s.line("AT+CEREG?",14412);s.line("+CEREG: 1,1",14412);s.line("OK",14412);
  assert(s.idle());assert(s.command(20000,1,true,true,true).empty());
  int64_t epoch;assert(s.clock.utc(s.now(20000),epoch) && epoch==1762300820LL);
 }
 {bg770a::TelemetryServices s;s.ntpEnabled=false;s.gnssEnabled=false;
  assert(s.command(nowMs=0,1,true,true,true).empty());assert(s.idle());}
 int64_t e;assert(bg770a::parseHttpTime("2024-02-29T23:59:59.123+09:00",e));
 assert(bg770a::isoUtc(e+1)=="2024-02-29T15:00:00+00:00");
 assert(bg770a::parseHttpTime("2026-11-06T22:01:57+09:00",e));
 assert(bg770a::isoUtc(e)=="2026-11-06T13:01:57+00:00");
 assert(!bg770a::parseHttpTime("2023-02-29T00:00:00+09:00",e));
 assert(!bg770a::parseHttpTime("2024-01-01T00:00:00+99:99",e));
 assert(!bg770a::parseHttpTime("2024-01-01T00:00:00x+09:00",e));
 assert(bg770a::isoUtc(4102444799LL)=="2099-12-31T23:59:59+00:00");
 std::cout<<"Combined AT arbitration, real transport, recovery and UTC tests passed\n";
}
