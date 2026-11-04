#include <cassert>
#include <cstdint>
#include <deque>
#include <iostream>
#include <string>
#include "gnss-session.h"
static uint32_t nowMs = 0;
uint32_t millis() { return nowMs; }
#include "module/at_client/AtClient.hpp"
struct Uart {
  std::deque<unsigned char> rx;
  std::string tx;
  void waitReadAvailable(int timeout) { assert(timeout == 0); }
  int read() { if (rx.empty()) return -1; auto c=rx.front(); rx.pop_front(); return c; }
  void write(int c) { tx += char(c); }
};
struct Module : wiocellular::module::at_client::AtClient<Module> {
  Uart uart;
  Uart& getInterface() { return uart; }
};
int main() {
  for (bool fragmented : {false, true}) {
    Module module; bg770a::GnssSession session; int networkUrc = 0;
    auto gnss = module.registerUrcHandler2([&](const std::string& s) { return session.line(s,nowMs); });
    auto network = module.registerUrcHandler2([&](const std::string& s) {
      if(s.compare(0,8,"+CEREG: ")==0) { ++networkUrc; return true; } return false;
    });
    auto inject = [&](const std::string& text) {
      for(unsigned char c:text) { module.uart.rx.push_back(c); if(fragmented) module.doWork(0); }
      for(int i=0;i<12;++i) module.doWork(0);
    };
    auto exchange = [&](const std::string& command,const std::string& data="") {
      auto wire=session.takeCommand(); assert(wire==command+"\r");
      module.writeBinary(wire.data(),wire.size());
      // Real modem echo is command + CR CR LF, not a synthetic clean line.
      inject(command+"\r\r\n"+data+"OK\r\n");
    };
    nowMs=0; assert(session.begin(nowMs)); exchange("AT+CFUN=4"); exchange("AT+QGPS=1");
    nowMs=1000; session.tick(nowMs);
    exchange("AT+QGPSLOC=2", "+QGPSLOC: 123456.000,35.12345,139.12345,1.2,12.0,3,0.00,0.0,0.0,041126,08\r\n");
    assert(session.fix().valid); exchange("AT+QGPSEND"); exchange("AT+CFUN=1");
    exchange("AT+CEREG=1"); exchange("AT+CEREG?", "+CEREG: 1,1\r\n+CEREG: 1\r\n");
    assert(networkUrc==1); assert(session.canSend());
  }
  std::cout << "Actual WioCellular transport passed (fragmented and complete lines)\n";
}
