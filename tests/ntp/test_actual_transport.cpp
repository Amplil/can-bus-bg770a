#include <cassert>
#include <cstdint>
#include <deque>
#include <iostream>
#include <string>
#include "ntp-session.h"
static uint32_t nowMs=0;
uint32_t millis() { return nowMs; }
#include "module/at_client/AtClient.hpp"

struct FakeUart {
 std::deque<unsigned char> incoming;
 std::string written;
 int waits=0;
 void waitReadAvailable(int timeout) { assert(timeout==0); ++waits; }
 int read() { if(incoming.empty()) return -1; auto c=incoming.front(); incoming.pop_front(); return c; }
 void write(int c) { written.push_back(static_cast<char>(c)); }
 void inject(const std::string& bytes) { for(unsigned char c:bytes) incoming.push_back(c); }
};
struct Module : wiocellular::module::at_client::AtClient<Module> {
 FakeUart uart;
 FakeUart& getInterface() { return uart; }
};
struct Harness {
 Module module;
 ntp_candidate::NtpClock clock;
 ntp_candidate::NtpSession session{clock};
 int otherUrc=0;
 using Handle=std::unique_ptr<Module::UrcHandler>;
 Handle ntp,other;
 Harness() {
   nowMs=0;
   ntp=module.registerUrcHandler2([this](const std::string& s){return session.line(s,nowMs);});
   other=module.registerUrcHandler2([this](const std::string& s){if(s=="+CEREG: 1" || s=="+QIURC: \"pdpdeact\",1") {++otherUrc;return true;} return false;});
 }
 void begin() {
   assert(session.begin(nowMs,1,"time.cloudflare.com"));
   auto bytes=session.command()+"\r";
   module.writeBinary(bytes.data(),bytes.size());
   assert(module.uart.written==bytes);
 }
 void pump(int calls=12) { for(int i=0;i<calls;++i) { module.doWork(0); session.tick(nowMs); } }
 bool synced() { int64_t epoch; return clock.utc(nowMs,epoch); }
};
const std::string ntp="+QNTP: 0,\"2026/11/04,00:00:00+00\"\r\n";
int main() {
 // Real upstream readResponse and processingUrc: CR echo, optional CRLF,
 // fragmented serial input, arbitrary network URCs, and all result orderings.
 for(int order=0;order<3;++order) {
   Harness h;h.begin();
   const auto echo=h.session.command()+"\r\r\n";
   if(order==0) h.module.uart.inject(ntp+"+CEREG: 1\r\n"+echo+"OK\r\n");
   if(order==1) h.module.uart.inject(echo+"+CEREG: 1\r\n"+ntp+"OK\r\n");
   if(order==2) h.module.uart.inject(echo+"OK\r\n+CEREG: 1\r\n"+ntp);
   h.pump();assert(h.synced());assert(h.otherUrc==1);assert(h.session.modemIdle());
 }
 { Harness h;h.begin();const auto echo=h.session.command()+"\r\r\n";
   for(char c:echo+"OK\r\n"+ntp) {h.module.uart.inject(std::string(1,c));h.pump(1);}
   h.pump();assert(h.synced()); }
 { Harness h;h.begin();h.module.uart.inject(h.session.command()+"\r\nOK\r\n");h.pump();
   assert(!h.synced());assert(!h.session.modemIdle());
   h.module.uart.inject("+QIURC: \"pdpdeact\",1\r\n"+ntp);h.pump();assert(h.otherUrc==1);assert(h.synced()); }
 { Harness h;h.begin();nowMs=10000;h.pump();assert(!h.session.modemIdle());
   h.module.uart.inject(h.session.command()+"\r\r\nOK\r\n"+ntp+"+CEREG: 1\r\n");h.pump();
   assert(!h.synced());assert(h.otherUrc==1);assert(h.session.modemIdle());assert(!h.clock.ready(nowMs)); }
 { Harness h;h.begin();h.module.uart.inject(h.session.command()+"\r\r\nOK\r\n");h.pump();
   nowMs=130000;h.pump();h.module.uart.inject(ntp);h.pump();assert(!h.synced());assert(h.session.modemIdle());assert(!h.clock.ready(nowMs)); }
 { Harness h;h.begin();h.module.uart.inject(h.session.command()+"\r\r\nERROR\r\n");h.pump();
   assert(h.session.modemIdle());assert(!h.synced());nowMs=60000;assert(h.clock.ready(nowMs)); }
 { Harness h;h.begin();h.module.uart.inject("OK\r\n"+ntp);h.pump();assert(!h.synced());assert(!h.session.modemIdle()); }
 { Harness h;assert(!h.session.begin(0,1,"host\"\rAT+RESET"));assert(h.session.modemIdle());assert(h.module.uart.written.empty()); }
 // Synthetic nonzero metadata replies pass through the real dispatcher without
 // applying that metadata to the UTC clock digits, even byte by byte.
 int64_t expected;assert(ntp_candidate::parse("2026/11/06,13:01:57+00",expected));
 for(const char* suffix:{"+36","-28"}) {
  Harness h;h.begin();
  const auto reply=h.session.command()+"\r\r\nOK\r\n+QNTP: 0,\"2026/11/06,13:01:57"+suffix+"\"\r\n";
  for(char c:reply) {h.module.uart.inject(std::string(1,c));h.pump(1);}
  h.pump();int64_t epoch;assert(h.clock.utc(nowMs,epoch) && epoch==expected);
 }
 std::cout<<"Actual upstream AtClient transport: response ordering, fragmentation and UTC timezone metadata passed; all read waits were zero.\n";
}
