#include <cassert>
#include <cstdint>
#include <deque>
#include <iostream>
#include <string>
#include <functional>
#include <utility>
#include "ntp-session.h"
static uint32_t fakeMs=0;
uint32_t millis() { return fakeMs; }
#include "module/at_client/AtClient.hpp"
struct FakeUart {
 std::deque<unsigned char> rx; std::string tx;
 void waitReadAvailable(int timeout) { assert(timeout==0); }
 int read() { if(rx.empty())return -1;auto c=rx.front();rx.pop_front();return c; }
 void write(int c) { tx+=char(c); }
 void inject(const std::string& s) { for(unsigned char c:s)rx.push_back(c); }
};
struct Module:wiocellular::module::at_client::AtClient<Module> {
 FakeUart uart; FakeUart& getInterface() { return uart; }
};
using WioCellularModule=Module;
Module WioCellular;
struct Network { enum class NetworkState { Connected, NotSearching }; struct Config { int pdpContextId=1; }config; bool online=true; NetworkState getNetworkState(){return online?NetworkState::Connected:NetworkState::NotSearching;} } WioNetwork;
using WioCellularNetwork=Network;
// LTE NTP, not NITZ. Seconds resolution is not an accuracy guarantee.
static const char NTP_SERVER[] = "time.cloudflare.com";
static ntp_candidate::Uptime64 NtpUptime;
static ntp_candidate::NtpClock NtpClock;
static ntp_candidate::NtpSession NtpSession{NtpClock};
using NtpHandler = decltype(std::declval<WioCellularModule&>().registerUrcHandler2(
    std::declval<std::function<bool(const std::string&)>>()));
static NtpHandler NtpUrcHandler;

static uint64_t monotonicNow() { return NtpUptime.update(millis()); }

static void serviceNtp(bool allowStart) {
  // Existing public API dispatches all lines to registered URC handlers.
  // Bound the number of lines handled per call; do not enter command wait loops.
  for (int i = 0; i < 4; ++i) WioCellular.doWork(0);
  const uint64_t now = monotonicNow();
  NtpSession.tick(now);
  if (!allowStart || !NtpSession.modemIdle() || !NtpClock.ready(now)) return;
  if (WioNetwork.getNetworkState() != WioCellularNetwork::NetworkState::Connected) return;
  if (!NtpSession.begin(now, WioNetwork.config.pdpContextId, NTP_SERVER)) return;
  const std::string wire = NtpSession.command() + "\r";
  WioCellular.writeBinary(wire.data(), wire.size());
  // No echo/OK wait here. Future doWork(0) calls handle echo, final result and URC.
  // UART transmit buffering/CTS still requires target-hardware verification.
}


int main() {
 int other=0;
 auto networkHandler=WioCellular.registerUrcHandler2([&](const std::string& s){if(s=="+CEREG: 1"){++other;return true;}return false;});
 NtpUrcHandler=WioCellular.registerUrcHandler2([](const std::string& s){return NtpSession.line(s,monotonicNow());});
 WioNetwork.online=false;serviceNtp(true);assert(WioCellular.uart.tx.empty());
 WioNetwork.online=true;serviceNtp(true);assert(!NtpSession.modemIdle());
 auto sent=WioCellular.uart.tx;assert(sent==NtpSession.command()+"\r");
 for(int i=0;i<20;++i) { serviceNtp(true); }
 assert(WioCellular.uart.tx==sent);
 WioCellular.uart.inject(NtpSession.command()+"\r\r\n+CEREG: 1\r\n+QNTP: 0,\"2026/11/04,00:00:00+00\"\r\nOK\r\n");
 serviceNtp(false);assert(NtpSession.modemIdle());assert(other==1);
 int64_t epoch;assert(NtpClock.utc(monotonicNow(),epoch));assert(epoch==1793750400);
 serviceNtp(true);assert(WioCellular.uart.tx==sent);
 fakeMs=21600000;serviceNtp(true);assert(WioCellular.uart.tx==sent+sent);
 fakeMs+=10000;serviceNtp(true);assert(!NtpSession.modemIdle());
 fakeMs+=100000;serviceNtp(true);assert(WioCellular.uart.tx==sent+sent);
 std::cout<<"Exact candidate serviceNtp integration passed with actual upstream AtClient\n";
}
