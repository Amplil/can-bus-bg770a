#include "telemetry-services.h"
#include "telemetry-time.h"
#include <cassert>
#include <iostream>
int main(){
 ntp_candidate::Uptime64 u;
 // Exact problematic low-tick interleaving, with stale local capture in position 3.
 assert(u.update(19750)==19750);assert(u.update(19765)==19765);
 assert(u.update(19750)==19765);assert(u.latestTick()==19765);
 assert(u.update(19780)==19780);
 // The legacy unsigned-delta formula reproduces the supplied wrong UTC exactly.
 uint64_t legacy=19750;uint32_t previous=19750;
 for(uint32_t sample:{19765U,19750U,19780U}) {legacy+=uint32_t(sample-previous);previous=sample;}
 assert(legacy-u.value()==0x100000000ULL);
 int64_t anchor;assert(bg770a::parseHttpTime("2026-11-05T06:38:43+00:00",anchor));
 assert(bg770a::isoUtc(anchor+int64_t((legacy-19742)/1000))=="2026-12-24T23:41:30+00:00");
 for(uint32_t sample:{19950U,19951U,20056U,20071U,20056U,20072U}) {legacy+=uint32_t(sample-previous);previous=sample;}
 assert(bg770a::isoUtc(anchor+int64_t((legacy-19742)/1000))=="2027-02-12T16:44:17+00:00");
 uint64_t event=0;assert(u.project(19750,event) && event==19750);
 assert(u.value()==19780);assert(!u.project(19781,event));
 ntp_candidate::Uptime64 wrap;
 assert(wrap.update(0xFFFFFFF0U)==0xFFFFFFF0ULL);
 assert(wrap.update(0x20U)==0x100000020ULL);
 assert(wrap.update(0xFFFFFFF8U)==0x100000020ULL); // late pre-wrap sample
 assert(wrap.latestTick()==0x20U);
 assert(wrap.project(0xFFFFFFF8U,event) && event==0xFFFFFFF8ULL);
 assert(wrap.update(0x30U)==0x100000030ULL);
 assert(!wrap.project(0x31U,event));
 ntp_candidate::Uptime64 unknown;assert(!unknown.project(0,event));
 unknown.update(100);assert(!unknown.project(0xFFFFFFF0U,event));
 const auto saved=unknown.value();assert(unknown.update(100+0x80000000U)==saved);
 // Stale samples must not produce a fake NTP ACK timeout / resync.
 bg770a::TelemetryServices ntp;
 assert(!ntp.command(100,1,true,true,false).empty());ntp.now(105);ntp.tick(99);
 assert(ntp.gnss.state()==bg770a::GnssSession::State::Idle);
 assert(ntp.ntp.phase()==ntp_candidate::NtpSession::Phase::Echo);
 ntp.tick(10099);assert(ntp.gnss.state()==bg770a::GnssSession::State::Idle);
 ntp.tick(10100);assert(ntp.gnss.state()==bg770a::GnssSession::State::ResetNeeded);
 bg770a::TelemetryServices nw;assert(!nw.command(0xFFFFFFF0U,1,true,true,false).empty());
 nw.tick(0x26FFU);assert(nw.gnss.state()==bg770a::GnssSession::State::Idle);
 nw.tick(0x2700U);assert(nw.gnss.state()==bg770a::GnssSession::State::ResetNeeded);
 bg770a::TelemetryServices synced;
 assert(!synced.command(19420,1,true,true,false).empty());
 synced.line(synced.ntp.command(),19425);synced.line("OK",19428);
 synced.line("+QNTP: 0,\"2026/11/05,06:38:43+00\"",19742);
 synced.now(19750);synced.now(19765);
 assert(synced.command(19750,1,true,true,false).empty());
 synced.tick(19780);
 uint64_t historical=0;assert(synced.uptime.project(19750,historical));
 int64_t atEvent;assert(synced.clock.utc(historical,atEvent) && atEvent==anchor);
 assert(synced.uptime.project(19420,historical));assert(!synced.clock.utc(historical,atEvent));
 assert(std::string(synced.clock.quality(synced.uptime.value()))=="synced");
 assert(synced.ntp.phase()==ntp_candidate::NtpSession::Phase::Idle);
 // Canonical low tick also protects GNSS's unsigned raw-millis deadlines.
 bg770a::TelemetryServices gnss;gnss.ntpEnabled=false;
 assert(gnss.command(200,1,true,false,true)=="AT+CFUN=4\r");
 gnss.now(205);gnss.tick(199);
 assert(gnss.gnss.state()==bg770a::GnssSession::State::Offline);
 gnss.tick(20199);assert(gnss.gnss.state()==bg770a::GnssSession::State::Offline);
 gnss.tick(20200);assert(gnss.gnss.state()==bg770a::GnssSession::State::ResetNeeded);
 // Genuine GNSS deadline crossing a true uint32 wrap remains enforceable.
 bg770a::TelemetryServices gw;gw.ntpEnabled=false;
 assert(!gw.command(0xFFFFFFF0U,1,true,false,true).empty());
 gw.tick(0x4E0FU);assert(gw.gnss.state()==bg770a::GnssSession::State::Offline);
 gw.tick(0x4E10U);assert(gw.gnss.state()==bg770a::GnssSession::State::ResetNeeded);
 // Millisecond UTC is based on the UTC sample boundary, not raw millis%1000.
 bg770a::TelemetryServices ms;
 int64_t epochMs;
 assert(!ms.clock.utcMilliseconds(ms.now(0xFFFFFFF0U),epochMs));
 ms.clock.seedFallback(anchor,ms.now(0xFFFFFFF0U));
 assert(ms.clock.utcMilliseconds(ms.now(0x20U),epochMs) && epochMs==anchor*1000+48);
 assert(bg770a::isoUtcMilliseconds(epochMs)=="2026-11-05T06:38:43.048+00:00");
 assert(!ms.clock.utcMilliseconds(0xFFFFFFEFULL,epochMs));
 assert(ms.clock.begin(ms.now(0x30U)));ms.clock.accepted(ms.now(0x31U));
 assert(ms.clock.urc("+QNTP: 0,\"2026/11/05,06:38:42+36\"",ms.now(0x40U)));
 assert(ms.clock.utcMilliseconds(ms.now(0x45U),epochMs) && epochMs==anchor*1000-995);
 assert(bg770a::isoUtcMilliseconds(1735689599999LL)=="2024-12-31T23:59:59.999+00:00");
 assert(bg770a::isoUtcMilliseconds(1735689600000LL)=="2025-01-01T00:00:00.000+00:00");
 std::cout<<"Stale/live sample separation, first-high sample, real/late wrap, half-range contract and NTP/GNSS deadlines passed\n";
}
