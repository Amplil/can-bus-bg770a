#pragma once
#include "ntp-clock.h"

namespace ntp_candidate {
// Sole AT transaction owner while active. Use only via the module's dispatcher.
class NtpSession {
public:
  enum class Phase { Idle, Echo, Ack, Result, Quarantined, Disabled };
  static constexpr uint64_t AckDeadlineMs = 10000;
private:
  NtpClock& clock_;
  Phase phase_ = Phase::Idle;
  uint64_t started_ = 0;
  std::string command_;
  bool echoSeen_ = false, ackSeen_ = false;
  void reconcile() {
    if(clock_.state()==NtpClock::State::Quarantined) phase_=ackSeen_ ? Phase::Disabled : Phase::Quarantined;
    else if(clock_.state()==NtpClock::State::Idle) phase_=Phase::Idle;
  }
public:
  explicit NtpSession(NtpClock& clock) : clock_(clock) {}
  void resetBoundary() { clock_.resetBoundary(); phase_=Phase::Disabled; }
  Phase phase() const { return phase_; }
  bool modemIdle() const { return phase_==Phase::Idle || phase_==Phase::Disabled; }
  const std::string& command() const { return command_; }
  bool begin(uint64_t now, int context, const std::string& server) {
    if(!modemIdle() || context<1 || context>5 || server.empty() || server.size()>64) return false;
    for(char c:server) if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||
                         (c>='0'&&c<='9')||c=='.'||c=='-')) return false;
    if(!clock_.begin(now)) return false;
    command_="AT+QNTP="+std::to_string(context)+",\""+server+"\",123,1";
    started_=now; echoSeen_=false; ackSeen_=false; phase_=Phase::Echo;
    return true;
  }
  void tick(uint64_t now) {
    if((phase_==Phase::Echo||phase_==Phase::Ack) && now-started_>=AckDeadlineMs)
      clock_.uncertain();
    clock_.tick(now);
    reconcile();
  }
  bool line(const std::string& response, uint64_t now) {
    tick(now);
    const bool echo = response==command_ || response==command_+"\r";
    if(response.compare(0,7,"+QNTP: ")==0) {
      const bool handled=clock_.urc(response,now);
      reconcile(); return handled;
    }
    if(phase_==Phase::Quarantined) {
      // A matching late echo plus its final code establishes the command boundary.
      // NTP remains disabled for this boot; no new sample/request can be confused.
      if(echo) { echoSeen_=true; return true; }
      const bool finalCode=response=="OK" || response=="ERROR" ||
                           response.compare(0,12,"+CME ERROR: ")==0;
      if(echoSeen_ && finalCode) { ackSeen_=true; phase_=Phase::Disabled; return true; }
      return false;
    }
    if(phase_==Phase::Echo && echo) { echoSeen_=true; phase_=Phase::Ack; return true; }
    if(response=="OK" && (phase_==Phase::Echo || phase_==Phase::Ack)) {
      if(phase_==Phase::Echo) clock_.uncertain(); // Missing echo: cannot correlate safely.
      else { ackSeen_=true; phase_=Phase::Result; clock_.accepted(now); }
      reconcile(); return true;
    }
    if(phase_==Phase::Ack && (response=="ERROR" || response.compare(0,12,"+CME ERROR: ")==0)) {
      clock_.rejected(now); reconcile(); return true;
    }
    return false; // Preserve all unrelated URCs for existing library handlers.
  }
};
}
