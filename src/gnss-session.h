#pragma once
#include "gnss-fix.h"
#include <string>
#include <cstring>

namespace bg770a {
// One AT owner. tick() never waits; the sketch dispatches lines via doWork(0).
class GnssSession {
public:
  enum class State { Idle, Offline, Start, PollWait, Poll, Stop, Online,
                     RestoreUrc, Registration, FlowControl, FlowSettle, ResetNeeded, BootWait, BootSettle };
  static constexpr uint32_t AcquisitionMs = 30000;
  static constexpr uint32_t RetryMs = 300000;
private:
  State state_ = State::Idle;
  uint32_t since_ = 0, acquisition_ = 0, lastAttempt_ = 0;
  bool attempted_ = false, disabled_ = false, pendingWrite_ = false;
  bool echo_ = false, registered_ = true, queryRegistered_ = false;
  uint32_t lastRegistrationPoll_ = 0;
  std::string command_, lastUtc_, lastDate_;
  GnssFix fix_{}, candidate_{};
  void command(State state, const char* text, uint32_t now) {
    state_ = state; command_ = text; since_ = now; echo_ = false; pendingWrite_ = true;
  }
  bool pending() const {
    return state_ == State::Offline || state_ == State::Start || state_ == State::Poll ||
           state_ == State::Stop || state_ == State::Online || state_ == State::RestoreUrc ||
           state_ == State::Registration || state_ == State::FlowControl;
  }
  void resetNeeded() {
    fix_ = GnssFix{}; candidate_ = GnssFix{}; registered_ = false;
    disabled_ = true; pendingWrite_ = false; state_ = State::ResetNeeded;
  }
  void finish(bool ok, uint32_t now) {
    switch (state_) {
    case State::Offline:
      if (ok) command(State::Start, "AT+QGPS=1", now);
      else { disabled_ = true; command(State::Online, "AT+CFUN=1", now); }
      break;
    case State::Start:
      if (ok) { state_ = State::PollWait; acquisition_ = since_ = now; }
      else { disabled_ = true; command(State::Stop, "AT+QGPSEND", now); }
      break;
    case State::Poll:
      if (ok && candidate_.valid &&
          (lastUtc_ != candidate_.utc || lastDate_ != candidate_.date)) {
        fix_ = candidate_; lastUtc_ = fix_.utc; lastDate_ = fix_.date;
        command(State::Stop, "AT+QGPSEND", now);
      } else if (uint32_t(now - acquisition_) >= AcquisitionMs) {
        command(State::Stop, "AT+QGPSEND", now);
      } else { state_ = State::PollWait; since_ = now; }
      candidate_ = GnssFix{};
      break;
    case State::Stop:
      // A rejected stop cannot establish that the GNSS engine is off.
      if (ok) command(State::Online, "AT+CFUN=1", now); else resetNeeded();
      break;
    case State::Online:
      if (ok) command(State::RestoreUrc, "AT+CEREG=1", now);
      else resetNeeded();
      break;
    case State::RestoreUrc:
      if (ok) { lastAttempt_ = now; queryRegistered_ = false;
        command(State::Registration, "AT+CEREG?", now); }
      else resetNeeded();
      break;
    case State::Registration:
      if (ok) { registered_ = queryRegistered_; state_ = State::Idle;
        lastRegistrationPoll_ = now; }
      else resetNeeded();
      break;
    case State::FlowControl:
      if (ok) { state_ = State::FlowSettle; since_ = now; }
      else resetNeeded();
      break;
    default: break;
    }
  }
public:
  void recover() { resetNeeded(); }
  State state() const { return state_; }
  bool modemIdle() const { return state_ == State::Idle; }
  bool canSend() const { return modemIdle() && registered_; }
  const GnssFix& fix() const { return fix_; }
  bool ready(uint32_t now) const {
    return !disabled_ && modemIdle() && registered_ &&
      (!attempted_ || uint32_t(now - lastAttempt_) >= RetryMs);
  }
  bool begin(uint32_t now) {
    if (!ready(now)) return false;
    attempted_ = true; lastAttempt_ = now; fix_ = GnssFix{};
    registered_ = false; command(State::Offline, "AT+CFUN=4", now); return true;
  }
  std::string takeCommand() {
    if (!pendingWrite_) return {};
    pendingWrite_ = false; return command_ + "\r";
  }
  // Call after physically resetting the modem. Never send AT across an unknown
  // timed-out transaction boundary. A hardware reset establishes a new boundary.
  void resetIssued(uint32_t now) { state_ = State::BootWait; since_ = now; }
  void tick(uint32_t now) {
    if (pending() && uint32_t(now - since_) >= 20000) { resetNeeded(); return; }
    if (state_ == State::BootWait && uint32_t(now - since_) >= 30000) {
      state_ = State::ResetNeeded; return;
    }
    if (state_ == State::BootSettle && uint32_t(now - since_) >= 2200) {
      command(State::FlowControl, "AT+IFC=2,2", now); return;
    }
    if (state_ == State::FlowSettle && uint32_t(now - since_) >= 2200) {
      command(State::Online, "AT+CFUN=1", now); return;
    }
    if (state_ == State::Idle && !registered_ &&
        uint32_t(now - lastRegistrationPoll_) >= 5000) {
      queryRegistered_ = false; command(State::Registration, "AT+CEREG?", now); return;
    }
    if (state_ == State::PollWait) {
      if (uint32_t(now - acquisition_) >= AcquisitionMs)
        command(State::Stop, "AT+QGPSEND", now);
      else if (uint32_t(now - since_) >= 1000)
        command(State::Poll, "AT+QGPSLOC=2", now);
    }
  }
  bool line(const std::string& text, uint32_t now) {
    // Observe but do not swallow registration notifications needed by WioNetwork.
    if (text.compare(0,8,"+CEREG: ") == 0) {
      const std::string value = text.substr(8);
      if (state_ == State::Registration && echo_ && value.find(',') != std::string::npos) {
        // Query response is <n>,<stat>; network URC parser expects <stat>.
        const auto comma = value.find(',');
        queryRegistered_ = comma != std::string::npos && comma + 1 < value.size() &&
          (value[comma + 1] == '1' || value[comma + 1] == '5') &&
          (comma + 2 == value.size() || value[comma + 2] == ',');
        return true;
      }
      registered_ = !value.empty() && (value[0] == '1' || value[0] == '5') &&
        (value.size() == 1 || value[1] == ',');
      if (state_ == State::Registration && echo_) queryRegistered_ = registered_;
      return false;
    }
    if (state_ == State::BootWait && text == "APP RDY") {
      state_ = State::BootSettle; since_ = now; return true;
    }
    if (!pending()) return false;
    if (text == command_ || text == command_ + "\r") { echo_ = true; return true; }
    if (state_ == State::Poll && echo_ && text.compare(0,10,"+QGPSLOC: ") == 0) {
      parseQgpsloc(text.c_str(), now, candidate_); return true;
    }
    const bool ok = text == "OK";
    const bool error = text == "ERROR" || text.compare(0,11,"+CME ERROR:") == 0;
    if (ok || error) {
      if (!echo_) resetNeeded(); else finish(ok, now);
      return true;
    }
    return false;
  }
};
}
