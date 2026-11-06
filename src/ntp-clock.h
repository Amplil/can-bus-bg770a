#pragma once
#include "ntp-time.h"

namespace ntp_candidate {
// Transport-independent state machine. All timestamps are monotonic milliseconds.
class NtpClock {
public:
  enum class State { Idle, Pending, Quarantined };
  static constexpr uint64_t TimeoutMs = 130000;
  static constexpr uint64_t RetryMs = 60000;
  static constexpr uint64_t ResyncMs = 6ULL * 60 * 60 * 1000;
  static constexpr uint64_t StaleMs = 24ULL * 60 * 60 * 1000;
private:
  State state_ = State::Idle;
  bool ntpSample_ = false;
  bool valid_ = false, ack_ = false, result_ = false, resultOk_ = false;
  uint64_t next_ = 0, start_ = 0, anchorMono_ = 0, receivedMono_ = 0;
  int64_t anchorEpoch_ = 0, receivedEpoch_ = 0;
  const char* error_ = "not_attempted";
  void finish(uint64_t now) {
    if (!ack_ || !result_) return;
    state_ = State::Idle;
    if (resultOk_) {
      valid_ = true; ntpSample_ = true; anchorEpoch_ = receivedEpoch_; anchorMono_ = receivedMono_;
      error_ = "none"; next_ = now + ResyncMs;
    } else { error_ = "ntp_result_invalid_or_failed"; next_ = now + RetryMs; }
  }
public:
  void seedFallback(int64_t epoch, uint64_t now) {
    if (valid_) return;
    valid_=true; ntpSample_=false; anchorEpoch_=epoch; anchorMono_=now;
  }
  void resetBoundary() { state_=State::Idle; next_=UINT64_MAX; }
  const char* source() const { return !valid_ ? "unsynced" : ntpSample_ ? "ntp" : "http_fallback"; }
  State state() const { return state_; }
  const char* lastError() const { return error_; }
  bool ready(uint64_t now) const { return state_ == State::Idle && now >= next_; }
  bool begin(uint64_t now) {
    if (!ready(now)) return false;
    state_=State::Pending; start_=now; ack_=false; result_=false; resultOk_=false;
    return true;
  }
  void accepted(uint64_t now) { if(state_==State::Pending) { ack_=true; finish(now); } }
  void rejected(uint64_t now) {
    if(state_!=State::Pending) return;
    state_=State::Idle; next_=now+RetryMs; error_="command_rejected";
  }
  void uncertain() {
    if(state_==State::Pending) { state_=State::Quarantined; error_="command_outcome_uncertain"; }
  }
  void tick(uint64_t now) {
    if(state_==State::Pending && now-start_>=TimeoutMs) {
      state_=State::Quarantined; error_="ntp_timeout";
    }
  }
  bool urc(const std::string& line, uint64_t now) {
    const std::string prefix="+QNTP: ";
    if(line.compare(0,prefix.size(),prefix)!=0) return false;
    tick(now); // Enforce deadline even if the UART delivers before the next poll.
    // Ignore unsolicited or duplicate notifications without changing the clock.
    if(state_==State::Idle || result_) return true;
    if(state_==State::Quarantined) {
      // A timed-out request has no identifier. Never use its late time sample.
      // Remain quarantined until reboot: a new request cannot be correlated safely.
      return true;
    }
    const std::string payload=line.substr(prefix.size());
    // Read-query responses start with a quote and must not terminate an NTP request.
    if(payload.empty() || payload[0]<'0' || payload[0]>'9') return true;
    result_=true; receivedMono_=now;
    resultOk_=payload.size()==26 && payload.compare(0,3,"0,\"")==0 &&
              payload.back()=='"' && parseQntpUtc(payload.substr(3,22),receivedEpoch_);
    finish(now);
    return true;
  }
  bool utc(uint64_t now, int64_t& epoch) const {
    if(!valid_ || now<anchorMono_) return false;
    epoch=anchorEpoch_+static_cast<int64_t>((now-anchorMono_)/1000);
    return true;
  }
  const char* quality(uint64_t now) const {
    if(!valid_) return "unsynced";
    return now-anchorMono_>=StaleMs ? "stale" : ntpSample_ ? "synced" : "fallback";
  }
  uint64_t ageSeconds(uint64_t now) const { return valid_ ? (now-anchorMono_)/1000 : 0; }
};
}
