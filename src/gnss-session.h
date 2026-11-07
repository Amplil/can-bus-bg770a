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
  enum class Event { None, AcquisitionStarted, PollNoFix, PollError, InvalidResponse,
                     DuplicateFix, FixAcquired, AcquisitionTimeout, CommandError,
                     CommandTimeout, ReattachmentTimeout, Recovery };
  struct Diagnostic {
    Event event = Event::None;
    int cmeError = -1;
    State state = State::Idle;
    uint32_t sequence = 0, elapsedMs = 0;
    unsigned pollCount = 0;
  };
  static constexpr uint32_t AcquisitionMs = 30000;
  // An initial autonomous fix can take longer than the periodic search window.
  // Keep this budget until a valid fix is confirmed, including after no-fix retries.
  static constexpr uint32_t InitialAcquisitionMs = 60000;
  static constexpr uint32_t RetryMs = 600000; // Ten minutes between acquisition starts.
  static constexpr uint32_t ReattachmentMs = 120000;
private:
  State state_ = State::Idle;
  uint32_t since_ = 0, acquisition_ = 0, lastAttempt_ = 0;
  bool attempted_ = false, disabled_ = false, pendingWrite_ = false;
  bool hasAcquiredFix_ = false, resumeAfterRecovery_ = false;
  bool echo_ = false, registered_ = true, queryRegistered_ = false;
  uint32_t lastRegistrationPoll_ = 0;
  uint32_t reattachmentStarted_ = 0;
  bool awaitingReattachment_ = false;
  int registrationStatus_ = -1, queryRegistrationStatus_ = -1;
  std::string command_, lastUtc_, lastDate_;
  GnssFix fix_{}, candidate_{};
  Diagnostic diagnostic_{};
  unsigned pollCount_ = 0;
  int lastPollError_ = -1;
  void report(Event event, uint32_t now, int cmeError = -1) {
    diagnostic_.event=event;diagnostic_.cmeError=cmeError;
    diagnostic_.state=state_;++diagnostic_.sequence;
    diagnostic_.pollCount=pollCount_;
    switch(event) {
    case Event::AcquisitionStarted: diagnostic_.elapsedMs=0;break;
    case Event::PollNoFix: case Event::PollError: case Event::InvalidResponse:
    case Event::DuplicateFix: case Event::FixAcquired: case Event::AcquisitionTimeout:
      diagnostic_.elapsedMs=uint32_t(now-acquisition_);break;
    case Event::ReattachmentTimeout:
      diagnostic_.elapsedMs=uint32_t(now-reattachmentStarted_);break;
    default: diagnostic_.elapsedMs=uint32_t(now-since_);break;
    }
  }
  // Keep diagnostics bounded and numeric: modem-controlled error text is never
  // retained or exposed. A malformed numeric token is an unknown error (-1).
  static int cmeErrorCode(const std::string& text) {
    if(text.compare(0,11,"+CME ERROR:")!=0)return -1;
    size_t i=11;
    while(i<text.size() && (text[i]==' ' || text[i]=='\t'))++i;
    if(i==text.size() || text[i]<'0' || text[i]>'9')return -1;
    unsigned code=0;
    while(i<text.size() && text[i]>='0' && text[i]<='9') {
      code=code*10+unsigned(text[i++]-'0');
      if(code>65535)return -1;
    }
    while(i<text.size() && (text[i]==' ' || text[i]=='\t'))++i;
    return i==text.size() ? static_cast<int>(code) : -1;
  }
  static bool registrationToken(const std::string& text, size_t begin, size_t end, int& value) {
    while(begin<end && (text[begin]==' ' || text[begin]=='\t'))++begin;
    while(end>begin && (text[end-1]==' ' || text[end-1]=='\t'))--end;
    if(begin==end)return false;
    int parsed=0;
    for(size_t i=begin;i<end;++i) {
      if(text[i]<'0' || text[i]>'9')return false;
      parsed=parsed*10+text[i]-'0';
      if(parsed>255)return false;
    }
    value=parsed;return true;
  }
  void prepareRegistrationQuery(uint32_t now) {
    queryRegistered_=false;queryRegistrationStatus_=-1;
    command(State::Registration,"AT+CEREG?",now);
  }
  void command(State state, const char* text, uint32_t now) {
    state_ = state; command_ = text; since_ = now; echo_ = false; pendingWrite_ = true;
    if(state==State::Poll)++pollCount_;
  }
  bool pending() const {
    return state_ == State::Offline || state_ == State::Start || state_ == State::Poll ||
           state_ == State::Stop || state_ == State::Online || state_ == State::RestoreUrc ||
           state_ == State::Registration || state_ == State::FlowControl;
  }
  void confirmRegistration(uint32_t now) {
    if(!registered_ || !modemIdle())return;
    awaitingReattachment_=false;
    if(resumeAfterRecovery_) {
      // Only completed recovery commands can reach Idle here. An explicit
      // ERROR has a known AT boundary; uncertain outcomes still require reset.
      // A later registration URC is as valid as the initial successful query.
      disabled_=false;resumeAfterRecovery_=false;
      attempted_=true;lastAttempt_=now;
      report(Event::Recovery,now);
    }
  }
  void resetNeeded() {
    fix_ = GnssFix{}; candidate_ = GnssFix{}; registered_ = false;
    awaitingReattachment_ = false;
    disabled_ = true; pendingWrite_ = false; state_ = State::ResetNeeded;
  }
  bool expireAcquisition(uint32_t now) {
    if ((state_ != State::PollWait && state_ != State::Poll) ||
        uint32_t(now - acquisition_) < acquisitionTimeoutMs()) return false;
    report(Event::AcquisitionTimeout,now,lastPollError_);
    // A sent poll must not extend the search deadline or accept a late fix.
    // Reset establishes a fresh AT boundary when its final result is missing;
    // an unsent poll can safely be replaced by the normal stop command.
    if (state_ == State::Poll && !pendingWrite_) resetNeeded();
    else { candidate_ = GnssFix{}; command(State::Stop,"AT+QGPSEND",now); }
    return true;
  }
  void finish(bool ok, uint32_t now, int cmeError = -1) {
    if(!ok && state_!=State::Poll)report(Event::CommandError,now,cmeError);
    switch (state_) {
    case State::Offline:
      if (ok) command(State::Start, "AT+QGPS=1", now);
      else { disabled_ = true; resumeAfterRecovery_=true; command(State::Online, "AT+CFUN=1", now); }
      break;
    case State::Start:
      if (ok) { state_ = State::PollWait; acquisition_ = since_ = now; }
      else { disabled_ = true; resumeAfterRecovery_=true; command(State::Stop, "AT+QGPSEND", now); }
      break;
    case State::Poll:
      lastPollError_=ok ? -1 : cmeError;
      if (ok && candidate_.valid &&
          (lastUtc_ != candidate_.utc || lastDate_ != candidate_.date)) {
        fix_ = candidate_; lastUtc_ = fix_.utc; lastDate_ = fix_.date;
        hasAcquiredFix_ = true;
        report(Event::FixAcquired,now);
        command(State::Stop, "AT+QGPSEND", now);
      } else if (uint32_t(now - acquisition_) >= acquisitionTimeoutMs()) {
        report(Event::AcquisitionTimeout,now,lastPollError_);
        command(State::Stop, "AT+QGPSEND", now);
      } else {
        report(!ok ? Event::PollError : candidate_.valid ? Event::DuplicateFix : Event::PollNoFix,
               now,lastPollError_);
        state_ = State::PollWait; since_ = now;
      }
      candidate_ = GnssFix{};
      break;
    case State::Stop:
      // A rejected stop cannot establish that the GNSS engine is off.
      if (ok) command(State::Online, "AT+CFUN=1", now); else resetNeeded();
      break;
    case State::Online:
      if (ok) {
        reattachmentStarted_=now;awaitingReattachment_=true;
        command(State::RestoreUrc, "AT+CEREG=1", now);
      }
      else resetNeeded();
      break;
    case State::RestoreUrc:
      if (ok) { prepareRegistrationQuery(now); }
      else resetNeeded();
      break;
    case State::Registration:
      if (ok) { registered_ = queryRegistered_; state_ = State::Idle;
        registrationStatus_=queryRegistrationStatus_;
        confirmRegistration(now);
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
  void recover() { report(Event::Recovery,since_);resetNeeded(); }
  State state() const { return state_; }
  const Diagnostic& diagnostic() const { return diagnostic_; }
  bool disabled() const { return disabled_; }
  uint32_t acquisitionTimeoutMs() const {
    return hasAcquiredFix_ ? AcquisitionMs : InitialAcquisitionMs;
  }
  bool modemIdle() const { return state_ == State::Idle; }
  bool canSend() const { return modemIdle() && registered_; }
  int registrationStatus() const { return registrationStatus_; }
  const GnssFix& fix() const { return fix_; }
  bool ready(uint32_t now) const {
    return !disabled_ && modemIdle() && registered_ &&
      (!attempted_ || uint32_t(now - lastAttempt_) >= RetryMs);
  }
  bool begin(uint32_t now) {
    if (!ready(now)) return false;
    attempted_ = true; lastAttempt_ = now; fix_ = GnssFix{};
    pollCount_=0;lastPollError_=-1;
    registered_ = false; command(State::Offline, "AT+CFUN=4", now);
    report(Event::AcquisitionStarted,now);return true;
  }
  std::string takeCommand() {
    if (!pendingWrite_) return {};
    pendingWrite_ = false; return command_ + "\r";
  }
  // Call after physically resetting the modem. Never send AT across an unknown
  // timed-out transaction boundary. A hardware reset establishes a new boundary.
  void resetIssued(uint32_t now) {
    state_ = State::BootWait; since_ = now;resumeAfterRecovery_=true;
    registrationStatus_=-1;queryRegistrationStatus_=-1;awaitingReattachment_=false;
  }
  void tick(uint32_t now) {
    if (expireAcquisition(now)) return;
    if (pending() && uint32_t(now - since_) >= 20000) {
      report(Event::CommandTimeout,now);resetNeeded();return;
    }
    // Returning from GNSS must not leave queued telemetry waiting forever for
    // LTE reattachment. Unknown command outcomes require physical reset before retry.
    if(awaitingReattachment_ && !registered_ &&
       uint32_t(now-reattachmentStarted_)>=ReattachmentMs) {
      report(Event::ReattachmentTimeout,now);resetNeeded();return;
    }
    if (state_ == State::BootWait && uint32_t(now - since_) >= 30000) {
      report(Event::CommandTimeout,now);state_ = State::ResetNeeded;return;
    }
    if (state_ == State::BootSettle && uint32_t(now - since_) >= 2200) {
      command(State::FlowControl, "AT+IFC=2,2", now); return;
    }
    if (state_ == State::FlowSettle && uint32_t(now - since_) >= 2200) {
      command(State::Online, "AT+CFUN=1", now); return;
    }
    if (state_ == State::Idle && !registered_ &&
        uint32_t(now - lastRegistrationPoll_) >= 5000) {
      prepareRegistrationQuery(now); return;
    }
    if (state_ == State::PollWait) {
      if (uint32_t(now - since_) >= 1000)
        command(State::Poll, "AT+QGPSLOC=2", now);
    }
  }
  bool line(const std::string& text, uint32_t now) {
    // Observe but do not swallow registration notifications needed by WioNetwork.
    if (text.compare(0,8,"+CEREG: ") == 0) {
      const std::string value = text.substr(8);
      const auto comma=value.find(',');
      if (state_ == State::Registration && echo_ && comma != std::string::npos) {
        auto second=comma+1;
        while(second<value.size() && (value[second]==' ' || value[second]=='\t'))++second;
        // Queries contain <n>,<stat>; an interleaved extended URC contains
        // <stat>,"<tac>",... and must still reach WioNetwork's URC handler.
        if(second==value.size() || value[second]!='"') {
          int mode=-1,status=-1;
          const auto nextComma=value.find(',',comma+1);
          const bool valid=registrationToken(value,0,comma,mode) && mode<=5 &&
            registrationToken(value,comma+1,nextComma==std::string::npos?value.size():nextComma,status);
          queryRegistrationStatus_=valid?status:-1;
          queryRegistered_=valid && (status==1 || status==5);
          return true;
        }
      }
      int status=-1;
      if(!registrationToken(value,0,comma==std::string::npos?value.size():comma,status))return false;
      registrationStatus_=status;registered_=status==1 || status==5;
      confirmRegistration(now);
      if (state_ == State::Registration && echo_) {
        queryRegistered_=registered_;queryRegistrationStatus_=status;
      }
      return false;
    }
    if (state_ == State::BootWait && text == "APP RDY") {
      state_ = State::BootSettle; since_ = now; return true;
    }
    if (expireAcquisition(now)) return false;
    if (!pending()) return false;
    if (text == command_ || text == command_ + "\r") { echo_ = true; return true; }
    if (state_ == State::Poll && echo_ && text.compare(0,10,"+QGPSLOC: ") == 0) {
      if(!parseQgpsloc(text.c_str(), now, candidate_))report(Event::InvalidResponse,now);
      return true;
    }
    const bool ok = text == "OK";
    const bool error = text == "ERROR" || text.compare(0,11,"+CME ERROR:") == 0;
    if (ok || error) {
      const int code=error ? cmeErrorCode(text) : -1;
      if (!echo_) {report(Event::Recovery,now,code);resetNeeded();}
      else finish(ok, now,code);
      return true;
    }
    return false;
  }
};
}
