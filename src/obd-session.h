#ifndef BG770A_OBD_SESSION_H
#define BG770A_OBD_SESSION_H
#include <stdint.h>
#include <stddef.h>
namespace bg770a {
enum class Support { Unknown, Unsupported, Supported };
// Fixed standard Mode 01 telemetry only. No transport probing or control services.
class ObdSession {
 public:
  enum class Reject { None, NoPending, Responder, Ecu, Service, Pid, Length, Deadline, Invalid };
  explicit ObdSession(bool extended=false, bool allowUnknown=false)
      : allowUnknown_(allowUnknown), extended_(extended) {}
  uint32_t requestId() const { return extended_ ? 0x18DB33F1UL : 0x7DF; }
  bool responder(uint32_t id) const {
    return extended_ ? id<=0x1FFFFFFFUL && (id & 0x1FFFFF00UL)==0x18DAF100UL
                     : id>=0x7E8 && id<=0x7EF;
  }
  Support support(uint8_t pid) const {
    if (!pid || pid>0xC0) return Support::Unknown;
    for(unsigned i=0;i<10;++i)if(pids()[i]==pid && observed_[i])return Support::Supported;
    const unsigned page=(pid-1)/32, bit=31-((pid-1)%32);
    if (!known_[page]) return Support::Unknown;
    return (maps_[page] & (uint32_t(1)<<bit)) ? Support::Supported : Support::Unsupported;
  }
  // At most three attempts per bitmap page. Silence remains unknown.
  // Explicit legacy compatibility mode polls only the existing read-only PIDs
  // when unknown; a valid bitmap exclusion is never bypassed.
  bool next(uint32_t now, uint8_t& pid) {
    if (waiting_ || (started_ && uint32_t(now-last_)<100)) return false;
    if (page_<6) {
      pid=uint8_t(page_*32);
      if (page_ && support(pid)!=Support::Supported) { page_=6; return false; }
    } else {
      bool found=false;
      for (unsigned n=0;n<10;++n) {
        pid=pids()[index_]; index_=(index_+1)%10;
        const auto state=support(pid);
        if (state==Support::Supported || (allowUnknown_ && state==Support::Unknown)) { found=true; break; }
      }
      if (!found) return false;
    }
    if(page_<6)++attempts_;
    pending_=pid; timedOut_=false; waiting_=true; started_=true; last_=now; return true;
  }
  void tick(uint32_t now) {
    if (waiting_ && uint32_t(now-last_)>=200) {
      waiting_=false; timedOut_=true; timeoutEvent_=true; last_=now;
      if (pending_%32==0 && attempts_>=3) page_=6; // timeout is not unsupported
    }
  }
  // Legacy UART driver supplies 8 bytes but no actual DLC/IDE/RTR metadata.
  bool accept(uint32_t id, const uint8_t* d, size_t bytes, uint32_t now) {
    accepted_=false;
    tick(now);
    reason_=Reject::None;
    if (!d || bytes<3) {reason_=Reject::Length;return false;}
    if (!responder(id)) {reason_=Reject::Responder;return false;}
    if (selected_ && id!=ecu_) {reason_=Reject::Ecu;return false;}
    if (d[1]!=0x41) {reason_=Reject::Service;return false;}
    if (!waiting_) {reason_=(timedOut_ && d[2]==pending_)?Reject::Deadline:Reject::NoPending;return false;}
    if (d[2]!=pending_) {reason_=Reject::Pid;return false;}
    const unsigned payload=length(pending_);
    if (!payload || bytes<3+payload || d[0]!=2+payload) {reason_=Reject::Length;return false;}
    accepted_=true;
    if (!selected_) { ecu_=id; selected_=true; }
    waiting_=false; last_=now;
    if (pending_%32==0) {
      unsigned page=pending_/32;
      maps_[page]=(uint32_t(d[3])<<24)|(uint32_t(d[4])<<16)|(uint32_t(d[5])<<8)|d[6];
      known_[page]=true; page_=page+1; attempts_=0;
      return false; // capabilities are not telemetry
    }
    for(unsigned i=0;i<10;++i)if(pids()[i]==pending_)observed_[i]=true;
    return true;
  }
  static unsigned length(uint8_t pid) {
    if (pid%32==0 && pid<=0xA0) return 4;
    if (pid==0xA6) return 4;
    if (pid==0x0C || pid==0x31 || pid==0x42) return 2;
    for (unsigned i=0;i<10;++i) if (pids()[i]==pid) return 1;
    return 0;
  }
  Reject rejection() const { return reason_; }
  uint8_t pendingPid() const { return pending_; }
  unsigned bitmapAttempt() const { return attempts_; }
  bool takeTimeout() { const bool value=timeoutEvent_;timeoutEvent_=false;return value; }
  bool compatibilityEnabled() const { return allowUnknown_; }
  bool acceptedFrame() const { return accepted_; }
  bool waiting() const { return waiting_; }
  bool selected() const { return selected_; }
  uint32_t ecu() const { return ecu_; }
 private:
  static const uint8_t* pids() {
    static const uint8_t values[]={0x0C,0x0D,0x05,0x04,0x0F,0x11,0x31,0xA6,0x42,0x46};
    return values;
  }
  Reject reason_=Reject::None;
  bool timedOut_=false,timeoutEvent_=false;
  bool accepted_=false;
  bool allowUnknown_, observed_[10]={};
  bool extended_, waiting_=false, started_=false, selected_=false;
  bool known_[6]={};
  uint32_t maps_[6]={}, last_=0, ecu_=0;
  unsigned page_=0,index_=0,attempts_=0;
  uint8_t pending_=0;
};
}
#endif
