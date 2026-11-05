#ifndef BG770A_OBD_SESSION_H
#define BG770A_OBD_SESSION_H
#include <stdint.h>
#include <stddef.h>
namespace bg770a {
enum class Support { Unknown, Unsupported, Supported };
// Fixed standard Mode 01 telemetry only. No transport probing or control services.
class ObdSession {
 public:
  explicit ObdSession(bool extended=false) : extended_(extended) {}
  uint32_t requestId() const { return extended_ ? 0x18DB33F1UL : 0x7DF; }
  bool responder(uint32_t id) const {
    return extended_ ? id<=0x1FFFFFFFUL && (id & 0x1FFFFF00UL)==0x18DAF100UL
                     : id>=0x7E8 && id<=0x7EF;
  }
  Support support(uint8_t pid) const {
    if (!pid || pid>0x60) return Support::Unknown;
    const unsigned page=(pid-1)/32, bit=31-((pid-1)%32);
    if (!known_[page]) return Support::Unknown;
    return (maps_[page] & (uint32_t(1)<<bit)) ? Support::Supported : Support::Unsupported;
  }
  // Discovery happens once per boot, at most three requests. Silence stays unknown.
  bool next(uint32_t now, uint8_t& pid) {
    if (waiting_ || (started_ && uint32_t(now-last_)<100)) return false;
    if (page_<3) {
      pid=uint8_t(page_*32);
      if (page_ && support(pid)!=Support::Supported) { page_=3; return false; }
    } else {
      bool found=false;
      for (unsigned n=0;n<9;++n) {
        pid=pids()[index_]; index_=(index_+1)%9;
        if (support(pid)==Support::Supported) { found=true; break; }
      }
      if (!found) return false;
    }
    pending_=pid; waiting_=true; started_=true; last_=now; return true;
  }
  void tick(uint32_t now) {
    if (waiting_ && uint32_t(now-last_)>=200) {
      waiting_=false; last_=now;
      if (pending_%32==0) page_=3; // timeout is not unsupported
    }
  }
  // Legacy UART driver supplies 8 bytes but no actual DLC/IDE/RTR metadata.
  bool accept(uint32_t id, const uint8_t* d, size_t bytes, uint32_t now) {
    tick(now);
    if (!d || !waiting_ || !responder(id) || (selected_ && id!=ecu_) || bytes<3) return false;
    const unsigned payload=length(pending_);
    if (!payload || bytes<3+payload || d[0]!=2+payload || d[1]!=0x41 || d[2]!=pending_) return false;
    if (!selected_) { ecu_=id; selected_=true; }
    waiting_=false; last_=now;
    if (pending_%32==0) {
      unsigned page=pending_/32;
      maps_[page]=(uint32_t(d[3])<<24)|(uint32_t(d[4])<<16)|(uint32_t(d[5])<<8)|d[6];
      known_[page]=true; page_=page+1;
      return false; // capabilities are not telemetry
    }
    return true;
  }
  static unsigned length(uint8_t pid) {
    if (pid==0 || pid==0x20 || pid==0x40) return 4;
    if (pid==0x0C || pid==0x31 || pid==0x42) return 2;
    for (unsigned i=0;i<9;++i) if (pids()[i]==pid) return 1;
    return 0;
  }
  bool selected() const { return selected_; }
  uint32_t ecu() const { return ecu_; }
 private:
  static const uint8_t* pids() {
    static const uint8_t values[]={0x0C,0x0D,0x05,0x04,0x0F,0x11,0x31,0x42,0x46};
    return values;
  }
  bool extended_, waiting_=false, started_=false, selected_=false;
  bool known_[3]={false,false,false};
  uint32_t maps_[3]={0,0,0}, last_=0, ecu_=0;
  unsigned page_=0,index_=0;
  uint8_t pending_=0;
};
}
#endif
