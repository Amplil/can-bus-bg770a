#pragma once
#include <stdint.h>
namespace bg770a {
// Verbose by default. Bound only raw-frame detail under heavy unrelated traffic;
// all request/timeout/configuration events and periodic totals remain visible.
struct CanDiagnostics {
  uint32_t queued=0, queueFailures=0, received=0, telemetry=0, bitmaps=0, dtc=0;
  uint32_t dropped[9]={}, syncDeferred=0, suppressed=0;
  bool detail(uint32_t now) {
    if(!window_ || uint32_t(now-windowAt_)>=1000) {window_=true;windowAt_=now;details_=0;}
    if(details_>=64){++suppressed;return false;}
    ++details_;return true;
  }
  bool summary(uint32_t now) {
    if(reported_ && uint32_t(now-reportAt_)<5000)return false;
    reported_=true;reportAt_=now;return true;
  }
 private:
  bool window_=false,reported_=false;
  uint32_t windowAt_=0,reportAt_=0;unsigned details_=0;
};
}
