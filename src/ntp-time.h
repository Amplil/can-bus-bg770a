#pragma once
#include <cstdint>
#include <string>

// Strict modem NTP date parser and wrap-safe monotonic extension.
namespace ntp_candidate {
inline bool leap(int y) { return y % 4 == 0 && (y % 100 != 0 || y % 400 == 0); }
enum class CalendarBasis { Offset, Utc };
inline bool parseCalendar(const std::string& s, int64_t& epoch, CalendarBasis basis) {
  // Modem-style calendar grammar, without enclosing quotes. Validate the
  // timezone indicator even when the command's clock digits are already UTC.
  if (s.size() != 22 || s[4]!='/' || s[7]!='/' || s[10]!=',' ||
      s[13]!=':' || s[16]!=':' || (s[19]!='+' && s[19]!='-')) return false;
  for (unsigned i=0;i<s.size();++i) {
    if (i==4||i==7||i==10||i==13||i==16||i==19) continue;
    if (s[i]<'0'||s[i]>'9') return false;
  }
  auto number=[&](int p,int n) { int v=0; while(n--) v=v*10+s[p++]-'0'; return v; };
  const int y=number(0,4), m=number(5,2), d=number(8,2), h=number(11,2),
            min=number(14,2), sec=number(17,2), tz=number(20,2)*(s[19]=='-'?-1:1);
  // Application range, not an assertion of the modem's supported date range.
  if(y<2000||y>2099||m<1||m>12||h>23||min>59||sec>59||tz< -48||tz>56) return false;
  const int mdays[]={31,28,31,30,31,30,31,31,30,31,30,31};
  if(d<1||d>mdays[m-1]+(m==2&&leap(y))) return false;
  int64_t days=0;
  for(int year=1970;year<y;++year) days+=365+leap(year);
  for(int month=1;month<m;++month) days+=mdays[month-1]+(month==2&&leap(y));
  days+=d-1;
  epoch=days*86400+h*3600+min*60+sec;
  if(basis==CalendarBasis::Offset)epoch-=tz*15*60;
  return true;
}
// Offset-bearing civil time, used by the ISO HTTP fallback adapter.
inline bool parse(const std::string& s, int64_t& epoch) {
  return parseCalendar(s,epoch,CalendarBasis::Offset);
}
// BG770A QNTP supplies UTC clock digits with a separate timezone indicator.
// Applying that indicator as an ISO offset subtracts the local offset twice.
inline bool parseQntpUtc(const std::string& s, int64_t& epoch) {
  return parseCalendar(s,epoch,CalendarBasis::Utc);
}
// Observe with every forward gap strictly below 2^31 ms (~24.85 days), including offline.
// Half-range ordering distinguishes a real forward wrap from a stale sample.
// A reset or a longer gap is ambiguous and requires a fresh clock instance.
class Uptime64 {
  uint32_t previous_ = 0;
  uint64_t elapsed_ = 0;
  bool initialized_ = false;
public:
  uint64_t update(uint32_t now) {
    if(!initialized_) { initialized_=true;previous_=now;elapsed_=now;return elapsed_; }
    const uint32_t delta=uint32_t(now-previous_);
    if(delta>=0x80000000UL)return elapsed_; // stale/ambiguous: do not move either anchor
    elapsed_+=delta;previous_=now;return elapsed_;
  }
  uint32_t latestTick() const { return previous_; }
  uint64_t value() const { return elapsed_; }
  // Historical event projection never mutates the live monotonic accumulator.
  // Future, ambiguous or pre-start samples cannot be used as historical times.
  bool project(uint32_t sample,uint64_t& event) const {
    if(!initialized_)return false;
    const uint32_t age=uint32_t(previous_-sample);
    if(age>=0x80000000UL || uint64_t(age)>elapsed_)return false;
    event=elapsed_-age;return true;
  }
};
}
