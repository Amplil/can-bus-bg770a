#pragma once
#include <cstdint>
#include <string>

// Offline design candidate only. Not integrated into, or built for, firmware.
namespace ntp_candidate {
inline bool leap(int y) { return y % 4 == 0 && (y % 100 != 0 || y % 400 == 0); }
inline bool parse(const std::string& s, int64_t& epoch) {
  // QNTP payload, without enclosing quotes. Fail closed on all malformed fields.
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
  epoch=days*86400+h*3600+min*60+sec-tz*15*60;
  return true;
}
// Call regularly, at least once per 2^32 milliseconds, including while offline.
class Uptime64 {
  uint32_t previous_ = 0;
  uint64_t elapsed_ = 0;
public:
  uint64_t update(uint32_t now) { elapsed_ += uint32_t(now-previous_); previous_=now; return elapsed_; }
};
}
