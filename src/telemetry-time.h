#pragma once
#include "ntp-time.h"
#include "gnss-fix.h"
#include <ctime>
#include <cstdio>
namespace bg770a {
// The fix carries UTC independently of the modem/NTP clock and upload time.
inline std::string gnssIsoUtc(const GnssFix& fix) {
  if(!fix.valid || !detail::dateValid(fix.date) || !detail::utcValid(fix.utc))return {};
  char out[30];
  std::snprintf(out,sizeof(out),"20%.2s-%.2s-%.2sT%.2s:%.2s:%.2s.%.3s+00:00",
                fix.date+4,fix.date+2,fix.date,fix.utc,fix.utc+2,fix.utc+4,fix.utc+7);
  return out;
}
inline bool parseHttpTime(const std::string& s, int64_t& epoch) {
  // WorldTimeAPI ISO8601 with optional fractional seconds and numeric zone.
  if (s.size()<25 || s[4]!='-' || s[7]!='-' || s[10]!='T') return false;
  size_t zone=19;
  if (s[zone]=='.') { ++zone; const auto start=zone;
    while(zone<s.size() && s[zone]>='0' && s[zone]<='9') ++zone;
    if(zone==start) return false;
  }
  if(s.size()!=zone+6 || (s[zone]!='+' && s[zone]!='-') || s[zone+3]!=':') return false;
  for (auto i : {zone+1,zone+2,zone+4,zone+5}) if(s[i]<'0'||s[i]>'9')return false;
  const int minutes=((s[zone+1]-'0')*10+s[zone+2]-'0')*60+(s[zone+4]-'0')*10+s[zone+5]-'0';
  if(minutes>840 || minutes%15 || (s[zone+4]-'0')*10+s[zone+5]-'0'>59) return false;
  std::string q=s.substr(0,19);q[4]='/';q[7]='/';q[10]=',';
  const char suffix[]={s[zone],char('0'+minutes/15/10),char('0'+minutes/15%10),0};
  q+=suffix;return ntp_candidate::parse(q,epoch);
}
inline std::string isoUtc(int64_t epoch) {
  if(epoch<946684800LL || epoch>4102444799LL) return {};
  const time_t seconds=static_cast<time_t>(epoch);
  const auto* calendar=std::gmtime(&seconds);if(!calendar)return {};
  char out[26];if(!std::strftime(out,sizeof(out),"%Y-%m-%dT%H:%M:%S+00:00",calendar))return {};
  return out;
}
inline std::string isoUtcMilliseconds(int64_t epochMs) {
  const auto seconds=isoUtc(epochMs/1000);
  if(seconds.empty() || epochMs<0)return {};
  char fraction[5];
  std::snprintf(fraction,sizeof(fraction),".%03u",unsigned(epochMs%1000));
  return seconds.substr(0,19)+fraction+seconds.substr(19);
}
}
