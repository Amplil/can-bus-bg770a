#ifndef BG770A_GNSS_FIX_H
#define BG770A_GNSS_FIX_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace bg770a {

struct GnssFix {
  bool valid;
  double latitude;
  double longitude;
  double hdop;
  double altitude;
  unsigned fix;
  unsigned satellites;
  char utc[11];  // hhmmss.sss
  char date[7];  // ddmmyy, interpreted as 2000..2099 for leap-day validation
  uint32_t receivedMillis;

  GnssFix() : valid(false), latitude(0), longitude(0), hdop(0), altitude(0),
              fix(0), satellites(0), receivedMillis(0) {
    memset(utc, 0, sizeof(utc));
    memset(date, 0, sizeof(date));
  }
};

namespace detail {
inline bool digit(char c) { return c >= '0' && c <= '9'; }
inline unsigned pair(const char* p) {
  return static_cast<unsigned>((p[0] - '0') * 10 + p[1] - '0');
}

// Decimal-only grammar: no exponent, hexadecimal, NaN/Inf, whitespace or suffix.
// Token length is bounded below, so accumulation cannot overflow even float32.
inline bool decimal(const char* p, double& value) {
  bool negative = false;
  if (*p == '-') { negative = true; ++p; }
  if (!digit(*p)) return false;
  double result = 0;
  while (digit(*p)) result = result * 10 + (*p++ - '0');
  if (*p == '.') {
    ++p;
    if (!digit(*p)) return false;
    double fraction = 0, scale = 1;
    while (digit(*p)) {
      fraction = fraction * 10 + (*p++ - '0');
      scale *= 10;
    }
    result += fraction / scale;
  }
  if (*p) return false;
  value = negative ? -result : result;
  return true;
}

inline bool utcValid(const char* p) {
  if (strlen(p) != 10 || p[6] != '.') return false;
  for (unsigned i = 0; i < 10; ++i)
    if (i != 6 && !digit(p[i])) return false;
  return pair(p) < 24 && pair(p + 2) < 60 && pair(p + 4) < 60;
}

inline bool dateValid(const char* p) {
  if (strlen(p) != 6) return false;
  for (unsigned i = 0; i < 6; ++i) if (!digit(p[i])) return false;
  const unsigned day = pair(p), month = pair(p + 2), year = 2000 + pair(p + 4);
  if (month < 1 || month > 12) return false;
  static const unsigned days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  const unsigned maximum = days[month - 1] + (month == 2 && year % 4 == 0 ? 1 : 0);
  return day >= 1 && day <= maximum;
}
}  // namespace detail

// Parse exactly one AT+QGPSLOC=2 response line, optionally ending in CR/LF.
// The caller must ensure its AT transaction succeeded before publishing the fix.
// Source: Quectel BG77xA-GL&BG95xA-GL GNSS Application Note V1.0,
// section 2.3.4. Documented bounds are enforced, including HDOP and nsat.
// Failure always clears out; previous coordinates must never escape as a new fix.
inline bool parseQgpsloc(const char* line, uint32_t receivedMillis, GnssFix& out) {
  out = GnssFix();
  if (!line || strncmp(line, "+QGPSLOC:", 9) != 0) return false;
  line += 9;
  while (*line == ' ' || *line == '\t') ++line;
  char fields[11][32] = {};
  for (unsigned field = 0; field < 11; ++field) {
    size_t size = 0;
    while (*line && *line != ',' && *line != '\r' && *line != '\n') {
      if (size >= sizeof(fields[field]) - 1) return false;
      fields[field][size++] = *line++;
    }
    if (!size) return false;
    if (field < 10) {
      if (*line != ',') return false;
      ++line;
    }
  }
  if (*line == '\r') ++line;
  if (*line == '\n') ++line;
  if (*line) return false;

  GnssFix candidate;
  if (!detail::utcValid(fields[0]) || !detail::dateValid(fields[9])) return false;
  if (!detail::decimal(fields[1], candidate.latitude) ||
      candidate.latitude < -89.99999 || candidate.latitude > 89.99999 ||
      !detail::decimal(fields[2], candidate.longitude) ||
      candidate.longitude < -179.99999 || candidate.longitude > 179.99999 ||
      !detail::decimal(fields[3], candidate.hdop) ||
      candidate.hdop < 0.5 || candidate.hdop > 99.9 ||
      !detail::decimal(fields[4], candidate.altitude)) return false;
  if (strcmp(fields[5], "2") != 0 && strcmp(fields[5], "3") != 0) return false;
  candidate.fix = static_cast<unsigned>(fields[5][0] - '0');

  double course, speedKmh, speedKnots;
  if (!detail::decimal(fields[6], course) || course < 0 || course >= 360 ||
      !detail::decimal(fields[7], speedKmh) || speedKmh < 0 || speedKmh > 9999.9 ||
      !detail::decimal(fields[8], speedKnots) || speedKnots < 0 || speedKnots > 9999.9)
    return false;
  const size_t satelliteLength = strlen(fields[10]);
  if (satelliteLength > 2) return false;
  unsigned satellites = 0;
  for (size_t i = 0; i < satelliteLength; ++i) {
    if (!detail::digit(fields[10][i])) return false;
    satellites = satellites * 10 + static_cast<unsigned>(fields[10][i] - '0');
  }
  if (satellites > 12) return false;
  candidate.satellites = satellites;
  memcpy(candidate.utc, fields[0], sizeof(candidate.utc));
  memcpy(candidate.date, fields[9], sizeof(candidate.date));
  candidate.receivedMillis = receivedMillis;
  candidate.valid = true;
  out = candidate;
  return true;
}

// Unsigned subtraction handles a millis() rollover. No fix survives >60 seconds,
// even if a caller supplies a larger limit. Call regularly to expire cached data;
// a 32-bit counter cannot distinguish intervals separated by an entire wrap.
inline bool isFresh(const GnssFix& fix, uint32_t nowMillis, uint32_t maxAgeMillis = 60000) {
  if (maxAgeMillis > 60000) maxAgeMillis = 60000;
  return fix.valid && static_cast<uint32_t>(nowMillis - fix.receivedMillis) <= maxAgeMillis;
}

}  // namespace bg770a
#endif
