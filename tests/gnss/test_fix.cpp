#include "../../src/gnss-fix.h"
#include <assert.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>

static const char* sample =
    "+QGPSLOC: 131140.000,31.84678,117.19838,1.3,90.5,3,0.00,0.0,0.0,110919,07";

static std::string replaceField(unsigned field, const std::string& value) {
  std::string result(sample);
  size_t begin = result.find(' ') + 1;
  for (unsigned i = 0; i < field; ++i) begin = result.find(',', begin) + 1;
  size_t end = result.find(',', begin);
  if (end == std::string::npos) end = result.size();
  result.replace(begin, end - begin, value);
  return result;
}

static void reject(const char* input) {
  bg770a::GnssFix output;
  assert(bg770a::parseQgpsloc(sample, 42, output));
  if (bg770a::parseQgpsloc(input, 99, output)) {
    std::cerr << "Unexpected accepted input: " << input << "\n";
    assert(false);
  }
  assert(!output.valid && output.latitude == 0 && output.longitude == 0);
  assert(output.receivedMillis == 0 && output.utc[0] == 0 && output.date[0] == 0);
  assert(!bg770a::isFresh(output, 99));
}

int main() {
  bg770a::GnssFix fix;
  assert(!bg770a::isFresh(fix, 0));
  assert(bg770a::parseQgpsloc(sample, 1234, fix));
  assert(fix.valid && fabs(fix.latitude - 31.84678) < 0.000001);
  assert(fabs(fix.longitude - 117.19838) < 0.000001);
  assert(fabs(fix.hdop - 1.3) < 0.000001 && fix.altitude == 90.5);
  assert(fix.fix == 3 && fix.satellites == 7 && fix.receivedMillis == 1234);
  assert(strcmp(fix.utc, "131140.000") == 0 && strcmp(fix.date, "110919") == 0);
  assert(bg770a::isFresh(fix, 1234));
  assert(bg770a::isFresh(fix, 61234));
  assert(!bg770a::isFresh(fix, 61235));
  assert(bg770a::isFresh(fix, 61235, 100000));
  assert(bg770a::isFresh(fix, 601234, 600000));
  assert(!bg770a::isFresh(fix, 601235, 600000));
  assert(!bg770a::isFresh(fix, 601235, UINT32_MAX));
  assert(!bg770a::isFresh(fix, 1233));
  assert(bg770a::parseQgpsloc(sample, UINT32_MAX - 999, fix));
  assert(bg770a::isFresh(fix, 59000));
  assert(!bg770a::isFresh(fix, 59001));
  assert(bg770a::isFresh(fix, 0, 1000));
  assert(!bg770a::isFresh(fix, 1, 1000));
  assert(bg770a::parseQgpsloc((std::string(sample) + "\r\n").c_str(), 1, fix));
  assert(bg770a::parseQgpsloc("+QGPSLOC: 000000.000,-35.12345,-139.54321,0.5,-40.2,2,359.59,9999.9,9999.9,290224,12", 0, fix));
  assert(fix.latitude < 0 && fix.longitude < 0 && fix.altitude == -40.2 && fix.fix == 2);
  assert(bg770a::parseQgpsloc(replaceField(1, "0.00000").c_str(), 0, fix));
  assert(bg770a::parseQgpsloc(replaceField(1, "89.99999").c_str(), 0, fix));
  assert(bg770a::parseQgpsloc(replaceField(2, "-179.99999").c_str(), 0, fix));
  assert(bg770a::parseQgpsloc(replaceField(3, "99.9").c_str(), 0, fix));
  assert(bg770a::parseQgpsloc(replaceField(9, "290200").c_str(), 0, fix));
  reject(NULL);
  reject("");
  reject("+CME ERROR: 516");
  reject("+QGPSLOC: ,,,,,,,,,,");
  reject((std::string("prefix") + sample).c_str());
  reject((std::string(sample) + ",1").c_str());
  reject((std::string(sample) + "\r\nOK").c_str());
  reject((std::string(sample) + "\n\r").c_str());
  for (unsigned field = 0; field < 11; ++field) {
    const char* bad[] = {"", "NaN", "inf", "1e99", "1junk", " 1", "1 ", "+1", "0x12", "-", ".", "1.", "1\nX", "1\rX", "1,2", "999999999999999999999999999999999999999999"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
      reject(replaceField(field, bad[i]).c_str());
  }
  const char* times[] = {"240000.000", "126000.000", "125960.000", "131140", "131140.00", "131140.0000", "-31140.000"};
  for (size_t i = 0; i < sizeof(times) / sizeof(times[0]); ++i) reject(replaceField(0, times[i]).c_str());
  const char* dates[] = {"000126", "320126", "010026", "011326", "310426", "290223", "300224", "01012026"};
  for (size_t i = 0; i < sizeof(dates) / sizeof(dates[0]); ++i) reject(replaceField(9, dates[i]).c_str());
  const unsigned fields[] = {1,1,2,2,3,3,5,5,5,6,6,7,7,8,8,10,10,10};
  const char* values[] = {"90", "-90", "180", "-180", "0.49", "100", "1", "4", "3.0", "-1", "360", "-1", "10000", "-1", "10000", "13", "-1", "7.0"};
  for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); ++i) reject(replaceField(fields[i], values[i]).c_str());
  // Truncate before any satellite digit: no prefix is a valid complete response.
  const std::string original(sample);
  for (size_t i = 0; i <= original.rfind(','); ++i) reject(original.substr(0, i).c_str());
  // Exercise arbitrary bytes with a terminating NUL under sanitizers.
  uint32_t random = 1;
  char buffer[256];
  for (unsigned run = 0; run < 10000; ++run) {
    for (unsigned i = 0; i < sizeof(buffer) - 1; ++i) {
      random = random * 1664525u + 1013904223u;
      buffer[i] = static_cast<char>(random >> 24);
    }
    buffer[255] = 0;
    bg770a::parseQgpsloc(buffer, 0, fix);
  }
  std::cout << "GNSS parser tests passed\n";
}
