#include "ntp-time.h"
#include <cassert>
#include <iostream>
int main() {
 using namespace ntp_candidate;
 int64_t e=0, ref=0;
 assert(parse("2000/01/01,00:00:00+00",e)&&e==946684800);
 assert(parse("2024/02/29,00:00:00+00",e)&&e==1709164800);
 assert(parse("2026/11/04,00:00:00+00",ref));
 assert(parse("2026/11/04,09:00:00+36",e)&&e==ref);
 assert(parse("2026/11/03,12:00:00-48",e)&&e==ref);
 assert(parse("2026/11/04,14:00:00+56",e)&&e==ref);
 assert(parse("2026/11/04,05:45:00+23",e)&&e==ref);
 assert(parse("2026/11/03,23:45:00-01",e)&&e==ref);
 assert(parse("2099/12/31,23:59:59+00",e)&&e==4102444799LL);
 const char* bad[]={"","2026/11/04,00:00:00", "2026/11/04,00:00:00+000", "2026/11/04,00:00:00+00x",
 "2026-11/04,00:00:00+00", "202X/11/04,00:00:00+00", "2026/11/04,00:00:00*00",
 "2023/02/29,00:00:00+00", "2024/02/30,00:00:00+00", "2026/04/31,00:00:00+00",
 "2026/00/01,00:00:00+00", "2026/13/01,00:00:00+00", "2026/11/00,00:00:00+00",
 "2026/11/04,24:00:00+00", "2026/11/04,23:60:00+00", "2026/11/04,23:59:60+00",
 "2026/11/04,00:00:00+57", "2026/11/04,00:00:00-49", "1999/12/31,00:00:00+00", "2100/01/01,00:00:00+00"};
 for(auto s:bad) { e=123; assert(!parse(s,e)); assert(e==123); }
 Uptime64 u;
 assert(u.update(0xfffffff0U)==0xfffffff0ULL);
 assert(u.update(0x20U)==0x100000020ULL);
 assert(u.update(0x20U)==0x100000020ULL);
 std::cout << "32 scenario checks passed (9 valid dates, 20 rejection cases, 3 uptime checks)\n";
}
