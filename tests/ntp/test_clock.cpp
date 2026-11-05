#include "ntp-clock.h"
#include <cassert>
#include <iostream>
using ntp_candidate::NtpClock;
const char* ok="+QNTP: 0,\"2026/11/04,09:00:00+36\"";
int main() {
 int64_t e=0,base=0; ntp_candidate::parse("2026/11/04,00:00:00+00",base);
 NtpClock c;
 assert(!c.utc(0,e)); assert(!c.urc("+OTHER: 0",0));
 assert(c.begin(0)); assert(!c.begin(1)); c.accepted(1); assert(!c.utc(1,e));
 c.urc(ok,100); assert(c.utc(1100,e)&&e==base+1);
 c.urc("+QNTP: 0,\"2000/01/01,00:00:00+00\"",2000); assert(c.utc(2100,e)&&e==base+2);
 assert(!c.ready(1000)); assert(c.ready(100+NtpClock::ResyncMs));
 c.begin(100+NtpClock::ResyncMs); c.rejected(200+NtpClock::ResyncMs);
 assert(c.utc(300+NtpClock::ResyncMs,e));
 assert(std::string(c.quality(100+NtpClock::StaleMs))=="stale");
 NtpClock early; early.begin(0); early.urc(ok,1); assert(!early.utc(2,e)); early.accepted(3); assert(early.utc(3,e)&&e==base);
 NtpClock error; error.begin(0); error.accepted(1); error.urc("+QNTP: 563",2);
 assert(!error.utc(3,e)); assert(!error.ready(100)); assert(error.ready(60002));
 NtpClock bad; bad.begin(0); bad.accepted(1); bad.urc("+QNTP: 0,\"2026/02/30,00:00:00+00\"",2); assert(!bad.utc(3,e));
 NtpClock timed; timed.begin(0); timed.accepted(1); timed.tick(130000); assert(timed.state()==NtpClock::State::Quarantined);
 timed.urc(ok,130001); assert(!timed.utc(130002,e)); assert(!timed.ready(999999999));
 NtpClock uncertain; uncertain.begin(0); uncertain.uncertain(); uncertain.urc(ok,2); assert(!uncertain.utc(3,e)); assert(!uncertain.ready(999999999));
 NtpClock query; query.begin(0); query.accepted(1); query.urc("+QNTP: \"server\",123",2); assert(query.state()==NtpClock::State::Pending);
 NtpClock delayed; delayed.begin(0); delayed.accepted(1); delayed.urc(ok,130001);
 assert(delayed.state()==NtpClock::State::Quarantined); assert(!delayed.utc(130002,e));
 std::cout << "NTP state-machine tests passed\n";
}
