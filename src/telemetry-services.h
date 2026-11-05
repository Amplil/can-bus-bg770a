#pragma once
#include "ntp-session.h"
#include "gnss-session.h"
namespace bg770a {
// Shared AT owner, also used by the sketch: synchronous modem work is allowed
// only when idle(). NTP has priority; GNSS begins only after telemetry attempted.
class TelemetryServices {
 public:
  ntp_candidate::Uptime64 uptime;
  ntp_candidate::NtpClock clock;
  ntp_candidate::NtpSession ntp{clock};
  GnssSession gnss;
  bool ntpEnabled=true, gnssEnabled=true;
  uint64_t now(uint32_t low) { return uptime.update(low); }
  bool line(const std::string& line, uint32_t low) {
    const auto mono=now(low);
    if (ntp.line(line,mono)) return true;
    return gnss.line(line,low);
  }
  void tick(uint32_t low) {
    ntp.tick(now(low));
    if (ntp.phase()==ntp_candidate::NtpSession::Phase::Quarantined) {
      // Unknown final response: a physical reset is needed before another AT owner.
      ntp.resetBoundary(); gnss.recover();
    }
    if (ntp.modemIdle()) gnss.tick(low);
  }
  bool atIdle() const { return ntp.modemIdle() && gnss.modemIdle(); }
  bool idle() const { return atIdle() && gnss.canSend(); }
  std::string command(uint32_t low, int context, bool online,
                      bool allowNtp, bool allowGnss) {
    tick(low);
    if (!ntp.modemIdle()) return {};
    if (gnss.modemIdle() && online && allowNtp && ntpEnabled &&
        clock.ready(now(low)) && gnss.canSend() && ntp.begin(now(low),context,"ntp.nict.jp"))
      return ntp.command()+"\r";
    if (online && allowGnss && gnssEnabled && ntp.modemIdle()) gnss.begin(low);
    return gnss.takeCommand();
  }
  void powerCycle() { gnss=GnssSession{}; }
};
}
