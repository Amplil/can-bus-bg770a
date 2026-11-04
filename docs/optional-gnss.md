# Optional GNSS (BG770A-GL)

The cellular example adds a top-level `location` object only when a valid GNSS fix
is available and at most 60 seconds old. Without a fix the existing CAN/vehicle
payload is sent without `location`. No zero coordinates or previous JSON location
are substituted. GNSS time is metadata for the position, not NTP synchronization.

## Timing and exclusivity

BG770A-GL cannot run LTE and GNSS simultaneously. The first telemetry attempt runs
before GNSS. Thereafter, at most once per five minutes (measured from restoration),
the example closes its TCP scope, sends `AT+CFUN=4`, starts GNSS with `AT+QGPS=1`,
and polls `AT+QGPSLOC=2` every second. It stops searching after 30 seconds and
attempts `AT+QGPSEND` then `AT+CFUN=1` and restores registration URCs.
A rejected GNSS stop instead takes the hardware-reset recovery path. Current
registration is explicitly queried after restoration and every five seconds
while disconnected, so missing an early registration URC cannot stall telemetry.

This intentionally interrupts LTE: the normal 10-second telemetry cadence is
extended during acquisition and network re-registration. It is not simultaneous
or continuous GNSS tracking. A cold start may need more than 30 seconds and produce
no fix; bounded telemetry interruption is preferred over waiting indefinitely.
An outstanding AT command may take another 20 seconds before timeout recovery.
CAN polling continues throughout acquisition, boot and re-registration, with a
50-message queue bound. This is not lossless CAN logging at high bus utilization.
The existing TCP operations still block; this change does not make them asynchronous.

The AT state machine owns the modem until restoration. It does not call the
library's synchronous query/execute functions for GNSS. Each command must receive
its matching echo and final result. Missing/late responses or restoration errors
trigger a hardware reset rather than send an AT command into an uncertain response
stream. Reset pulses block about 102 ms; subsequent boot waiting is asynchronous
and retried every 30 seconds if the modem stays unresponsive. Hardware flow
control is reconfigured with `AT+IFC=2,2` before LTE is restored. GNSS is disabled for
the rest of that boot after command/start errors or reset recovery, prioritizing
LTE telemetry. A physically unresponsive modem or unavailable cellular network
cannot transmit; the sketch keeps servicing CAN and never calls `abort()` for a
GNSS failure. Existing initial LTE setup behavior is unchanged.

A fix is committed only after a valid location response **and** its command's OK.
Repeated UTC/date from the previous accepted acquisition is rejected. Age is
checked again immediately before serialization, because opening a TCP connection
may take longer than the 60-second freshness allowance. A stale fix is omitted.
The bounded latest CAN queue is retained on failed sends and cleared on success.

## Payload

`location`: `latitude`, `longitude` (decimal degrees), `altitude_m`, `hdop`,
`fix_type` (2/3), `satellites`, `utc_hhmmss` (hhmmss.sss), `date_ddmmyy`,
`age_ms`, `source` (`gnss`). These coordinates describe the acquisition time,
not necessarily each historical CAN frame. GPS date/time is not a replacement for
an authenticated/reliable system clock. No GNSS data is persisted outside RAM.

## Hardware and validation

Use a compatible **passive** GNSS antenna on the GNSS connector, not the LTE
connector. The Seeed board does not support active antennas. The official verified
combination is JCA043SM3M20R plus JC-SMA-J-IPEX(IPX/U.FL). An unspecified purchased
antenna cannot be assumed compatible. Configure the build's BOARD_VERSION for the
actual board before flashing (the repository currently defaults to v1.0).

- [Seeed hardware manual](https://seeedjp.github.io/Wiki/Wio_BG770A/hardware.html)
- [Quectel GNSS Application Note, section 2.3.4](https://www.quectel.com/content/uploads/2024/04/Quectel_BG77xA-GLBG95xA-GL_GNSS_Application_Note_V1.0.pdf)
- [WioCellular source](https://github.com/SeeedJP/wio_cellular)

Host tests: `bash tests/gnss/run_tests.sh` (parser, session and actual WioCellular
AT transport under UBSan). The vendored transport fixtures are from WioCellular
0.3.16, commit `9275de1d15f8bd2761f562b8895edc0ee9d1a1c5`, with its MIT license.
Target build: `pio run -e seeed_wio_bg770a`.

Hardware, SIM and antenna testing remains required: outdoor fix, absent antenna,
no fix, LTE re-registration after each GNSS window, unavailable network, modem
error/timeout/reset, CAN load during acquisition/recovery, and freshness after a
slow TCP connection. Host tests and compilation do not prove RF reception or
modem firmware behavior. No hardware was flashed as part of this change.

## Separate NTP PR

This branch is based directly on main and does not include the parallel LTE NTP
change. Both change cellular.ino and must be reconciled before using them together:
GNSS must not start while NTP owns an AT transaction (`NtpSession.modemIdle()` is
false), and NTP must not start while GNSS owns the modem or LTE is offline. Share
one AT arbiter, dispatch both services during CAN waits, and repeat combined tests.
Do not combine the two service calls without this arbitration.
