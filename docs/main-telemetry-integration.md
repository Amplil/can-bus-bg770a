# Telemetry on the replacement main

Base: `Amplil/can-bus-bg770a` main `93af60465fbfec11a852343c302180161a5ae251`.
This ports the requested functionality, not the old main snapshot. The new main's
`imsi` + `data[]` envelope, numeric record keys (including odometer), DTC strings,
30-second send interval, CAN power gating, ten-minute idle sleep and hourly empty
heartbeat remain. UTC ISO8601 `time` retains its string type and explicit offset;
the previous code incorrectly relabeled local time with +09:00 and mishandled
long uptime. Added metadata is optional for consumers. The ingest service itself
is not available here; compatibility with strict rejection of extra fields needs
server validation. Required numeric keys retain the main's zero defaults; these
are placeholders for missing measurements, not evidence of PID support.

## Complete fork inventory (2026-11-05)

GitHub branches and all upstream PRs were read before publication:

| Existing fork branch / head | Prior PR | Disposition |
| --- | --- | --- |
| `main` / `72ebe58` | old upstream baseline | Superseded intentionally; no restoration of its schema/history |
| `codex/lte-ntp-time-sync-20261104` / `8599c70` | #2 closed, unmerged | Validated NTP result, UTC, resync, transport tests ported first |
| `codex/optional-gnss-20261104` / `6896d00` | #3 closed, unmerged | Bounded GNSS, recovery, freshness and tests ported through shared AT owner |
| `codex/hustler-compatibility-20261105` / `f24c604` | #4 closed, unmerged | CAN configuration, PID discovery, validated replies and tests adapted to new schema |

These were all four fork branches at audit time. The fork itself had no PRs. Upstream #1 was merged and came
from Amplil, not this fork. No additional unpublished feature branch was found.
Old fork main differs from replacement main in the cellular sketch and CAN driver
/header; replacement main's DTC/odometer additions are kept. Old raw CAN JSON and
arbitrary ECU-role labels are intentionally not restored. No experimental combined
modem branch was found or included. The three closed PRs stay closed.

One replacement PR combines all three because they share the sketch, build setup,
and AT transaction ownership. NTP remains the priority. CAN validation is modem
independent; GNSS depends on the shared owner to avoid interrupting NTP, sockets,
IMSI reads or power transitions. This is not a stack requiring the obsolete PRs.

## Time and modem ownership

Only a matching command echo, final OK, and valid `+QNTP` success notification
commit an NTP sample. An initial OK does not prove synchronization. Dates, leap
days and modem quarter-hour offsets are validated. UTC is extended from millis
with a 64-bit monotonic counter; resync is due every six hours, stale after 24.
Startup services CAN while waiting for NTP (135-second overall wait). Existing
WorldTimeAPI HTTP fallback is retained with two bounded attempts, complete JSON
validation and offset parsing. It is labeled `http_fallback`, never `ntp`. HTTP
is unauthenticated as in main. Failure of both sources defers timestamped records
and retries fallback while online, rather than inventing a date or rebooting.

`time_source`, `time_sync_state`, `utc_epoch_s` accompany each acquired record;
root time metadata describes the current clock. Already queued records are not
retimestamped on a later NTP correction. NTP/GNSS/synchronous HTTP cannot issue AT
commands concurrently. Unknown NTP command boundaries trigger the GNSS recovery
path's physical modem reset, restore flow control and LTE registration, disable
further NTP/GNSS acquisition for that boot, and preserve any prior clock anchor.
No AT is sent to a deliberately sleeping modem. Idle sleep needs an idle AT
boundary, not successful network registration. On CAN activity after sleep the
modem is restarted before sending. No hardware was reset or flashed in this task.

## GNSS

The first normal telemetry attempt precedes GNSS; subsequent acquisitions occur
at most every five minutes. GNSS temporarily takes LTE offline, searches for up
to 30 seconds, stops GNSS and restores LTE/registration. A pending command may
add its 20-second deadline. CAN receive/validation continues during this process.
Command errors or uncertain boundaries use bounded reset/boot recovery. No-fix
and stale-fix paths omit `location`; freshness is rechecked after TCP connection
setup. No zero coordinates are substituted. Location is current acquisition
metadata for the envelope, not the position of every queued historical record.

`location`: latitude/longitude (degrees), altitude_m, hdop, fix_type, satellites,
utc_hhmmss, date_ddmmyy, age_ms, source=gnss. Use a compatible **passive** GNSS
antenna. Hardware, firmware, antenna, RF, SIM and LTE reattachment need physical
validation. GNSS and LTE are exclusive; the send cadence can lengthen.

## Hustler / CAN

| Family | Generation / engine family | Verified vehicle support |
| --- | --- | --- |
| MR31S / MR41S | First generation / R06A | Unknown |
| MR52S | Second generation / R06A turbo | Unknown |
| MR92S | Second generation / R06D NA | Unknown |

Suzuki launch sources: [2014](https://www.suzuki.co.jp/release/a/2013/1224/),
[2020](https://www.suzuki.co.jp/release/a/2019/1224/),
[2026 refresh](https://www.suzuki.co.jp/release/a/2026/0527/).
No per-model bitrate, frame format or PID mappings are inferred from connector
shape or third-party compatibility. The inherited default is 500 kbps / 11-bit,
not a certified Hustler setting. Explicitly select `OBD_CAN_RATE` (250/500) and
`OBD_CAN_EXTENDED` (0/1) only with verified vehicle specifications. There is no
active protocol/bitrate probing. Grove GD32E103 CAN hardware cannot implement
K-line or another non-CAN physical layer through software.

Mode 01 bitmap discovery is bounded to PID 00 through A0 (six pages, following
only advertised continuation bits). The A0 page is needed to preserve main's
standard PID A6 odometer. The first valid PID 00 responder is selected for the
boot; this does not identify its ECU role. Polling supports ten existing PIDs,
only if advertised. Bit exclusions are unsupported; missing, negative and timed
out responses remain unknown. Discovery resets when the CAN module is reinitialized.
A 200 ms response deadline and 100 ms minimum request gap reject unrelated,
wrong-ECU, wrong-PID, wrong-service and malformed single-frame payloads. A
four-second acquisition window follows the existing ten-second group interval;
no record is created until an accepted measurement has a valid timestamp.

Root `obd.supported_pids` exposes support state; zero defaults in required fields
are not unsupported measurements. `distance_traveled` (PID 31) is distance since
DTC clear, not odometer. PID A6 remains `odometer`. Existing Mode 03 DTC read is
retained with selected-ECU, pending-request and single-frame validation; zero DTC
padding is ignored, hexadecimal digits are preserved. Multiframe DTC is still
marked MULTI and is not reassembled. No Mode 04 clear, other control service or
ISO-TP flow-control frames are added or sent by the cellular example.

UART receive supplies ID + eight bytes but no true DLC/IDE/RTR. Application length
checks cannot prove actual frame length/format. Existing partial UART reads and
synchronous TCP/HTTP/power operations can block and lose traffic. This is not a
lossless logger or a multi-ECU aggregator. Queue is bounded to 12 newest records;
failed sends retain records until that bound. No unverified DBC/ECU-role claims.

## Validation and remaining physical work

Run `bash tests/ntp/run_tests.sh`, `bash tests/gnss/run_tests.sh`, and
`bash tests/integration/run_tests.sh`. ArduinoJson 7.4.3 headers are required;
set ARDUINOJSON_INCLUDE or install PlatformIO dependencies. The integration suite
extracts and compiles current sketch functions and loop, not an old copied loop.
Transport fixtures are WioCellular 0.3.16 at
`9275de1d15f8bd2761f562b8895edc0ee9d1a1c5` with its MIT license.

`pio run` compiles all five profiles: combined/default, NTP without GNSS, GNSS
without NTP, CAN with HTTP time only, and combined 29-bit/250 kbps. Profiles verify
compilation, not vehicle protocol selection. Build dependency pins and wrapper
are shared. CI runs the three host suites and all target profiles.

Required before claiming hardware/model support: record actual vehicle/year/trim,
protocol and module firmware; capture raw requests and PID bitmaps per family;
compare telemetry/DTC/odometer with an independent scan tool. Test no ECU, multiple
ECUs, negative/late replies, ignition transitions, sustained bus load, missing NTP
notifications, timezone rollover, HTTP fragmentation, no GNSS antenna/fix, slow
TCP, LTE reattachment, modem recovery, no-network idle sleep, CAN wake and heartbeat.
No real vehicle, board, antenna, SIM or ingest server was available for these tests.

## Recorded local result

All three UBSan host suites PASS. All five PlatformIO 6.2.0 profiles PASS.
Combined default: RAM 12,380 / 237,568 bytes; flash 133,520 / 798,720 bytes.
The framework emits its three existing ignored `used` attribute warnings in
startup_nrf_common.c for each clean target build; there were no build errors.
No independent agent review was performed. Physical/end-to-end ingest tests
remain NOT RUN for the unavailable hardware and server listed above.
