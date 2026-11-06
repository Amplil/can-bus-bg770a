# Telemetry on the replacement main

Base: `Amplil/can-bus-bg770a` main `93af60465fbfec11a852343c302180161a5ae251`.
This ports the requested functionality, not the old main snapshot. The new main's
`imsi` + `data[]` envelope, numeric record keys (including odometer), DTC strings,
30-second measurement send interval and CAN power gating remain. Idle sleep is
at most ten minutes and ends by the next hourly heartbeat deadline.
An explicit timestamp-only heartbeat is due at startup as soon as LTE, device
identity and validated UTC are available, including when no OBD or GPS reading
is available. Further heartbeats are due one hour after the last acknowledged
heartbeat, independently of measurement uploads.
UTC ISO8601 `time` retains its string type and explicit offset;
the previous code incorrectly relabeled local time with +09:00 and mishandled
long uptime. Added metadata is optional for consumers. The companion dashboard
accepts the root GNSS location and preserves its acquisition time separately
from OBD record time. Each OBD numeric field and DTC field starts as JSON `null`
and is populated only by an accepted reply. A measured zero remains numeric `0`.
A measurement row with no acquired OBD measurement, DTC result or valid GPS fix
is discarded before upload. An explicit heartbeat is stored separately from
measurement rows. A real GPS-only row is eligible while all OBD fields remain `null`.

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
The BG770A QNTP decoder treats its clock digits as UTC and validates the trailing
timezone indicator as metadata. It does not subtract that indicator again.
HTTP ISO timestamps use a separate offset-aware path: `22:01:57+09:00` becomes
`13:01:57+00:00`. GNSS fixes already carry UTC. These source conversions are
independent of the dashboard's selectable JST/UTC presentation.

`NTP-SAMPLE` prints only a validated, fixed-length `response_time` and its
`interpreted_utc`, allowing the modem's response to be checked against an
independent clock. The supplied November 6 send trace showed `04:01:57+00:00`
at monitor time `22:01:57` JST; the correct UTC for that monitor time is
`13:01:57`. The subsequently inspected startup monitor also showed the nine-hour
error immediately at `NTP-RESULT`, with `source=ntp`. Neither trace includes the
original QNTP result. The regression response `13:01:57+36` is synthetic: the
former offset subtraction maps it to `04:01:57`, and the QNTP-specific path
retains `13:01:57`. Existing stored or queued records are not rewritten.

Reference: [Quectel BG770A TCP/IP application note, section 2.3.14](https://quectel.com/content/uploads/2024/02/Quectel_BG770A-GLBG95xA-GL_TCPIP_Application_Note_V1.1.pdf)
and [Seeed's BG770A clock interface](https://seeedjp.github.io/wio_cellular/classwiocellular_1_1module_1_1bg770a_1_1commands_1_1Bg770aHardwareRelatedCommands.html),
which exposes the GMT clock and time difference separately. The diagnostic
records the actual response so firmware-specific behavior can be distinguished.

Startup services CAN while waiting for NTP (135-second overall wait). Existing
WorldTimeAPI HTTP fallback is retained with two bounded attempts, complete JSON
validation and offset parsing. It is labeled `http_fallback`, never `ntp`. HTTP
is unauthenticated as in main. Failure of both sources defers timestamped OBD records
and retries fallback while online, rather than inventing a date or rebooting.

`time_source`, `time_sync_state`, `utc_epoch_s` accompany each acquired record;
root time metadata describes the current clock. Already queued records are not
retimestamped on a later NTP correction. NTP/GNSS/synchronous HTTP cannot issue AT
commands concurrently. Unknown NTP command boundaries trigger the GNSS recovery
path's physical modem reset, restore flow control and LTE registration, disable
further NTP/GNSS acquisition for that boot, and preserve any prior clock anchor.
No AT is sent to a deliberately sleeping modem. Idle sleep needs an idle AT
boundary, not successful network registration. On CAN activity after sleep the
modem is restarted before sending. Physical verification is required for each
supported hardware and firmware configuration.

## GNSS

After network/time setup and device identification, startup immediately polls
one complete OBD round before starting GNSS. The round finishes after capability
discovery and a pass through the ten configured telemetry PIDs: supported PIDs
and permitted unknown PIDs receive a reply or reach their existing timeout;
excluded PIDs are skipped. The first round starts without the normal ten-second
group delay and is allowed to finish even if it exceeds the usual four-second
window. Bitmap replies or the first measurement alone do not finish the round.
The resulting OBD record becomes due for LTE transmission immediately, before
GNSS takes the radio offline. The send attempt waits for LTE readiness and any
pending CAN reply. An acknowledgment removes the transmitted prefix; a failed
attempt retains it for retry and permits the GPS search to proceed. A round
without timestamped measurements, including a silent/unavailable ECU, proceeds
to GPS without sending a fabricated OBD row.

During the GPS search, normal OBD PID polling and CAN reply collection continue.
Accepted OBD measurements accumulate in the existing bounded 12-record queue.
A valid fix becomes a GPS-only row and is sent as soon as LTE/registration is
restored, independently of ECU availability or OBD responses.
The row uses GNSS UTC seconds for `time`,
the exact acquisition time including milliseconds for `gps_time`, and direct
per-record `latitude`, `longitude` and `altitude`. All OBD and DTC fields remain
`null`. Once a fix is queued or the search ends without a fix, the accumulated
OBD queue becomes due immediately. After GNSS stops and LTE/registration is
restored, the sender finishes any pending CAN reply, attempts the GPS row first
when available, and drains acknowledged OBD prefixes about one second apart
within the 1,024-byte payload limit. A rejected GPS upload retains its own row
for the normal 30-second retry while OBD uploads proceed independently. OBD
send failures retain the unacknowledged queue for the normal retry interval.
The separate GPS document survives bounded OBD-queue eviction during prolonged
delivery failure, and idle sleep waits for its acknowledgment. A no-fix result
releases the OBD backlog without inventing coordinates or numeric zero readings.

Subsequent acquisition starts are due every ten minutes, measured from the
previous acquisition start, after the initial attempt completes and the OBD
queue drains. If the initial attempt found a fix, its upload must first be
confirmed. A no-fix startup result permits periodic acquisition even before any
upload succeeds. Each valid periodic fix also creates a GPS-only row and is sent
independently of OBD replies; any such queued fix must be acknowledged before a
further acquisition starts. NTP, active CAN replies, queued telemetry, LTE availability and
modem ownership can defer a periodic acquisition. GNSS temporarily takes LTE offline.
Before the first confirmed fix in a modem power cycle, each attempt searches for
up to 60 seconds, measured from acknowledgment of `AT+QGPS=1`; after a valid fix,
subsequent attempts use 30 seconds.
A no-fix result keeps the longer budget for the next eligible attempt.
The search deadline also applies to an in-flight location query; a late reply
cannot publish a fix. The firmware stops GNSS at a known AT boundary, or resets
the modem when a query is still awaiting its final response, then restores
LTE/registration. Stop, reset and network reattachment take additional time
after the search budget. OBD polling and CAN receive/validation continue during
this process. Command errors or uncertain boundaries use bounded reset/boot recovery.

A cached fix may accompany ordinary OBD batches as root `location` for up to ten
minutes after receipt so LTE reattachment lasting over one minute does not
discard it. Freshness is rechecked after TCP connection setup. No-fix and
expired-fix paths omit `location`; no zero coordinates are substituted. Each
acquisition begins with a cleared fix. Root `location` describes the timestamped
GNSS fix for the envelope, independently of every queued historical OBD record.
Each queued GPS-only row retains its original acquisition time across
LTE recovery and rejected uploads, including delivery delayed beyond the cached
fix's ten-minute lifetime.

`location`: latitude/longitude (degrees), altitude_m, hdop, fix_type, satellites,
utc_hhmmss, date_ddmmyy, age_ms, source=gnss. Use a compatible **passive** GNSS
antenna. The dashboard validates the GNSS UTC/date, attaches root `location` to one
record in the received batch and stores the actual fix time as `gps_timestamp`
(`gpsTimestamp` in API responses). Its nullable database migration must be
applied before running the updated dashboard. Existing record-level GPS payloads
remain supported. HDOP remains dimensionless and is not converted to accuracy in
meters. Direct per-record GPS, including the startup GPS-only row, is also
accepted without an OBD measurement. The map can show the latest received fix
and its acquisition time before any OBD response and retain it when later OBD
records contain no GPS. Unavailable OBD values display as `-`.

Concurrent startup GPS/OBD collection, buffered delivery, the ten-minute GNSS
cadence, position accuracy and location delivery through the updated dashboard
require physical validation. Hardware,
firmware, antenna, RF, SIM and LTE reattachment remain relevant. GNSS and LTE are exclusive; the send
cadence can lengthen.

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
only advertised continuation bits), with at most three attempts per page. The A0 page is needed to preserve main's
standard PID A6 odometer. The first valid PID 00 responder is selected for the
boot; this does not identify its ECU role. Polling supports ten existing PIDs,
with explicit unknown-PID compatibility polling enabled by default (see field-regression
follow-up below). Bit exclusions are unsupported; missing, negative and timed
out responses remain unknown until a valid positive telemetry reply is observed.
Discovery resets when the CAN module is reinitialized.
A 200 ms response deadline and 100 ms minimum request gap reject unrelated,
wrong-ECU, wrong-PID, wrong-service and malformed single-frame payloads. A
four-second acquisition window follows the existing ten-second group interval
after the initial complete round; no OBD record is created until an accepted
measurement has a valid timestamp.
An accepted no-fault Mode 03 reply records DTC count `0` and code string `""`,
including when it is the first acquisition in a row. An unanswered or rejected
DTC request leaves those fields `null`. Before sending, the sketch removes rows
whose ten OBD numeric fields and DTC result are all unavailable and which carry
no valid GPS fix; a second transport guard refuses such unmarked rows. Removal waits for
outstanding CAN replies to finish. A validated zero speed, temperature, RPM or
other measurement is eligible for upload even when no other PID returned data.
The startup GPS-only row is eligible independently of all PID responses.

Root `obd.supported_pids` exposes support state independently of each record's
nullable readings. `distance_traveled` (PID 31) is distance since
DTC clear, not odometer. PID A6 remains `odometer`. Existing Mode 03 DTC read is
retained with selected-ECU, pending-request and single-frame validation; declared DTC count and SF length are checked, hexadecimal digits are preserved. Multiframe DTC is still
marked MULTI with a `null` count and is not reassembled. No Mode 04 clear, other control service or
ISO-TP flow-control frames are added or sent by the cellular example.

UART receive supplies ID + eight bytes but no true DLC/IDE/RTR. Application length
checks cannot prove actual frame length/format. Existing partial UART reads and
synchronous TCP/HTTP/power operations can block and lose traffic. This is not a
lossless logger or a multi-ECU aggregator. Queue is bounded to 12 newest records;
failed sends retain records until that bound. No unverified DBC/ECU-role claims.

## Validation and remaining physical work

Run `bash tests/cellular/run_tests.sh` with `ARDUINOJSON_INCLUDE` and
`WIOCELLULAR_INCLUDE` pointing to installed libraries' `src` directories.
The configured dependencies are ArduinoJson 7.4.3 and WioCellular 0.3.16; record
the installed versions when validating with other compatible headers. This suite
does not invoke the separate historical compatibility test, which reads Git
history. The integration checks extract and compile current sketch functions and
loop, not an old copied loop.
Transport fixtures are WioCellular 0.3.16 at
`9275de1d15f8bd2761f562b8895edc0ee9d1a1c5` with its MIT license.

`pio run` compiles all seven profiles: combined/default, NTP without GNSS, GNSS
without NTP, CAN with HTTP time only, and combined 29-bit/250 kbps, 11-bit/250 kbps and 29-bit/500 kbps. Profiles verify
compilation, not vehicle protocol selection. Build dependency pins and wrapper
are shared. CI runs the three host suites and all target profiles.

Required before claiming hardware/model support: record actual vehicle/year/trim,
protocol and module firmware; capture raw requests and PID bitmaps per family;
compare telemetry/DTC/odometer with an independent scan tool. Test no ECU, multiple
ECUs, negative/late replies, ignition transitions, sustained bus load, missing NTP
notifications, timezone rollover, HTTP fragmentation, no GNSS antenna/fix, slow
TCP, LTE reattachment, modem recovery, no-network idle sleep, CAN wake and heartbeat.
The archived integration tests below ran without a real vehicle, board, antenna,
SIM or ingest server.

## Archived integration validation

These results precede the ten-minute GNSS and empty-record guards described
above. They establish the earlier integration baseline. Compile validation of
the cellular example with those guards is recorded in
[cellular delivery validation](cellular-send.md#validation); physical validation
of the GPS and empty-record behavior remains outstanding.

All three UBSan host suites PASS. All seven PlatformIO 6.2.0 profiles PASS.
Combined default: RAM 12,516 / 237,568 bytes; flash 145,088 / 798,720 bytes.
The framework emits its three existing ignored `used` attribute warnings in
startup_nrf_common.c for each clean target build; there were no build errors.
The archived baseline did not include physical/end-to-end ingest tests because
the hardware and server listed above were unavailable.

## Review follow-up: activity and receive deadlines

Two reported issues were reproduced against head `150b8f0` with failing host
regressions before the fixes. Validated capability, telemetry and DTC responses
now update CAN activity before the UTC availability check. Activity wakes a
sleeping modem independently of whether a timestamped record can be created;
unsynced frames still never create fabricated timestamps or records. Unrelated
frames do not wake the modem. Accepted DTC replies also end the pending request
without requiring UTC.

DTC's 200 ms deadline is now checked after each receive returns, including during
a batch. The fake advances time by up to 100 ms for each unrelated frame before
a delayed reply: late SF/FF responses are rejected, 199 ms is accepted, 200 ms is
rejected, and wrap across millis is covered. The activity regression exercises
unsynced sleep/wake in the actual extracted loop with a simulated power restart.
These are host simulations, not physical UART, modem or vehicle tests.

The HTTP helper's actual reception from WorldTimeAPI remains **NOT TESTED**:
`test_sketch.py` supplies the HTTP datetime with a stub. It tests the current
fallback orchestration, date validation, clock anchoring and AT exclusion, not
actual fragmented socket reads or the public server. That distinction also
applies to the earlier host results above.

## Field regression follow-up (reported against c6eb9f0)

Confirmed with failing regressions against the previous extracted sketch:
1. A missing PID00 response exhausted discovery after one try; all telemetry stayed
   Unknown and was never requested again until CAN reinitialization. Unlike the old
   main's unconditional ten read-only PID requests after ten seconds, that made
   telemetry depend on receiving a capability bitmap.
2. At 30 seconds the empty-data branch fell through into a send even without records.
3. Receiving any TCP bytes was treated as delivery success, so the provided numeric
   `400` rejection cleared the queued records and printed Completed.

These are verified code paths, not proof that this vehicle lost PID00 specifically.
The missing request prints were also a visibility regression: their absence alone
could not establish whether UART transmission occurred. A failed CAN configuration
is another possible cause of no requests; there is no physical evidence here that
it occurred in the reported run. The user's older-version success demonstrates
working legacy telemetry on that setup, not support for every Hustler generation.

### Recovery policy and visibility

`OBD_ALLOW_UNKNOWN_PIDS=1` is the explicit compatibility policy in the sketch. It
restores requests for only the ten existing standard Mode 01 telemetry PIDs after
bounded bitmap discovery (three attempts per page, 200 ms deadline, 100 ms minimum
gap). It does not probe transport, bitrate, diagnostic services or new PIDs. Unknown
is not converted to unsupported or assumed supported: only a matching positive
response establishes observed support. Known bitmap exclusions are still skipped.
Set `OBD_ALLOW_UNKNOWN_PIDS=0` for strict advertised/observed-only polling. Model
coverage remains vehicle_unverified. The first validated responder (bitmap or
telemetry) becomes the consistent ECU source; wrong ECU/PID/service/length and late
responses are still rejected.

Queued PID requests and CAN configuration failures are now logged without device
identity/payload values. Queued means UART submission, not an ECU acknowledgment.
`obd.can_ready` and `obd.unknown_pid_fallback` explain the gate/policy. All required
rate/mask/filter operations must succeed. Driver configuration also checks the
exit-to-data-mode acknowledgment; the old driver discarded that result. Command
response storage and waits are bounded under echo/noise. The driver source's line
endings were normalized while editing it. No setting is automatically swept or
changed on a live bus after a configuration failure; check stage logs and retry
explicit initialization with the verified settings.

### Ingest acknowledgments and hourly heartbeat

The sender refuses empty `data[]` before opening a socket. It sends a separate
heartbeat document at startup as soon as LTE, device identity and validated UTC
are ready, then one hour after each acknowledged heartbeat:

```json
{"imsi":"<device identity>","data":[{"time":"2026-11-06T12:00:00+00:00","type":"heartbeat"}]}
```

The modem wakes for a due heartbeat even without CAN activity. Each idle sleep
is capped at the remaining time to that deadline. Pending CAN replies, GNSS
radio work, LTE reattachment and unavailable UTC defer the send until the shared
modem and a validated clock are available. A failed or unconfirmed send retains
the original heartbeat timestamp and retries at the normal 30-second interval.
Only a positive heartbeat acknowledgment starts the next hour. Accepted OBD
and GPS uploads do not reset that interval, discard a pending heartbeat or
replace its retained timestamp. Queued measurement/GPS rows do not block a due
heartbeat. A ready startup heartbeat is attempted before initial GNSS acquisition;
an already active acquisition finishes before the heartbeat uses the radio.
Heartbeats do not update the vehicle-activity timer or populate any measurement.

The companion dashboard accepts only explicitly marked heartbeat rows without
readings, saves their type, and shows them as `生存確認` on its data
page together with the latest communication and heartbeat timestamps. The page
retries initial device discovery and periodically checks for saved records even
while WebSocket is connected. Unmarked
timestamp-only rows remain invalid. Heartbeats are excluded from measurement
charts and summaries. Apply the dashboard's additive heartbeat database migration
and update the dashboard before flashing firmware that emits this record type.

Local heartbeat validation on 2026-11-06 passed the safe cellular host suite
(NTP, GNSS, startup, OBD, telemetry services, timestamp handling, ACK parsing,
batching, record validation and UART driver). The final extracted-sketch suite
passed its default case and 47 named scenarios, including fifteen heartbeat cases:
immediate ready startup before GNSS, delayed UTC/identity/LTE readiness without
waiting an hour, startup with queued measurements, an hourly schedule independent
of accepted OBD/GPS uploads, retained heartbeat timestamps across measurement
acknowledgments, delivery despite a rejected GPS upload, hourly wake/send,
timestamp retention and retry, registration recovery, pending CAN responses,
continued CAN polling after rejection and without LTE, and sleep deadlines
across timer wrap.

At startup the USB serial log reports `[HEARTBEAT] status=due reason=startup`.
A supported positive response then produces `[HEARTBEAT] status=accepted`.
`status=unconfirmed` means the timestamp remains pending for retry; an unavailable
validated clock is reported as `status=deferred reason=UTC_unavailable`.

The complete sketch also compiled with the installed Arduino toolchain for
`SeeedJP:nrf52:wio_bg770a`, board version 1.0, core 1.5.0, WioCellular 0.3.12
and ArduinoJson 7.4.2: 176,460 bytes of program storage and 12,732 bytes of global
RAM. These installed library versions differ from the PlatformIO pins; the
PlatformIO profiles were not rebuilt for heartbeat validation. Host socket and
modem responses are simulated. No board was flashed and hourly physical
delivery, deployed dashboard behavior and the database migration still require
verification on the target system.

[SORACOM's documented TCP reply](https://users.soracom.io/ja-jp/docs/unified-endpoint/settings/)
is numeric status plus body. Unified status 207 means partial failure; 200 means
all configured destinations succeeded. The parser supports status 200/201/204 with
a complete JSON object/array success body, and documented bare Harvest/Funnel
201/204 only at clean response end. Explicit errors (including 400, 5xx, 207,
JSON error/ng/false) are rejected. Unknown/custom plaintext ACKs, HTTP status-line
responses, oversized or truncated ACKs remain unconfirmed and retain the queue.
No HTTP/1.1 envelope is inferred from the reported `400 {error...}` TCP response.

The exact provided sanitized 400 shape is a regression fixture. Current extracted
`cellularSend` is tested through simulated socket reads with fragmentation, empty
payload gating, rejection retention and confirmed-success clearing. Unlike the
older test, the sender function itself is not stubbed. TCP/modem I/O is still
simulated. The WorldTimeAPI HTTP receive helper remains stubbed as stated above.
The acknowledgment read is bounded to 16 receives / ten seconds / 2048 bytes.
Absent/truncated ACK can mean delivery succeeded but confirmation was lost, so a
retry may duplicate records; no exactly-once delivery or server idempotency is
claimed. The 12-record bound can still evict oldest records during prolonged failure.
Only returned service status is checked; the device cannot independently verify
external backend durability or custom SORACOM configuration correctness.

Payload and response bodies, including the device identity, are no longer printed
by the sender; only byte counts and numeric status are logged. Actual identifiers
were not copied into tests, documentation or PR text. Fixtures use synthetic labels.

### Verification and physical retest

Host regressions cover startup scheduling, absent/negative/malformed/late bitmap,
finite retries and legacy telemetry availability, known exclusions, configuration
failure with no unconfigured transmission, actual UART command/legacy PID byte
layout, empty 30-second send, fragmented numeric 400/207/200,
unknown/truncated ACKs and prior arbitration/sleep/DTC deadline cases. All three
host suites and seven target profiles passed locally. Latest main remains 93af604.
**Physical retest of this patch: NOT DONE.** No board flashing or main merge was
performed. Verify stage logs and PID00 retries/fallback requests on the same setup,
then confirm actual measured data and ingest success without exposing the device
identity. The root causes above are code-confirmed; hardware configuration and
vehicle bitmap behavior still require physical observation.

## Latest main-format comparison and verbose diagnostics

See [actual-main wire/schema comparison](main-wire-compatibility.md) for unchanged
Mode 01 bytes, intentional receiver/Mode 03 differences, the fixed synchronous
work versus pending-response deadline collision, default verbose logs and the
seven explicit build profiles. CAN Mode 03 SF decoding now consumes the declared
DTC count before code pairs; earlier uncounted synthetic DTC fixtures were replaced
with clearly labeled counted CAN fixtures, not inferred vehicle data.


## Fixed-length field responses and stale-millis correction

The supplied PID00 SF declaration 07 is now accepted with bounded length validation
and padding isolation. The verbose sketch's stale local `now` call no longer feeds
an older tick into live uptime. Uptime and coordinator layers additionally reject
stale ordering and preserve real wraps/deadlines. The exact observed field subset,
legacy-formula reproduction of both wrong UTC values, sampling contract and host
regressions are documented in [the comparison guide](main-wire-compatibility.md).
No new vehicle response is claimed; corrected physical retest remains NOT DONE.
