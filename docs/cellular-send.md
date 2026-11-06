# Cellular delivery investigation, 2026-11-06

The Arduino IDE Serial Monitor shows successful LTE connection and NTP before
the first telemetry upload. At 19:44:58, TCP open/connect succeeded and the
device sent a 1,440-byte payload containing three queued records. At
19:44:59.152 the endpoint returned status 400. The queue was retained. The
running firmware printed no response reason, so this trace establishes endpoint
rejection, rather than a failed LTE attachment or TCP connection. A narrow
transcription without identifiers is in `tests/fixtures/field-cellular-reject.json`.

Immediately after rejection, the firmware issued `AT+CFUN=4` to start optional
GNSS. It stopped GNSS about 30 seconds later and restored `CFUN=1`, then repeatedly
polled `AT+CEREG?` while valid CAN data accumulated to the 12-record queue limit.
The old send gate silently returned false while registration remained false,
and the loop still advanced its send timer. GNSS had been enabled by an attempted
upload, even though no upload had been accepted.

## Changes

- After network/time setup and device identification, the firmware makes its
  first bounded GNSS acquisition before normal OBD polling or telemetry uploads.
  A valid fix creates a GPS-only row and is sent as soon as LTE is restored,
  without waiting for an OBD reply. A no-fix attempt returns to normal OBD work.
  Every startup or periodic fix creates its own GPS-only row, so later fixes
  also upload when the vehicle returns no OBD data. Subsequent GNSS acquisitions
  wait for any queued fix's confirmed delivery and an empty OBD queue. A startup no-fix result still
  permits periodic acquisition even before any upload succeeds. Rejected or
  unconfirmed responses never acknowledge records.
- Send deferrals report initialization, AT ownership, or registration state every
  five seconds. A readiness deferral leaves the upload due, so recovery can send
  immediately in the next CAN-idle gap.
- Registration parsing accepts whitespace around complete integer tokens. It
  distinguishes queries from extended notifications, preserves later notification
  ordering, and never treats status 15 as status 1 or 5. A successful query can
  confirm recovery without a new unsolicited notification reaching WioNetwork.
- LTE reattachment after GNSS has a 120-second deadline. Failure enters the
  existing physical-reset recovery path and disables optional GNSS for that boot.
- Until the first confirmed fix after a modem power cycle, each GNSS search has
  a 180-second budget. Subsequent searches after that fix retain the 30-second
  budget. A no-fix result does not shorten the next search. The start log reports
  the active budget as `search_timeout_ms`.
- After the startup attempt, GNSS acquisition starts are due every ten minutes
  from the previous start, after the initial attempt completes and the OBD queue
  drains. If the initial attempt found a fix, its upload must first be confirmed.
  Busy AT/CAN owners or an unavailable LTE connection defer periodic acquisition.
  A cached location retains its original GNSS UTC/date and may accompany OBD
  uploads as root `location` for up to ten minutes after receipt, including after
  LTE restoration lasting over one minute.
- Each GPS-only row uses GNSS UTC seconds as `time` and the full GNSS UTC
  acquisition time, including milliseconds, as `gps_time`. Its `latitude`,
  `longitude` and `altitude` are stored directly on the row; all OBD and DTC
  fields remain `null`. Delivery is due immediately after LTE recovery.
  A rejected or unconfirmed upload retains the row and uses the normal
  30-second retry interval. Periodic GNSS waits for this upload to be confirmed
  and for the queue to drain. The queued row retains its original acquisition
  timestamp even when delivery is delayed beyond the cached-location lifetime.
  OBD collection resumes once the fix is queued; ordinary OBD uploads and idle
  sleep wait for its acknowledgment. The separate GPS document is preserved
  when the bounded OBD queue drops its oldest samples during prolonged failure.
- OBD numeric fields and DTC results start as JSON `null` and change only after
  accepted replies. Unanswered, rejected and unsupported PID readings remain
  `null`; a genuine numeric zero remains `0`. A complete no-fault Mode 03 reply
  records count `0` and an empty code string. Unsupported multiframe DTC replies
  retain the `MULTI` marker with a `null` count. After pending CAN replies finish,
  rows with no OBD measurement, DTC result or valid GPS fix are discarded; the
  sender also rejects such unmarked rows before opening a socket. A zero-speed-only reading
  and a GPS-only reading are both eligible.
- At startup, as soon as LTE, device identity and validated UTC are ready, the
  device sends a separate heartbeat before beginning initial GNSS acquisition.
  One hour after each acknowledged heartbeat, the modem wakes and sends another
  separate `data` row containing only `time` and
  `type: "heartbeat"`. It never inserts zero-valued vehicle readings. A missing
  validated UTC clock, busy modem/CAN owner or unavailable network defers delivery;
  an unconfirmed send retains its timestamp for a 30-second retry. Idle sleep
  ends by the hourly deadline. Only successful heartbeat delivery resets that
  deadline; normal OBD/GPS uploads cannot postpone or satisfy a heartbeat. A
  heartbeat does not count as vehicle activity. The companion dashboard stores and labels these
  rows on the data page while excluding them from measurement charts and summaries.
  Deploy its heartbeat database migration and dashboard update before flashing
  the updated device firmware.
- Each send contains a complete JSON envelope with a prefix of records fitting
  a conservative 1,024-byte application budget. Only that acknowledged prefix is
  removed; remaining records are sent after about one second when the AT/CAN
  owners are idle. Failed uploads keep the normal 30-second retry interval.
- Negative replies are collected across reads, up to 2,048 private bytes and the
  existing receive deadline. `LTE-REJECT` prints status, byte count, and a fixed
  category such as `request_shape`, `payload_decode`, `data_empty`, `time_format`,
  `payload_shape`, `authentication_failed`, or `endpoint_rejected`. Server-supplied
  strings, identifiers, URLs, and response bodies are never printed.

## GPS diagnostics

GPS acquisition and delivery have separate log stages. Polling commands alone
do not establish that a fix was acquired or sent. Valid coordinates are printed
to the local USB serial monitor; device identity, authentication, full payloads,
server response bodies and malformed modem text remain redacted.

| Log | Meaning |
| --- | --- |
| `GPS-ACQUIRE status=started` | An acquisition attempt has started; `search_timeout_ms` gives the configured search window. |
| `GPS-POLL status=no_fix cme_error=516` | The modem has no fix yet. Other errors include their numeric code; invalid and repeated responses have distinct statuses. |
| `GPS-FIX status=acquired` | A validated fix received its matching final OK. The log includes UTC acquisition time, latitude, longitude, altitude, HDOP, satellites and fix type. |
| `GPS-QUEUE status=queued` | The acquired fix was copied into the retained GPS upload document. `GPS-POSITION source=queued` prints its stored coordinates. |
| `GPS-UPLOAD status=waiting` | A queued fix is waiting for the stated modem, LTE, CAN or retry condition. The row is retained; waiting logs are limited to changes or five-second intervals. |
| `GPS-PAYLOAD` | Describes the actual bounded JSON batch. `direct_gps_rows` counts coordinate-bearing rows; `root_location=1` means the envelope also contains a cached position. These fields are separate because the cached position can replace a row's coordinates on ingestion. |
| `GPS-POSITION source=record` / `source=root_location` | Coordinates and acquisition time actually included in that batch. A root location also shows its cache age. |
| `GPS-UPLOAD status=awaiting_ack` | The modem accepted the batch write. Endpoint acceptance is still pending. |
| `GPS-ACK status=accepted` | A supported positive endpoint acknowledgement was received. GPS counts describe this acknowledged batch, not the rest of the queue. |
| `GPS-ACK status=unconfirmed` | The endpoint rejected the batch or acceptance was not established; records remain queued. Socket/batch/send failures also explicitly report retained records. |
| `GPS-ACQUIRE status=no_fix_timeout` / `GPS-ERROR` | Search timeout, command error/timeout, LTE reattachment timeout or recovery. `disabled=1` means further GNSS attempts are disabled for this boot. |

`direct_gps_rows=0 root_location=0` explicitly identifies an OBD-only batch.
A positive endpoint acknowledgement does not independently prove database
storage or map rendering. The numeric no-fix code is documented in Quectel's
[GNSS application note, Table 6](https://www.quectel.com/content/uploads/2024/04/Quectel_BG77xA-GLBG95xA-GL_GNSS_Application_Note_V1.0.pdf).

At 22:24 JST on November 6, a read-only check of the configured dashboard
database found 1,377 telemetry rows and no rows with coordinates, while the
latest telemetry receipt was 22:23:57 JST. Both the GPS timestamp and nullable
OBD migrations were confirmed applied in the preceding 22:20 check. This
establishes missing stored GPS
data at that time; it does not distinguish a failed fix from a rejected GPS
upload. The supplied 22:14 trace shows startup polling but no fix or GPS-bearing
payload, so the new stage logs are needed to distinguish those outcomes.

The actual Serial Monitor was inspected again for the 22:30 startup. The first
two queries returned 505 (session not active), followed by 516 (no fix). At
22:30:41.220 the old search limit ended the attempt with
`status=no_fix_timeout cme_error=516 polls=29 elapsed_ms=30000 disabled=0`.
GNSS stopped, LTE was restored with `CFUN=1`, and registration remained 0.
The later `reattachment_timeout elapsed_ms=120000 disabled=1` entered the
existing modem recovery path. This run acquired no coordinates; polling alone
was not a GPS upload. No registration-parser defect was found.

The 30-second first-fix limit was shorter than the 39-second average cold-start
acquisition reported in Quectel's
[BG770A-GL Low Power Design Guide, Table 13](https://quectel.com/content/uploads/2024/04/Quectel_BG770A-GL_Low_Power_Design_Guide_V1.0.pdf).
The longer initial budget addresses that premature cutoff; it does not guarantee
a fix or resolve the separately observed LTE reattachment failure. Seeed's
[hardware manual](https://seeedjp.github.io/Wiki/Wio_BG770A/hardware.html)
specifies a separate passive GNSS antenna and does not support active antennas.

## Endpoint contract and the original 400

The local dashboard source at `OBD2-Tracker-Dashboard/schemas/telemetry.schema.ts`
accepts the firmware's UTC offset format, per-record GPS and root `location`.
It validates a root fix's UTC/date, maps its coordinates onto one record in the batch, and preserves
the fix time independently of that record's OBD time. HDOP is not treated as
accuracy in meters. The nullable `gps_timestamp` database migration is required
for the updated dashboard; API responses expose this as `gpsTimestamp`. The map
retains the latest received location and shows when it was acquired, including
when subsequent OBD records have no location. Dashboard ingestion preserves
nullable OBD fields and accepts genuine zero readings. A GPS-only row with valid
coordinates and acquisition time is accepted even when every OBD and DTC field
is `null`; it can place the vehicle on the map before the first OBD response.
Its nullable OBD database migration must be applied before deploying the updated
server. The dashboard
shows unavailable readings as `-` and keeps zero values visible. Previously stored
zeros cannot be reliably classified as missing or measured and are preserved.
These source contracts do
not establish which code or forwarding configuration is deployed.

[SORACOM Beam's TCP-to-HTTP reference](https://users.soracom.io/ja-jp/docs/beam/tcp-http/)
documents automatic Base64 `payload` wrapping. The device therefore continues to
send its existing JSON directly. Adding a second Base64 envelope would change the
decoded contract. The same reference states that ordinary TCP data can be split
or combined into forwarding requests. WioCellular also splits payloads exceeding
its 1,460-byte send size across writes. Keeping batches small avoids that client
multi-write path; a smaller application batch does **not** guarantee network
message boundaries. The observed 1,440-byte upload was below the client limit,
so fragmentation is a possible explanation for 400, not a confirmed diagnosis.

The updated board run described below returned accepted uploads. The original
400 body was not logged, so its precise rejection message cannot be recovered.
For a future rejection, `payload_decode` points to forwarded JSON/decode
failure; `payload_shape` or `time_format` points to payload validation;
`request_shape` points to the forwarding envelope. Unknown replies stay rejected
and report `endpoint_rejected` rather than guessing success. No SORACOM group
settings, deployed dashboard code, or device identifiers were changed.

## Validation

Run `bash tests/cellular/run_tests.sh` with `ARDUINOJSON_INCLUDE` and
`WIOCELLULAR_INCLUDE` pointing to the installed libraries' `src` directories.

- The GPS-first startup code with QNTP UTC correction, GPS diagnostics and the
  extended initial fix window compiles offline with Arduino CLI 1.2.0 for Wio
  BG770A board 1.0 (`SeeedJP:nrf52:wio_bg770a:board_version=1_0`), using installed
  ArduinoJson 7.4.2, WioCellular 0.3.12 and SeeedJP nRF52 Boards 1.5.0. It uses
  174,700 bytes of program storage and 12,668 bytes of global RAM. The core emits
  its three existing ignored `used` attribute warnings; compilation has no errors.
  Verification uses the installed dependencies and does not upload firmware.
- The complete cellular host suite passes with GPS-first startup, QNTP UTC parsing and GPS diagnostics.
  Initial-fix regressions now prove that polling continues past 30 seconds,
  a first fix at about 40 seconds is uploaded and acknowledged through the
  extracted sketch, no-fix attempts stop at 180 seconds and retain that budget
  for a retry, and searches after a valid fix use 30 seconds. Timer rollover is
  covered. These are synthetic modem responses, not a physical GPS fix.
  Actual extracted-sketch regressions cover a first GPS upload before queued
  OBD data and without a prior clock, GPS-only delivery without an ECU response,
  retained fixes and 30-second retries after rejection, and ten-minute periodic
  acquisition. No-fix cases release CAN collection even while LTE registration
  is unavailable; a later periodic fix can upload without a prior successful
  upload. A validated fix survives a subsequent GNSS stop failure and modem reset
  with its original acquisition time. NTP ownership remains serialized before
  GNSS. Existing cases also cover fragmented safe errors, registration
  deferral/resumption, prefix-only queue acknowledgment, prompt backlog draining,
  nullable JSON fields, confirmed no-fault DTC replies, empty row exclusion,
  genuine zero readings, DTC information, and CAN/UTC behavior.
- GNSS session and real AT-dispatcher tests cover whitespace, extended/interleaved
  registration notifications, missing notifications, reattachment timeout and
  timer wrap. Endpoint and batch tests cover privacy, fragmentation, byte bounds,
  preserved record values, rejected queues, and allocation failures.
- The record regression sends each of the ten OBD numeric fields at genuine
  zero through JSON batch serialization and parsing while checking the other
  fields remain `null`. It rejects strings, booleans, NaN and infinity as
  measurements. The actual-sketch regression sends a validated zero-speed-only
  response and a confirmed no-fault DTC reply, checks malformed and timed-out RPM
  leave readings unavailable, and rejects a completely unmeasured row.
- The NTP regression uses a synthetic QNTP response with UTC clock digits
  `13:01:57` and timezone indicator `+36`. It checks that clock formatting,
  record time, safe `NTP-SAMPLE` diagnostics and the actual serialized upload
  all retain `13:01:57+00:00`. A separate HTTP `22:01:57+09:00` case converts
  its ISO offset once to the same UTC instant. This synthetic response is not
  substituted for the original modem result, which the field log did not print.
- GPS diagnostic regressions distinguish a candidate response from a fix
  accepted after final OK, and cover no-fix code 516, malformed-response privacy,
  duplicate fixes, acquisition/command/reattachment timeouts and disabled GNSS.
  Actual-sketch tests exercise valid zero coordinates, GPS queue retention,
  failed socket open/connect/write, rejected acknowledgements, root locations,
  and bounded batches whose pending GPS row is outside the sent prefix.
  An accepted OBD-only batch explicitly reports zero GPS data.

The NTP offset correction, GPS-first startup, the ten-minute cadence and location reporting through the
updated dashboard require a new physical end-to-end check. The physical retest
below establishes earlier delivery and LTE recovery behavior, not validation of
these GPS and nullable-reading changes.

Host response/registration samples are synthetic unless explicitly listed in
the observed field fixtures.

## Physical retest

With the user's approval, Arduino CLI uploaded the built sketch through COM3
and the Arduino IDE Serial Monitor was reconnected. The board returned status
200 at 20:00:23.226, 20:00:25.140, and 20:00:27.062 for one-record JSON batches
of 762, 762, and 761 bytes. The queue changed from three records to zero only
after those acknowledgments. GNSS then started, establishing the then-current
delivery gate on the actual device. This run predates GPS-first startup.

After GNSS stopped and CFUN=1 was restored, repeated explicit CEREG queries
reported registration 0. This device therefore reproduced the reattachment
failure rather than a whitespace-only parser failure. The 120-second recovery
deadline triggered a modem reset; the monitor showed boot/flow-control recovery,
CFUN=1/CEREG restoration, and LTE online again at 20:03:17.033. Further uploads
returned 200 while the accumulated queue drained. Optional GNSS remains disabled
for the rest of this boot after this recovery.

Sustained delivery was also observed at 20:05:06.015, 20:05:07.936, and
20:05:09.856. The last acknowledgment removed one record and reported
`remaining=0`; the following CAN summary also reported `queue=0`. New CAN
measurements continued arriving afterward.

Quectel's [GNSS application note, pages 11–15 and 67](https://www.quectel.com/content/uploads/2024/04/Quectel_BG77xA-GLBG95xA-GL_GNSS_Application_Note_V1.0.pdf)
explains that GNSS and WWAN share the radio stack and that network attachment can
remain blocked by GNSS priority. Priority/loaded-state queries were not captured
on this run, so that internal cause is not established. The observed failure and
successful reset recovery are established by the monitor trace. The current
fix bounds recovery and restores telemetry; it does not change persisted GNSS
priority settings.

Status 200 establishes positive endpoint acknowledgments. The dashboard database
was not independently queried. The smaller batches' success is evidence for the
delivery fix, not a packet capture proving the original segmentation mechanism.
