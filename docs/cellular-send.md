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

- Optional GNSS now starts only after a confirmed successful upload and after
  the remaining queue is empty. A rejected first upload keeps LTE available for
  retries. Rejected or unconfirmed responses never acknowledge records.
- Send deferrals report initialization, AT ownership, or registration state every
  five seconds. A readiness deferral leaves the upload due, so recovery can send
  immediately in the next CAN-idle gap.
- Registration parsing accepts whitespace around complete integer tokens. It
  distinguishes queries from extended notifications, preserves later notification
  ordering, and never treats status 15 as status 1 or 5. A successful query can
  confirm recovery without a new unsolicited notification reaching WioNetwork.
- LTE reattachment after GNSS has a 120-second deadline. Failure enters the
  existing physical-reset recovery path and disables optional GNSS for that boot.
- Each send contains a complete JSON envelope with a prefix of records fitting
  a conservative 1,024-byte application budget. Only that acknowledged prefix is
  removed; remaining records are sent after about one second when the AT/CAN
  owners are idle. Failed uploads keep the normal 30-second retry interval.
- Negative replies are collected across reads, up to 2,048 private bytes and the
  existing receive deadline. `LTE-REJECT` prints status, byte count, and a fixed
  category such as `request_shape`, `payload_decode`, `data_empty`, `time_format`,
  `payload_shape`, `authentication_failed`, or `endpoint_rejected`. Server-supplied
  strings, identifiers, URLs, and response bodies are never printed.

## Endpoint contract and the original 400

The local dashboard source at `OBD2-Tracker-Dashboard/schemas/telemetry.schema.ts`
accepts the firmware's current UTC offset format and strips optional metadata.
The actual installed schema passed a synthetic current-format record. This
does not establish which code or forwarding configuration is deployed.

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

- Arduino CLI compilation for Wio BG770A board 1.0 with installed WioCellular
  0.3.12 and SeeedJP nRF52 Boards 1.5.0 passed.
- Actual extracted-sketch regressions cover rejected-first-send GNSS exclusion,
  fragmented safe errors, registration deferral/resumption, prefix-only queue
  acknowledgment, and prompt backlog draining, alongside existing CAN/UTC checks.
- GNSS session and real AT-dispatcher tests cover whitespace, extended/interleaved
  registration notifications, missing notifications, reattachment timeout and
  timer wrap. Endpoint and batch tests cover privacy, fragmentation, byte bounds,
  preserved record values, rejected queues, and allocation failures.

Host response/registration samples are synthetic unless explicitly listed in
the observed field fixtures.

## Physical retest

With the user's approval, Arduino CLI uploaded the built sketch through COM3
and the Arduino IDE Serial Monitor was reconnected. The board returned status
200 at 20:00:23.226, 20:00:25.140, and 20:00:27.062 for one-record JSON batches
of 762, 762, and 761 bytes. The queue changed from three records to zero only
after those acknowledgments. GNSS then started, establishing the new delivery
gate on the actual device.

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
