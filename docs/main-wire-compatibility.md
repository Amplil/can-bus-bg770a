# Comparison with the working main

Compared actual source at main `93af60465fbfec11a852343c302180161a5ae251`
with the integration branch. `tests/compat/test_main_compatibility.py` compiles
both real driver versions and executes their requests/configuration commands.
It also compiles both actual record initialization/update functions and compares
required JSON values/types. All response bytes in that test are explicitly
**synthetic host fixtures**, not reconstructed vehicle captures. No device
identity from a real log is included.

| Item | Main 93af604 | Current integration | Compatibility / reason |
| --- | --- | --- | --- |
| CAN default | 500 kbps, standard 11 bit | Same | Default profile remains unchanged |
| UART | Serial1, 9600 baud | Same | Same baud constant and configuration wire bytes |
| Functional request ID | 0x7DF, ext=0, RTR=0 | Same default | 29-bit 0x18DB33F1 requires explicit configuration |
| Mode 01 application bytes | `02 01 PID 00 00 00 00 00` | Same | Same PCI=2 and zero padding, all ten PIDs tested against actual main |
| Grove UART TX | 4-byte big-endian ID + ext + RTR + 8 data = 14 bytes | Same | Actual driver captures match byte-for-byte |
| Grove UART RX | 4-byte ID + 8 data = 12 bytes | Same | Both drivers decode the same synthetic fixture; no DLC/IDE/RTR metadata is added/invented |
| Telemetry PIDs/order | 0C,0D,05,04,0F,11,31,A6,42,46 | Same now | Previously A6 was last; reordered to remove an unnecessary difference |
| PID00 discovery | None | Bounded initial 00..A0 bitmap discovery | Adds read-only requests and startup delay; three attempts/page, then explicit unknown-PID compatibility policy. Not an all-model capability claim |
| PID spacing / batch | >50 ms between requests, one group after each 10 s | >=100 ms after reply/timeout, up to 200 ms pending, four-second window followed by ten-second group gap | Slower bounded polling avoids overlap; it is not the same timing as main. Response deadline can reject a slow ECU; physical latency remains unknown |
| Received data | DTC service check; otherwise uses byte 2 as PID without correlating an outstanding Mode 01 request | Selected response ECU, service 41, exact pending PID, expected SF application length, processing deadline | Intentional protection against unrelated/malformed/stale frames. No inference that main's broader acceptance was correct |
| ECU selection | Can mix any received IDs into one record | First validated responder remains selected until CAN reinit | Consistent source; can omit PIDs available only on another ECU. Diagnose `DROP selected ECU mismatch`; no ECU-role assumption |
| Configuration exit | Sends AT+Q but discards its return value | Requires command and exit acknowledgment | Same positive wire commands; no ACK now blocks polling. A real module missing ACK despite working data mode could explain a difference, but is unconfirmed. Do not disable all checks blindly |
| Mode 03 request | `02 03 00 00 00 00 00 00` | `01 03 00 00 00 00 00 00` | Intentional one-byte service request: Mode 03 has no PID. Only UART byte 6 (PCI) differs; this cannot change Mode 01 RPM request bytes |
| Mode 03 decode | Treats bytes after service as code pairs | Counted ISO 15765 single-frame response: service 43, count, code pairs | Main's uncounted interpretation is not restored. Count/length mismatch is rejected. Multiframe remains MULTI, not reassembled |
| Server JSON | Identity field + data array; time string, ten numeric telemetry fields, DTC string/count | Same required keys/types/conversions; optional provenance/location/OBD metadata | Actual schema/update functions compare equal on synthetic Mode 01 values. UTC offset is truthful rather than forcibly relabeled; strict backend acceptance of extra keys is still untested |
| Empty data / rejected send | May send empty data, clears on received bytes | Never sends empty data; clear only on supported positive service ACK | Needed for reported endpoint rejection; heartbeat intent is locally pending rather than fabricated data |

ELM Electronics' [manufacturer data sheet, p.34](https://www.elmelectronics.com/wp-content/uploads/2016/07/ELM327DS.pdf)
shows Mode 03 as a service without a PID and describes the CAN response's added
DTC-count byte. The one-byte request and counted SF decoding follow that format;
this reference does not prove a particular ECU accepts it. Legacy malformed or
uncounted responses are not guessed into valid DTCs.

## Confirmed deadline/ownership bug and minimal fix

The previous integration head could queue Mode 01 and then enter synchronous
TCP open/connect/receive in the same loop. A 500 ms host-simulated open causes
the pending request to expire before the next CAN service even though a frame
could have been queued in UART during that interval. Main had no matching
200 ms processing deadline, so its delayed read did not drop for that reason.
This code-path difference is reproduced by a failing actual-sketch regression
before the fix. It is a plausible cause of field loss, **not a confirmed cause
on the vehicle**, because the new field log is unavailable.

Now CAN is serviced first. Pending Mode 01 or Mode 03 postpones synchronous TCP,
identity/time queries, power transitions, new modem commands and physical modem
reset. Existing async AT input is dispatched while waiting. The sender/time
helpers also have defensive entry guards. Once a response is processed or its
200 ms timeout completes, synchronous work may resume in an idle gap. The actual
sketch tests cover deferral and resumption, including a 500 ms simulated socket
open; they do not claim real UART arrival timestamps. Synchronous work started
when no OBD request is pending can still pause new sampling and lose unrelated
traffic. This patch does not make all socket/power operations asynchronous.

## Default verbose diagnostics

The normal `seeed_wio_bg770a` profile prints `profile=main-compatible-diag-v1`.
No special debug build or interactive command is needed. Logs include:

- uptime milliseconds and valid current UTC where available; clock source/quality,
  NTP/GNSS named state changes, safe AT command writes and time readiness/failure;
- each configuration result with timestamp, bitrate, ID width, request ID and UART
  baud; missing exit ACK explicitly says data mode is unconfirmed;
- request ID/PID and all eight data bytes, UART queue result, pending wait, bitmap
  attempts and timeouts; queueing is not proof of CAN bus transmission/ECU receipt;
- actual returned CAN ID and eight bytes, ACCEPT telemetry/bitmap/Mode03 or DROP
  reasons: responder, selected ECU, service, PID, PCI/application length, no pending
  request or processing deadline; accepted activity with no UTC is separately logged;
- queue additions/clears, LTE deferral/open/connect/status, and a five-second total
  summary (UART queued/failed, RX, accepted types, rejection counters, suppression).

Accepted frames and TX events are not suppressed. Under heavy unrelated traffic,
raw rejected-frame details are limited to 64 lines/second; suppression totals and
all rejection counters remain visible. This protects diagnostic USB output from
turning heavy bus traffic into unbounded output. Verbose logging can still affect
timing and must be measured on hardware. Device identity, authentication and server
payload/response bodies remain redacted. CAN raw bytes are printed by the explicit
user request, not substituted from a known vehicle example.

## Explicit transport choices and hardware limits

Seven compilation profiles cover independent modem features plus all four explicit
11/29-bit and 250/500 kbps CAN configurations. Building them does **not** send all
combinations, select a vehicle model or probe a live bus. The default stays
500 kbps / 11 bit. Use only settings verified for the actual vehicle.
K-line/J1850 need a compatible physical interface, not a software CAN fallback.

Module supply/revision and UART logic voltage are physically unverified. The
[Longan hardware reference](https://docs.longan-labs.cc/114992978/#hardware-connection)
contains a 5 V supply requirement that differs from some sales descriptions.
No voltage change or direct-connection safety is recommended here; identify the
actual module revision and logic levels before hardware changes. Its library is
a wire reference, not a drop-in replacement adopted by this patch.

**Physical retest NOT DONE. No flash or merge.** Obtain the new verbose trace to
distinguish configuration blocking, UART queueing, raw RX with rejection, true
absence of returned frames, and deadline/synchronous-work deferral. Host fixtures
prove code behavior and format comparison only, not all-Hustler interoperability.

UART queue success now checks that all 14 bytes were accepted by HardwareSerial;
main ignored the return from write(). A partial write logs unconfirmed framing and
stops further tracker requests until reinitialization instead of claiming success.
Both the current driver DTC wrapper and cellular request use the one-byte Mode 03
service. These changes do not alter a successful Mode 01 UART frame.
