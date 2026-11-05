# Hustler compatibility: unverified vehicle coverage

This change improves standard Mode 01 CAN telemetry handling. It does **not**
certify any Hustler model or establish its protocol, bitrate, ID width or PIDs.

| Generation | Candidate families | Vehicle validation |
| --- | --- | --- |
| First (January 2014 launch) | MR31S, MR41S (R06A) | Unknown |
| Second (January 2020 launch, including later refreshes) | MR52S (R06A turbo), MR92S (R06D NA) | Unknown |

Suzuki launch sources: [2014](https://www.suzuki.co.jp/release/a/2013/1224/),
[2020](https://www.suzuki.co.jp/release/a/2019/1224/),
[2026 refresh](https://www.suzuki.co.jp/release/a/2026/0527/).
An OBD-II connector or aftermarket device compatibility is not evidence of
standard Mode 01 CAN support. No per-family transport mapping is encoded here.

## Configuration and behavior

The inherited default remains 500 kbps / 11-bit. It is an example default,
not a Hustler specification. Set `OBD_CAN_RATE` to `CAN_RATE_250` or
`CAN_RATE_500` and `OBD_CAN_EXTENDED` to 0 (11-bit) or 1 (29-bit) only after
verifying the actual vehicle. There is no active bitrate/protocol autodetection.
The Grove GD32E103 UART CAN module cannot implement K-line or another non-CAN
physical layer through software. Different hardware would be required.

Only functional Mode 01 requests are sent (0x7DF or 0x18DB33F1). Discovery
requests PID 00, then 20 and 40 only when the previous bitmap advertises the
continuation bit. At most three discovery requests occur per boot; timeout or
negative response leaves the unobserved range unknown. Restart to retry.
The first valid PID 00 responder is selected for that boot. Other ECU responses
cannot overwrite its data; this is a consistent source policy, not an engine ECU
identification rule. A vehicle with required PIDs on multiple ECUs needs further
design and logs. No write/control services or ISO-TP flow-control frames are sent.

Only advertised telemetry PIDs are polled. Matching responder, Mode 01 positive
service, requested PID and exact expected single-frame application length are
required. Unrelated, malformed, negative and late responses do not complete the
request or update vehicle data. A 200 ms deadline and 100 ms minimum gap bound
requests; polling is cooperative rather than a per-PID wait loop. Deadlines
remain correct across millis wrap. No response is never treated as unsupported.
The added `obd.supported_pids` fields expose unknown/supported/unsupported.
Existing vehicle values retain their acquisition timestamp and may become stale;
support is not a freshness assertion. Existing JSON keys and modem flow remain.
`distance_traveled` is retained for consumers but its `meaning` explicitly says
`distance_since_dtc_clear` (PID 31), not odometer.

The module's existing receive API supplies ID plus eight bytes, **without actual
DLC, IDE or RTR**. Application length checks cannot prove true frame length or
frame format. UART partial-read timeout and synchronous cellular operations can
still block and lose traffic; this is not a lossless logger. Raw frames are
classified Unknown because there is no Suzuki DBC. Never infer ECU roles from
arbitrary CAN ID ranges. No physical hardware was flashed or tested.

## Required physical validation

For each family/year/engine/trim record vehicle identification, authoritative
protocol specification, wiring, CAN rate and ID format; confirm module firmware
and frame metadata limitations. Capture raw requests/replies, selected ECU,
PID 00/20/40 bitmaps, missing/negative replies, and compare each decoded value
against an independent diagnostic tool. Test idle/load, ignition transitions,
no ECU/no reply, multiple responders, LTE interruption and sustained traffic.
Do not mark a model supported until those records establish the claimed coverage.

Host checks: `bash tests/obd/run_tests.sh` (UBSan).
Target compile: `pio run -e seeed_wio_bg770a`.

## Integration order

Independent base: main 72ebe58e061a759a715c3ae66d353eeea4e729f1.
Keep [NTP #2](https://github.com/Amplil/can-bus-bg770a/pull/2) first (user priority),
then reconcile this polling/validation change, then optional
[GNSS #3](https://github.com/Amplil/can-bus-bg770a/pull/3).
All touch the cellular example, JSON array initialization and PlatformIO setup.
Preserve NTP timestamps on accepted/raw frames, and use this single validated
receiver during GNSS waits instead of its current unvalidated receive path.
Retain GNSS send-failure queue behavior and debug drain removal. NTP and GNSS
still need one shared modem arbiter and combined tests. No experimental combined
modem code or either draft branch is included here. Nothing is merged.
