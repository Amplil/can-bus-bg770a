# Independent response timestamps and periodic GPS

## Record contract

Each accepted Mode 01 response or accepted Mode 03 response creates its own sparse row. `time` is the UTC estimate captured immediately after `can.receive()` returns, before tracing, deadline processing or decoding. It is **response-receipt time**, not the ECU's internal measurement time, which this protocol does not expose. A row contains only fields supplied by that response. A measured zero remains zero; unavailable fields are omitted. The existing unsupported multi-frame DTC marker remains `dtc_codes: "MULTI"`, `dtc_count: null`.

Example rows from separate responses:

```json
{"time":"2026-11-06T12:34:56.730+00:00","engine_rpm":2000}
{"time":"2026-11-06T12:34:56.850+00:00","vehicle_speed":0}
```

Milliseconds retain clock resolution; they do not claim millisecond clock accuracy. NTP still anchors to the received whole-second UTC sample. HTTP fallback retains the existing whole-second anchor. Network delay and the source's accuracy still apply. Resynchronization may step UTC backwards. Equal timestamps are valid and are never incremented to manufacture uniqueness. Retried/queued rows keep their original timestamp and values. Downstream analysis must use stable record IDs and cannot assume timestamp uniqueness or complete rows. Backend validation must accept milliseconds before this firmware is deployed.

GPS remains an independent row with the fix's exact `gps_time`. Cached root `location` is no longer added to OBD envelopes. The envelope and existing endpoint acknowledgment semantics remain unchanged.

## Bounded retention

- Synced OBD queue: 120 sparse rows, approximately twelve ten-PID rounds before DTC traffic. Oldest row is evicted on capacity exhaustion with `QUEUE-DROP-OLDEST`, one logged loss per eviction. This is bounded RAM buffering, not durable storage.
- Each send remains at most 1024 serialized bytes including the complete envelope. Only an acknowledged prefix is removed. The batch builder copies only the fitting prefix, not the entire backlog. Failed/unconfirmed sends retain all rows, subject to subsequent bounded-capacity eviction.
- UTC unavailable: accepted raw responses retain extended monotonic receipt milliseconds, CAN ID and eight response bytes in a separate 120-entry RAM ring. `UNSYNCED-RETAIN` identifies this condition; overflow evicts the oldest and reports cumulative `dropped`. The host struct occupies under 3KB. These records are not uploaded or retrospectively assigned UTC. Current backend fields do not persist an unsynced monotonic/raw record. They are lost at reboot or ring overflow; a new backend contract is needed for durable unsynced acquisition.
- Allocation failures are logged. All seven configured target builds pass. Static RAM is 15,628 of 237,568 bytes; flash is 177,248–177,680 of 798,720 bytes. Runtime heap/stack headroom and long-running behavior still require physical verification; host allocator measurements are not board heap measurements.

## GPS defects addressed

The periodic scheduler previously required an empty OBD queue. Active driving continually replenishes this queue, so periodic GPS could starve. Periodic acquisition now proceeds at a safe CAN/AT boundary even with queued OBD responses.

GNSS recovery previously left `disabled` latched for the entire boot. A completed recovery now rearms GNSS only after confirmed registration and a ten-minute cooldown. This includes registration arriving later via URC and explicit command rejection followed by successful stop/online restoration. Unknown/timed-out AT outcomes still require physical reset. A pre-reset registration notification cannot rearm GNSS. Repeated failures remain subject to recovery and cooldown; existing boot/reset retries themselves are not changed.

These are reproduced code defects. They do not prove which firmware was flashed or what happened during the user's latest drive. No vehicle flash, deployment or physical GPS test is performed by these host tests.

## Host verification

```sh
ARDUINOJSON_INCLUDE=../deps/ArduinoJson/src \
WIOCELLULAR_INCLUDE=../deps/WioCellular/src \
bash tests/cellular/run_tests.sh

ARDUINOJSON_INCLUDE=../deps/ArduinoJson/src \
COMPAT_BASELINE_DIR=../compat-baseline \
bash tests/integration/run_tests.sh
```

`COMPAT_BASELINE_DIR` is optional for an explicitly supplied, verified snapshot of pinned compatibility commit `93af60465fbfec11a852343c302180161a5ae251`. Normal checkouts retain `git show` against that commit. Compatibility asserts preserve UART/PID/decoder behavior while explicitly accounting for milliseconds and unavailable DTC fields.

Tests exercise the extracted real sketch with ArduinoJson and the upstream AT transport, including distinct/equal receipt milliseconds, logging delays, true millis wrap, backwards clock resync, immutable retry, zero values, sparse queue retention and 1024-byte batches, no cached root GPS, startup/periodic GNSS with OBD backlog, unsynced bounded retention, and recovery cooldowns. Fixtures are synthetic, not new vehicle evidence.

## Target build verification

All seven PlatformIO environments passed compilation and linking: `seeed_wio_bg770a`, `ntp_only`, `gnss_only`, `can_only`, `can_29bit_250`, `can_11bit_250`, and `can_29bit_500`. PlatformIO 6.2.0 used the pinned Nordic platform, GCC ARM 12.3.1, WioCellular 0.3.16, and ArduinoJson 7.4.3. Build telemetry was disabled. No upload/flash command was run. Static size checks do not establish runtime memory stability or vehicle correctness.
