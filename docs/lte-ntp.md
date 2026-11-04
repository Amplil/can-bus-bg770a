# LTE NTP candidate for the CAN tracker

This change adds LTE NTP timestamps without GNSS or PPS. Hardware behavior and timing accuracy are not verified. Do not treat host tests or a successful firmware build as a hardware release approval.

The selected cellular example preserves its existing uptime `timestamp` and `session_start` fields. Each CAN/vehicle record also stores `utc_epoch_s`, `time_source` and `time_sync_state`. UTC is null before the first valid synchronization; one-second representation does not guarantee one-second accuracy. Old records are not rewritten on resynchronization. Upload metadata reports the latest synchronization age/error separately.

NTP requests use the configured PDP context and `time.cloudflare.com` (editable in the sketch), port 123, through the BG770A `AT+QNTP` command. The initial network/PDP setup is unchanged. Only cached network-registration state is consulted before a request; this is not a guarantee that the PDP context is still active. Command rejection or an NTP error leaves CAN collection running.

## AT ownership and failure behavior

`writeBinary` sends the short AT line through the library's existing public API. `doWork(0)` routes echo, final response, NTP results and unrelated URCs through the existing dispatcher. No `executeCommand`, `queryCommand` or synchronous PDP query is used by NTP. The matching command echo is required before accepting OK. A successful NTP result and command OK are both required before using the sample. The 10-second acknowledgement deadline and 130-second completion deadline use one monotonic start time rather than restarting on every incoming line.

Only one NTP transaction is active; cellular socket sends are deferred while it owns the AT channel. Successful NTP refreshes after six hours; definite command/result failures wait one minute. Calendar quality becomes stale after 24 hours without success.

If the modem has acknowledged the command but no NTP result arrives by the deadline, NTP is disabled for this boot and delayed QNTP results are ignored. The AT boundary is already complete, so cellular transmissions can resume without starting another NTP request. Existing valid time remains available as holdover.

If command echo/acknowledgement is missing, ownership is uncertain. The candidate waits for a matching late echo and its final OK/ERROR, then disables NTP for the boot and releases the channel. If that final boundary never arrives, cellular transmission remains blocked; CAN collection continues. No blind retry, unverified reset or hidden automatic hardware reset is performed. Production use needs a tested modem-reset/reinitialization recovery path for this true command-channel failure. A firmware restart also restarts CAN collection and is not prescribed as a lossless recovery.

The existing TCP send/receive implementation can itself block the CAN loop. This candidate does not make the entire tracker lossless or real-time. UART write buffering/CTS behavior, worst-case CAN polling gaps, network outages, power management, clock drift and reference-clock offset all require target validation. NTP is unauthenticated here and is unsuitable as an independent security/trusted-time authority.

## Host tests

From the repository root run:

    sh tests/ntp/run_tests.sh

Tests require an installed C++ compiler with UBSan; they install nothing and use no network. The two portable helper tests use C++11. Transport tests use C++17 because the captured upstream fixture uses nested namespace syntax. A fifth test uses the resolved ArduinoJson headers (or ARDUINOJSON_INCLUDE) to verify real JSON storage/serialization; it reports SKIPPED when those headers are unavailable. The fixtures are test-only, not a replacement production dependency.

Fixtures copied unmodified from SeeedJP/wio_cellular main on 2026-11-04:
- AtClient.hpp blob 1534f3fd962002bf256b5d27a176af944fbfd6e5.
- CountdownTimer.hpp blob 0675689d3bea3911dab3f35834437bff4667a785.
- The resolved WioCellular dependency is pinned to 0.3.16; both fixture files were compared byte-for-byte with that installed package.

The transport tests compile the actual upstream line reader/URC dispatcher with a fake UART. The sketch-service fixture contains the exact service code from the candidate plus host mocks. The host tests do not compile the full Arduino sketch or emulate hardware. Re-extract the fixture when changing that service code; do not infer coverage for untested code.

## Build selection and remaining checks

The PlatformIO source filter selects only `src/grove-can-bus.cpp` and the thin `platformio/cellular-main.cpp` entry point, excludes `tests/`, and adds the existing driver header directory to include paths. The entry point stays outside the Arduino library src directory, so it cannot inject duplicate setup/loop symbols into Arduino IDE consumers. It includes the Arduino IDE sketch as its single source of truth and provides explicit function declarations: PlatformIO does not preprocess nested `.ino` examples with this repository's `src_dir=.` configuration. This avoids linking host test mains or the other example into firmware. The alternate command example is no longer selected by this environment; build it separately with an appropriate source filter.

The existing sketch already required ArduinoJson but did not declare it. The build now declares ArduinoJson 7.4.3 and WioCellular 0.3.16, and pins the Seeed platform/framework to the verified source revisions:
- platform-nordicnrf52: 8f55ff26c0822ffcbff978be1fddfb8c88c57577.
- Adafruit_nRF52_Arduino: 514f28ee405952ecc58558b5e0608a2b63d9901a.

Use PlatformIO Core 6.2.0 and run `pio run -e seeed_wio_bg770a`. The build is separate from flashing; do not use the upload target without hardware authorization. The CAN message array is initialized with `.to<JsonArray>()`; the former assignment of a default `JsonArray()` produced a null array and is covered by the real-ArduinoJson regression test.

Hardware acceptance: confirm command support on the modem firmware, NTP success/error/timeout handling, DNS and UDP/123 reachability through the actual SIM/APN, timestamp comparisons, no overlap with TCP commands, sustained CAN capture during each failure path, and tested AT ownership recovery. No hardware flashing or LTE requests were performed during preparation.

Primary specifications:
- https://quectel.com/content/uploads/2024/02/Quectel_BG770A-GLBG95xA-GL_TCPIP_Application_Note_V1.1.pdf (2.3.14)
- https://quectel.com/content/uploads/2024/05/Quectel_BG77xA-GLBG95xA-GL_AT_Commands_Manual_V1.1.pdf (9.2)
- https://github.com/SeeedJP/wio_cellular
- https://developers.cloudflare.com/time-services/ntp/usage/

## Verification record (2026-11-04)

PlatformIO Core 6.2.0 built and linked the Wio BG770A cellular firmware using the pinned dependencies above and ARM GCC 12.3.1. The firmware image has not been flashed. The initial successful build reported 10,304 bytes of static RAM and 107,020 bytes of flash; these are linker figures, not peak runtime heap/stack measurements. The host run passed five programs, including the actual resolved ArduinoJson integration, with warnings-as-errors and UBSan. Test fixture source files match WioCellular 0.3.16 byte-for-byte. The framework's upstream startup code emits ignored-attribute warnings; this is not a warning-free dependency build.

The separate GNSS feature must arbitrate the same modem: do not switch radio mode or issue GNSS commands while this NTP session owns the AT channel, and do not start NTP while GNSS owns it. This PR does not add GNSS or authorize hardware flashing.
