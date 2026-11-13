# Cellular startup stall

The supplied startup log ended at `Cellular bus initialized`. That message was
printed **before** `WioCellular.begin()`, modem power-on, network configuration,
and the three-minute registration wait. CAN initialization followed all four
steps. The log therefore did not identify which call had stopped progressing.

The installed Arduino library is WioCellular 0.3.12. Its
`src/network/Bg770aNetwork.hpp` has an unbounded loop in `begin()`: when APN,
technology, or band settings need changing, it sends `CFUN=0` and waits for its
cached registration state to become `NotSearching`. A missing `+CEREG: 0`
notification leaves that cache unchanged even when the modem has already
stopped searching. The sketch's later `NETWORK_TIMEOUT` does not cover this
loop. The startup regression reproduces this case with the actual installed
network header and synthetic modem responses.

`src/cellular-startup.h` switches the radio off and explicitly queries
registration until state 0, with a 20-second polling deadline, before calling
`WioNetwork.begin()`. The library then initializes its cache from the queried
offline state and can apply settings without waiting for a notification. The
helper explicitly restores `CFUN=1`, including when all settings already match.
It is used for both initial startup and wake from modem power-off. The installed
WioCellular library itself is unchanged.

The sketch now initializes CAN after `WioCellular.begin()` (which switches Grove
power off) and before powering on or connecting LTE. Logs identify
`CELLULAR-BEGIN`, `CELLULAR-POWER`, and `CELLULAR-INIT` stages (`radio-off`,
`registration`, `network-config`, `radio-on`). `LTE-WAIT` reports state and elapsed
time every five seconds. An LTE registration timeout continues into the CAN
loop. A power-on or preparation failure disables modem operations and retries
initialization after 60 seconds, only when no OBD response is pending. Without a
valid clock, CAN diagnostics continue and timestamped measurement records remain
deferred.

The polling deadline does not replace individual library AT timeouts.
`powerOn(20000)` can retry the APP RDY wait and then execute flow-control AT
commands, so it can take longer than 20 seconds. Low-level UART flow-control
waits remain in the installed library. Errors inside `WioNetwork.begin()` or
`canCommunicate()` print `CELLULAR-ERROR` with the library location before
entering the existing abort/reset handler. The new stage logs distinguish these
paths; the supplied short log alone does not confirm the physical board's exact
stop location.

## Validation

- Arduino CLI build passed for `SeeedJP:nrf52:wio_bg770a`, board 1.0, using
  SeeedJP nRF52 Boards 1.5.0 and the installed WioCellular 0.3.12.
- `tests/startup/run_tests.sh` compiles the actual installed network header and
  checks the original missing-notification stall, successful/delayed preparation,
  unchanged settings, explicit radio restoration, polling timeout, uptime wrap,
  and AT failures. Set `WIOCELLULAR_INCLUDE` to a WioCellular `src` directory.
- `tests/integration/test_sketch.py` executes the actual LTE wait function and
  main loop against host transports. It checks five-second progress, CAN receive
  service during the wait, timeout without abort, later connectivity, modem
  suppression after failure, and a 60-second retry only after pending OBD ends.
- Existing NTP, GNSS, OBD, driver, telemetry, UTC, and acknowledgment checks passed.

No device was flashed and no post-fix hardware trace was collected. The build
and host checks establish code behavior; the next board run will identify any
remaining hardware or modem-specific failure through its last reported stage.
