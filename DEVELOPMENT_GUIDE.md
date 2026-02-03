# Development Guide (Windows-first, Hardware-optional)

This guide helps you get into a regular development loop for the BG770A CAN/OBD-II project,
even if you don't have the embedded board or a vehicle on your desk.

## 1) Install prerequisites

### PlatformIO (Firmware builds)
1. Install **Visual Studio Code**: https://code.visualstudio.com/
2. Install the **PlatformIO IDE** extension.
3. Open this repo folder in VS Code.

### Python (PC simulator)
1. Install **Python 3.11+**: https://www.python.org/downloads/
2. Confirm it is on your PATH:

```bash
python --version
```

## 2) Understand the repo layout

- `src/` contains library code for the BG770A CAN module.
- `main.cpp` provides a minimal Arduino sketch for PlatformIO builds.
- `simulator/` contains a PC-only OBD-II simulator for hardware-free testing.
- `src/obd-logic.*` contains pure parsing helpers that can run on PC or device.

## 3) Firmware dev loop (when you have hardware)

1. **Build** firmware:

```bash
pio run
```

2. **Upload** to the Wio BG770A board (when connected):

```bash
pio run --target upload
```

3. **Monitor logs** (optional):

```bash
pio device monitor
```

If you see missing `Serial`/USB CDC linker errors, ensure `USE_TINYUSB` is enabled in
`platformio.ini` (already set in this repo).

## 4) Hardware-free dev loop (PC simulator)

Use this to develop parsing, validation, and packaging logic without a board or vehicle.

1. **Run the simulator**:

```bash
python simulator/obd_simulator.py
```

2. The simulator prints JSON to stdout:
   - `frames`: raw OBD-II PID response bytes.
   - `signals`: the decoded values used to generate the frames.

3. **Customize simulated vehicle values** by editing `SIGNALS` in
   `simulator/obd_simulator.py` (RPM, speed, temps, etc.).

## 5) Suggested local development workflow

1. **Develop core logic first** using the simulator:
   - Create or update parsing/validation code in `src/obd-logic.*`.
   - Use simulator output as test inputs in your PC-side tooling.

2. **Integrate hardware code** later:
   - Replace simulator input with real CAN/OBD data from the board.
   - Keep the parsing/validation logic shared between PC and device.

3. **Iterate**:
   - Simulator: fast feedback without hardware.
   - Hardware: final validation in real vehicle conditions.

## 6) Troubleshooting tips

- If `pio run` fails with missing `setup()`/`loop()`, verify `main.cpp` exists at repo root.
- If USB serial is missing, verify `USE_TINYUSB` is present in `platformio.ini`.
- If Python can't be found, reinstall Python and check the “Add to PATH” option.
