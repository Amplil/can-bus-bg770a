# PC Simulation Guide

This folder contains a lightweight PC simulator that generates OBD-II PID response frames
so you can develop and validate parsing, retry, and packaging logic without hardware.

## Prerequisites (Windows)

- Install **Python 3.11+** from https://www.python.org/downloads/
- Ensure `python` is available in your PATH.

## Run the simulator

From the repo root:

```bash
python simulator/obd_simulator.py
```

This prints simulated PID responses in JSON with raw bytes and decoded values. You can
redirect the output to a log file or pipe it into your own tooling.

## Customize signals

Edit the `SIGNALS` dictionary in `obd_simulator.py` to change default speeds, RPMs, or temperatures.
The generator will adjust the raw bytes accordingly.
