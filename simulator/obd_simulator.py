import json
import time

SIGNALS = {
    "rpm": 2350.0,
    "speed_kmh": 72.0,
    "coolant_c": 88.0,
    "engine_load_pct": 42.0,
    "intake_air_c": 32.0,
    "throttle_pct": 18.0,
    "distance_km": 12345.0,
    "voltage_v": 13.8,
    "ambient_c": 27.0,
}


def build_pid_frame(pid: int, a: int, b: int = 0, c: int = 0) -> list[int]:
    # 0x41 indicates a response to mode 0x01
    return [0x04, 0x41, pid, a & 0xFF, b & 0xFF, c & 0xFF, 0x00, 0x00]


def encode_rpm(rpm: float) -> tuple[int, int]:
    value = int(rpm * 4)
    return (value >> 8) & 0xFF, value & 0xFF


def encode_speed(speed_kmh: float) -> int:
    return int(speed_kmh) & 0xFF


def encode_temp(temp_c: float) -> int:
    return int(temp_c + 40) & 0xFF


def encode_pct(pct: float) -> int:
    return int((pct / 100.0) * 255) & 0xFF


def encode_distance_km(distance_km: float) -> tuple[int, int]:
    value = int(distance_km)
    return (value >> 8) & 0xFF, value & 0xFF


def encode_voltage(voltage_v: float) -> tuple[int, int]:
    value = int(voltage_v * 1000)
    return (value >> 8) & 0xFF, value & 0xFF


def generate_frames(signals: dict) -> list[dict]:
    rpm_a, rpm_b = encode_rpm(signals["rpm"])
    dist_a, dist_b = encode_distance_km(signals["distance_km"])
    volt_a, volt_b = encode_voltage(signals["voltage_v"])

    frames = [
        {"pid": "0x0C", "name": "engine_rpm", "frame": build_pid_frame(0x0C, rpm_a, rpm_b)},
        {"pid": "0x0D", "name": "vehicle_speed", "frame": build_pid_frame(0x0D, encode_speed(signals["speed_kmh"]))},
        {"pid": "0x05", "name": "coolant_temp", "frame": build_pid_frame(0x05, encode_temp(signals["coolant_c"]))},
        {"pid": "0x04", "name": "engine_load", "frame": build_pid_frame(0x04, encode_pct(signals["engine_load_pct"]))},
        {"pid": "0x0F", "name": "intake_air_temp", "frame": build_pid_frame(0x0F, encode_temp(signals["intake_air_c"]))},
        {"pid": "0x11", "name": "throttle_position", "frame": build_pid_frame(0x11, encode_pct(signals["throttle_pct"]))},
        {"pid": "0x31", "name": "distance_traveled", "frame": build_pid_frame(0x31, dist_a, dist_b)},
        {"pid": "0x42", "name": "control_module_voltage", "frame": build_pid_frame(0x42, volt_a, volt_b)},
        {"pid": "0x46", "name": "ambient_air_temp", "frame": build_pid_frame(0x46, encode_temp(signals["ambient_c"]))},
    ]
    return frames


def main() -> None:
    while True:
        payload = {
            "timestamp": time.time(),
            "frames": generate_frames(SIGNALS),
            "signals": SIGNALS,
        }
        print(json.dumps(payload))
        time.sleep(1)


if __name__ == "__main__":
    main()
