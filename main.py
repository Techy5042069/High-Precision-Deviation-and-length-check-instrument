#!/usr/bin/env python3

import json
import os
import socket
import sys

from version import VERSION
from discovery import auto_detect_serial, validate_ip
from master import GantryMaster


LAST_CONN_FILE = "last_connection.json"
SENSOR_PORT = 5001


# ─────────────────────────────────────────────────────────────
# SAVE / LOAD LAST SENSOR IP
# ─────────────────────────────────────────────────────────────

def load_last_ip():
    if not os.path.exists(LAST_CONN_FILE):
        return None

    try:
        with open(LAST_CONN_FILE, "r") as f:
            data = json.load(f)

        return data.get("sensor_ip")

    except Exception:
        return None


def save_last_ip(ip):
    try:
        with open(LAST_CONN_FILE, "w") as f:
            json.dump({"sensor_ip": ip}, f, indent=2)

    except Exception as e:
        print(f"  Could not save last IP: {e}")


# ─────────────────────────────────────────────────────────────
# QUICK SENSOR CONNECTION TEST
# ─────────────────────────────────────────────────────────────

def test_sensor(ip, port=SENSOR_PORT, timeout=1.0):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.close()
        return True

    except Exception:
        return False


# ─────────────────────────────────────────────────────────────
# DISCOVERY
# ─────────────────────────────────────────────────────────────

def discover_connections():

    print(f"\n  Gantry Master v{VERSION}")
    print("  ── Discovery ──────────────────────────────────\n")

    # ── MOTOR SERIAL ─────────────────────────────────────────

    candidates = auto_detect_serial()
    motor_port = None

    if candidates:

        print("  Serial ports found:")

        for i, p in enumerate(candidates):
            print(f"    {i+1}. {p}")

        if len(candidates) == 1:

            ans = input(
                f"  Use {candidates[0]}? [Y/n]: "
            ).strip().lower()

            if ans != "n":
                motor_port = candidates[0]

        else:

            idx = input(
                "  Select port or Enter manual: "
            ).strip()

            if idx.isdigit() and 1 <= int(idx) <= len(candidates):
                motor_port = candidates[int(idx) - 1]

    if not motor_port:
        motor_port = input(
            "  Motor port (COM3 / /dev/ttyACM0): "
        ).strip()

    # ── SENSOR IP ───────────────────────────────────────────

    sensor_ip = None

    last_ip = load_last_ip()

    if last_ip:

        ans = input(
            f"\n  Try last sensor IP {last_ip}? [Y/n]: "
        ).strip().lower()

        if ans != "n":

            print(f"  Testing {last_ip}...")

            if test_sensor(last_ip):
                print("  Sensor reachable ✔")
                sensor_ip = last_ip
            else:
                print("  No response.")

    while not sensor_ip:

        ip = input(
            "\n  Sensor IP (manual): "
        ).strip()

        if not validate_ip(ip):
            print("  Invalid IP format.")
            continue

        print(f"  Testing {ip}...")

        if test_sensor(ip):
            print("  Sensor reachable ✔")
            sensor_ip = ip
        else:
            ans = input(
                "  Connection failed. Use anyway? [y/N]: "
            ).strip().lower()

            if ans == "y":
                sensor_ip = ip

    return motor_port, sensor_ip


# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":

    if len(sys.argv) == 3:

        motor_port = sys.argv[1]
        sensor_ip  = sys.argv[2]

    else:

        motor_port, sensor_ip = discover_connections()

    # save successful IP
    save_last_ip(sensor_ip)

    app = GantryMaster(motor_port, sensor_ip)
    app.connect()
    app.run()