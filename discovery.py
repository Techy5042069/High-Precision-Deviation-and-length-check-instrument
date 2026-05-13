# ─────────────────────────────────────────────────────────────────────────────
# discovery.py
#
# Simple hardware discovery:
#   ✔ Auto-detect serial (motor Arduino)
#   ✖ No LAN scanning
#   ✔ Sensor IP is manually provided by user
# ─────────────────────────────────────────────────────────────────────────────

import socket
import serial.tools.list_ports


# ─────────────────────────────────────────────────────────────
# AUTO DETECT MOTOR ARDUINO (SERIAL)
# ─────────────────────────────────────────────────────────────
def auto_detect_serial():
    """
    Returns a list of likely Arduino serial ports.

    Example:
        ['COM3'] or ['/dev/ttyACM0']
    """
    ports = serial.tools.list_ports.comports()
    candidates = []

    keywords = (
        "arduino",
        "ch340",
        "cp210",
        "ftdi",
        "usb",
        "acm",
        "r4",
    )

    for p in ports:
        desc = (p.description or "").lower()
        mfg  = (p.manufacturer or "").lower()

        if any(k in desc or k in mfg for k in keywords):
            candidates.append(p.device)

    return candidates


# ─────────────────────────────────────────────────────────────
# MANUAL SENSOR IP INPUT
# ─────────────────────────────────────────────────────────────
def get_sensor_ip():
    """
    Ask user to manually enter the sensor Arduino IP.
    """
    print("\n[Discovery] Enter sensor Arduino IP manually.")
    print("Example: 192.168.137.45\n")

    while True:
        ip = input("Sensor IP: ").strip()

        if validate_ip(ip):
            return ip

        print("  Invalid IP format. Try again (e.g. 192.168.137.45)")


# ─────────────────────────────────────────────────────────────
# OPTIONAL CONNECTION TEST (NOT REQUIRED)
# ─────────────────────────────────────────────────────────────
def test_sensor_connection(ip: str, port: int = 5001, timeout: float = 1.0):
    """
    Optional: verify sensor is reachable.
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))

        data = s.recv(256)
        s.close()

        print(f"\n[Discovery] Connected to {ip}:{port}")

        if b"IN:" in data or b"sensor" in data.lower():
            print("[Discovery] Sensor detected ✔")
        else:
            print("[Discovery] Connected (no banner yet)")

        return True

    except Exception as e:
        print(f"\n[Discovery] Connection failed: {e}")
        return False


# ─────────────────────────────────────────────────────────────
# IP VALIDATION
# ─────────────────────────────────────────────────────────────
def validate_ip(ip: str) -> bool:
    parts = ip.split(".")

    if len(parts) != 4:
        return False

    try:
        return all(0 <= int(p) <= 255 for p in parts)
    except ValueError:
        return False