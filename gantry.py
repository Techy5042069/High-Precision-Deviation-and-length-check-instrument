#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# gantry.py  —  Two-Arduino Gantry Controller
#
# Motor Arduino  : wired USB-Serial  → ST:<seq>,<steps>
# Sensor Arduino : WiFi UDP stream   → continuous 2-byte ADC packets
#
# On each ST: the latest ADC reading from the UDP stream is snapshotted.
# OOR detection runs entirely in Python.
#
# Usage:
#   python gantry.py                        # interactive discovery
#   python gantry.py COM3 192.168.1.45      # direct
# ─────────────────────────────────────────────────────────────────────────────

import csv
import datetime
import json
import os
import queue
import socket
import struct
import sys
import threading
import time

import serial
import serial.tools.list_ports
from pynput import keyboard

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

VERSION   = "2.0"
BAUD_RATE = 921600

SENSOR_CENTER_MM = 30.0
SENSOR_RANGE_MM  = 5.0
ADC_BITS         = 14
ADC_MAX          = (1 << ADC_BITS) - 1   # 16383
ADC_REF_V        = 5.0

MAX_RPM        = 180.0
MIN_RPM        = 0.1
STEP_DELAY_MIN = 104
STEP_DELAY_MAX = 2500

DEFAULT_PULLEY_TEETH      = 80
DEFAULT_MICROSTEP         = 8
DEFAULT_FULL_STEPS        = 200
DEFAULT_PITCH_MM          = 2.0
DEFAULT_MANUAL_RPM        = 40.0
DEFAULT_SCAN_RPM          = 30.0
DEFAULT_SCAN_SAMPLE_STEPS = 10
DEFAULT_OOR_ARM_COUNT     = 5

MOTOR_SERIAL_PORT = None   # set at runtime
SENSOR_IP         = None   # set at runtime
SENSOR_LISTEN_PORT = 5001  # Arduino listens for commands here
SENSOR_STREAM_PORT = 5002  # Python listens for ADC data here

CONFIG_FILE    = "gantry_config.json"
LAST_CONN_FILE = "last_connection.json"

# ─────────────────────────────────────────────────────────────────────────────
# Physics helpers
# ─────────────────────────────────────────────────────────────────────────────

def adc_to_voltage(adc):
    return (adc / ADC_MAX) * ADC_REF_V

def voltage_to_dist(v):
    return (SENSOR_CENTER_MM - SENSOR_RANGE_MM) + (v / ADC_REF_V) * (2.0 * SENSOR_RANGE_MM)

def adc_to_dist(adc):
    v = adc_to_voltage(adc)
    if adc >= ADC_MAX or adc <= 0:
        return v, None, True
    return v, voltage_to_dist(v), False

def adc_to_delta_dist(adc):
    return ((adc / ADC_MAX) * SENSOR_RANGE_MM * 2) - SENSOR_RANGE_MM

def delta_dist_to_adc(delta_mm):
    return int((delta_mm + SENSOR_RANGE_MM) * (ADC_MAX / (SENSOR_RANGE_MM * 2)))

def calc_spr(microstep, full_steps=DEFAULT_FULL_STEPS):
    return full_steps * microstep

def calc_spmm(microstep, pulley_teeth, full_steps=DEFAULT_FULL_STEPS, pitch=DEFAULT_PITCH_MM):
    return calc_spr(microstep, full_steps) / (pulley_teeth * pitch)

def rpm_to_delay(rpm, spr):
    if rpm <= 0: return STEP_DELAY_MAX
    delay = int(1_000_000 / (2 * (rpm / 60.0) * spr))
    return max(STEP_DELAY_MIN, min(STEP_DELAY_MAX, delay))

def delay_to_rpm(delay_us, spr):
    return (1_000_000 / (2 * delay_us) / spr) * 60.0

def hw_max_rpm(microstep):
    spr = calc_spr(microstep)
    return min((1_000_000 / (2 * STEP_DELAY_MIN) / spr) * 60.0, MAX_RPM)

def resolution_mm(scan_sample_steps, microstep, pulley_teeth,
                  full_steps=DEFAULT_FULL_STEPS, pitch=DEFAULT_PITCH_MM):
    return scan_sample_steps / calc_spmm(microstep, pulley_teeth, full_steps, pitch)

# ─────────────────────────────────────────────────────────────────────────────
# Terminal helpers
# ─────────────────────────────────────────────────────────────────────────────

def clear():
    os.system("cls" if os.name == "nt" else "clear")

def prompt_float(msg, default, minv=None, maxv=None):
    while True:
        raw = input(f"  {msg} [{default}]: ").strip()
        if not raw: return default
        try:
            v = float(raw)
            if minv is not None and v < minv: print(f"  Min: {minv}"); continue
            if maxv is not None and v > maxv: print(f"  Max: {maxv}"); continue
            return v
        except ValueError:
            print("  Enter a number.")

def prompt_int(msg, default, minv=1, maxv=None):
    while True:
        raw = input(f"  {msg} [{default}]: ").strip()
        if not raw: return default
        try:
            v = int(raw)
            if v < minv: print(f"  Min: {minv}"); continue
            if maxv is not None and v > maxv: print(f"  Max: {maxv}"); continue
            return v
        except ValueError:
            print("  Enter an integer.")

# ─────────────────────────────────────────────────────────────────────────────
# SerialNode  —  motor Arduino (wired)
# ─────────────────────────────────────────────────────────────────────────────

class SerialNode:
    def __init__(self, port, baud=BAUD_RATE):
        self.port = port; self.baud = baud
        self.ser = None; self._lock = threading.Lock()
        self.lines = queue.Queue(); self.connected = False

    def connect(self):
        print(f"  Connecting motor Arduino on {self.port}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.connected = True
        print("  ✓ Motor connected")
        threading.Thread(target=self._recv, daemon=True).start()

    def send(self, msg):
        with self._lock:
            try: self.ser.write((msg + "\n").encode())
            except Exception as e: print(f"  [Motor] send error: {e}")

    def _recv(self):
        try:
            while True:
                line = self.ser.readline().decode(errors="replace").strip()
                if line: self.lines.put(line)
        except Exception: pass
        self.connected = False
        self.lines.put("__DISCONNECTED__")

    def poll(self, timeout=0.0):
        try: return self.lines.get(timeout=timeout)
        except queue.Empty: return None

# ─────────────────────────────────────────────────────────────────────────────
# UDPNode  —  sensor Arduino (WiFi)
#
# Receives a continuous stream of 2-byte little-endian uint16 ADC packets.
# latest_adc is always the most recent value — Python snapshots it on each ST:.
# ─────────────────────────────────────────────────────────────────────────────

class UDPNode:
    def __init__(self, sensor_ip,
                 listen_port=SENSOR_LISTEN_PORT,
                 stream_port=SENSOR_STREAM_PORT):
        self.sensor_ip   = sensor_ip
        self.listen_port = listen_port   # we SEND commands here
        self.stream_port = stream_port   # we RECEIVE adc data here
        self.latest_adc  = None
        self.connected   = False
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def connect(self):
        print(f"  Connecting sensor Arduino at {self.sensor_ip}...")
        self._rx.bind(("", self.stream_port))
        self._rx.settimeout(0.1)
        self.connected = True
        threading.Thread(target=self._recv, daemon=True).start()
        # Tell sensor to start streaming to us
        time.sleep(0.2)
        self.send("HELLO")
        print(f"  ✓ Sensor streaming (UDP {self.sensor_ip}:{self.listen_port}"
              f" → :{self.stream_port})")

    def send(self, msg):
        try:
            self._tx.sendto((msg + "\n").encode(),
                            (self.sensor_ip, self.listen_port))
        except Exception as e:
            print(f"  [Sensor] send error: {e}")

    def _recv(self):
        while self.connected:
            try:
                data, _ = self._rx.recvfrom(2)
                if len(data) == 2:
                    self.latest_adc = struct.unpack('<H', data)[0]
            except socket.timeout:
                continue
            except Exception:
                break
        self.connected = False

    def stop(self):
        self.send("STOP")
        self.connected = False

# ─────────────────────────────────────────────────────────────────────────────
# Discovery
# ─────────────────────────────────────────────────────────────────────────────

def auto_detect_serial():
    kw = ("arduino", "ch340", "cp210", "ftdi", "usb", "acm", "r4")
    return [p.device for p in serial.tools.list_ports.comports()
            if any(k in (p.description or "").lower() or
                   k in (p.manufacturer or "").lower() for k in kw)]

def validate_ip(ip):
    parts = ip.split(".")
    if len(parts) != 4: return False
    try: return all(0 <= int(p) <= 255 for p in parts)
    except ValueError: return False

def load_last_conn():
    try:
        with open(LAST_CONN_FILE) as f: return json.load(f)
    except Exception: return {}

def save_last_conn(data):
    try:
        with open(LAST_CONN_FILE, "w") as f: json.dump(data, f, indent=2)
    except Exception: pass

def discover():
    print(f"\n  Gantry v{VERSION}  —  Motor (serial) + Sensor (UDP WiFi)")
    print("  ── Discovery ───────────────────────────────────\n")
    last = load_last_conn()

    # ── Motor serial ──────────────────────────────────────────────────────────
    candidates = auto_detect_serial()
    motor_port = None
    last_port  = last.get("motor_port")

    if last_port:
        ans = input(f"  Use last motor port {last_port}? [Y/n]: ").strip().lower()
        if ans != "n": motor_port = last_port

    if not motor_port:
        if candidates:
            print("  Serial ports found:")
            for i, p in enumerate(candidates):
                print(f"    {i+1}. {p}")
            idx = input(f"  Select [1-{len(candidates)}] or Enter to type: ").strip()
            if idx.isdigit() and 1 <= int(idx) <= len(candidates):
                motor_port = candidates[int(idx) - 1]
        if not motor_port:
            motor_port = input("  Motor port (COM3 / /dev/ttyACM0): ").strip()

    # ── Sensor IP ─────────────────────────────────────────────────────────────
    sensor_ip  = None
    last_ip    = last.get("sensor_ip")

    if last_ip:
        ans = input(f"\n  Use last sensor IP {last_ip}? [Y/n]: ").strip().lower()
        if ans != "n": sensor_ip = last_ip

    while not sensor_ip:
        ip = input("\n  Sensor IP (e.g. 192.168.1.45): ").strip()
        if validate_ip(ip):
            sensor_ip = ip
        else:
            print("  Invalid IP.")

    save_last_conn({"motor_port": motor_port, "sensor_ip": sensor_ip})
    return motor_port, sensor_ip

# ─────────────────────────────────────────────────────────────────────────────
# Gantry  —  top-level controller
# ─────────────────────────────────────────────────────────────────────────────

class Gantry:
    def __init__(self, motor_port, sensor_ip):
        self.motor  = SerialNode(motor_port)
        self.sensor = UDPNode(sensor_ip)

        # Config
        self.pulley_teeth      = DEFAULT_PULLEY_TEETH
        self.microstep         = DEFAULT_MICROSTEP
        self.full_steps        = DEFAULT_FULL_STEPS
        self.pitch_mm          = DEFAULT_PITCH_MM
        self.manual_rpm        = DEFAULT_MANUAL_RPM
        self.scan_rpm          = DEFAULT_SCAN_RPM
        self.scan_sample_steps = DEFAULT_SCAN_SAMPLE_STEPS

        # OOR — handled entirely in Python
        self.use_oor       = False
        self.oor_lo        = 0
        self.oor_hi        = ADC_MAX
        self.oor_arm_count = DEFAULT_OOR_ARM_COUNT
        self._oor_consec   = 0
        self._oor_armed    = False
        self._oor_fired    = False

        # Live display
        self.last_volt = None
        self.last_dist = None
        self.limit_a   = False
        self.limit_b   = False

        # Scan state
        self.scan_data    = []
        self.scan_done    = False
        self.scan_aborted = False
        self.scan_active  = False
        self.scan_total   = 0

        # Manual mode
        self.manual_active = False
        self.active_key    = None

        self._load_config()

    # ── Derived ───────────────────────────────────────────────────────────────

    @property
    def _spr(self):
        return calc_spr(self.microstep, self.full_steps)

    @property
    def _spmm(self):
        return calc_spmm(self.microstep, self.pulley_teeth, self.full_steps, self.pitch_mm)

    @property
    def _res_mm(self):
        return resolution_mm(self.scan_sample_steps, self.microstep,
                             self.pulley_teeth, self.full_steps, self.pitch_mm)

    @property
    def _hw_max(self):
        return hw_max_rpm(self.microstep)

    # ── Config ────────────────────────────────────────────────────────────────

    def _config_dict(self):
        return {
            "pulley_teeth":      self.pulley_teeth,
            "microstep":         self.microstep,
            "manual_rpm":        self.manual_rpm,
            "scan_rpm":          self.scan_rpm,
            "scan_sample_steps": self.scan_sample_steps,
            "use_oor":           self.use_oor,
            "oor_lo_mm":         round(adc_to_delta_dist(self.oor_lo), 4),
            "oor_hi_mm":         round(adc_to_delta_dist(self.oor_hi), 4),
            "oor_arm_count":     self.oor_arm_count,
        }

    def _load_config(self):
        if not os.path.exists(CONFIG_FILE):
            self._save_config(); return
        try:
            with open(CONFIG_FILE) as f: d = json.load(f)
            self.pulley_teeth      = d.get("pulley_teeth",      self.pulley_teeth)
            self.microstep         = d.get("microstep",         self.microstep)
            self.manual_rpm        = d.get("manual_rpm",        self.manual_rpm)
            self.scan_rpm          = d.get("scan_rpm",          self.scan_rpm)
            self.scan_sample_steps = d.get("scan_sample_steps", self.scan_sample_steps)
            self.use_oor           = d.get("use_oor",           self.use_oor)
            if "oor_lo_mm" in d:
                self.oor_lo = delta_dist_to_adc(d["oor_lo_mm"])
                self.oor_hi = delta_dist_to_adc(d["oor_hi_mm"])
            self.oor_arm_count = d.get("oor_arm_count", self.oor_arm_count)
            print(f"  [Config] Loaded '{CONFIG_FILE}'.")
        except Exception as e:
            print(f"  [Config] Load error: {e} — using defaults.")

    def _save_config(self):
        try:
            with open(CONFIG_FILE, "w") as f: json.dump(self._config_dict(), f, indent=2)
            print(f"  [Config] Saved '{CONFIG_FILE}'.")
        except Exception as e:
            print(f"  [Config] Save error: {e}")

    # ── Connect ───────────────────────────────────────────────────────────────

    def connect(self):
        self.motor.connect()
        self.sensor.connect()
        time.sleep(0.5)
        # Drain motor banner
        deadline = time.time() + 1.5
        while time.time() < deadline:
            line = self.motor.poll(timeout=0.05)
            if line: print(f"  [Motor] {line}")
        self._push_config()

    def _push_config(self):
        m_d = rpm_to_delay(self.manual_rpm, self._spr)
        s_d = rpm_to_delay(self.scan_rpm,   self._spr)
        self.motor.send(f"V{m_d}"); time.sleep(0.02)
        self.motor.send(f"W{s_d}"); time.sleep(0.02)
        self.motor.send(f"B{self.scan_sample_steps}")
        # Push averaging setting to sensor
        self.sensor.send(f"AVG:{max(1, min(16, getattr(self, 'adc_avg', 4)))}")

    # ── Motor message parser ──────────────────────────────────────────────────

    def _parse_motor(self, line):

        # ── Step telemetry ────────────────────────────────────────────────────
        if line.startswith("ST:"):
            if not self.scan_active:
                return
            parts = line[3:].split(",")
            if len(parts) == 2:
                try:
                    seq   = int(parts[0])
                    steps = int(parts[1])

                    # Snapshot the latest UDP ADC reading — this is the
                    # "assume instant" pairing: whatever the sensor is reading
                    # right now corresponds to this motor position.
                    adc = self.sensor.latest_adc
                    if adc is None:
                        # Sensor not streaming yet — store as missing
                        self.scan_data.append((seq, steps, None, None))
                        return

                    v, dist, alarm = adc_to_dist(adc)
                    self.last_volt = v
                    self.last_dist = None if alarm else dist
                    self.scan_data.append((seq, steps, v, None if alarm else dist))

                    # ── Python-side OOR check ─────────────────────────────────
                    if self.use_oor and not self._oor_fired:
                        in_range = (self.oor_lo <= adc <= self.oor_hi)
                        if not self._oor_armed:
                            if in_range:
                                self._oor_consec += 1
                                if self._oor_consec >= self.oor_arm_count:
                                    self._oor_armed = True
                                    print(f"\n  ✔  [OOR ARMED] seq={seq}")
                            else:
                                self._oor_consec = 0
                        else:
                            if not in_range:
                                self._oor_fired = True
                                dev = adc_to_delta_dist(adc)
                                print(f"\n  ⚠  [OOR FIRED] seq={seq} "
                                      f"adc={adc} dev={dev:+.3f}mm")
                                self.motor.send("A")   # stop motor

                except ValueError:
                    pass
            return

        # ── SD: always processed — even after OOR abort ───────────────────────
        if line.startswith("SD:"):
            try: self.scan_total = int(line[3:])
            except ValueError: self.scan_total = 0
            self.scan_active = False
            self.scan_done   = True
            return

        if line == "SS":
            self.scan_data    = []
            self.scan_active  = True
            self.scan_done    = False
            self.scan_aborted = False
            self._oor_consec  = 0
            self._oor_armed   = False
            self._oor_fired   = False
            print("\n  [SCAN] Measuring...\n")
            return

        if line == "SA":
            self.scan_active  = False
            self.scan_aborted = True
            return

        if line == "LA": self.limit_a = True;  print("  [LIMIT A] Left end ◄")
        elif line == "LB": self.limit_b = True; print("  [LIMIT B] Right end ►")
        elif line == "CA": self.limit_a = False; print("  [INFO] Left limit cleared")
        elif line == "CB": self.limit_b = False; print("  [INFO] Right limit cleared")
        elif line.startswith("SP:"):
            try: print(f"  [SPEED] {delay_to_rpm(int(line[3:]), self._spr):.1f} RPM")
            except ValueError: pass
        elif line.startswith("SS_STEPS:"): print(f"  [Motor] ss={line[9:]}")
        elif line.startswith("WN:"): print(f"  ⚠  {line[3:]}")
        elif line.startswith("IN:"): print(f"  [Motor] {line[3:]}")

    # ── Poll loop  (motor only — UDP is handled by UDPNode._recv thread) ──────

    def _poll_loop(self):
        while True:
            line = self.motor.poll(timeout=0.002)
            if line:
                self._parse_motor(line)

    # ── Main menu ─────────────────────────────────────────────────────────────

    def run(self):
        threading.Thread(target=self._poll_loop, daemon=True).start()

        while True:
            clear()
            m_d = rpm_to_delay(self.manual_rpm, self._spr)
            s_d = rpm_to_delay(self.scan_rpm,   self._spr)

            oor_str = (
                f"ON  lo={adc_to_delta_dist(self.oor_lo):+.1f}mm "
                f"hi={adc_to_delta_dist(self.oor_hi):+.1f}mm "
                f"arm={self.oor_arm_count}"
                if self.use_oor else "OFF"
            )

            print("╔══════════════════════════════════════════════════════╗")
            print(f"║   Gantry v{VERSION}  —  Motor (serial) + Sensor (UDP)      ║")
            print("╠══════════════════════════════════════════════════════╣")
            print(f"║  Motor  : {self.motor.port:<20}  Serial {BAUD_RATE}   ║")
            print(f"║  Sensor : {self.sensor.sensor_ip:<20}  UDP stream          ║")
            print("╠══════════════════════════════════════════════════════╣")
            print(f"║  Pulley  : {self.pulley_teeth:<5}T  Microstep : {self.microstep:<4}  SPR : {self._spr:<6}    ║")
            print(f"║  Steps/mm : {self._spmm:<8.2f}  Resolution : {self._res_mm:.3f} mm/pt     ║")
            print(f"║  Manual RPM : {self.manual_rpm:<6.1f}  ({delay_to_rpm(m_d, self._spr):.1f} actual)            ║")
            print(f"║  Scan   RPM : {self.scan_rpm:<6.1f}  ({delay_to_rpm(s_d, self._spr):.1f} actual)            ║")
            print(f"║  Sample steps : {self.scan_sample_steps:<5}                               ║")
            print(f"║  OOR : {oor_str:<46}║")
            print("╠══════════════════════════════════════════════════════╣")
            print("║  1.  Manual mode                                     ║")
            print("║  2.  Full Scan                                       ║")
            print("║  3.  Configure                                       ║")
            print("║  Q.  Quit                                            ║")
            print("╚══════════════════════════════════════════════════════╝")

            ch = input("\n  Select: ").strip().upper()
            if   ch == "1": self._run_manual()
            elif ch == "2": self._run_scan()
            elif ch == "3": self._run_config()
            elif ch == "Q": break

        self.motor.send("S")
        self.sensor.stop()
        print("  Bye.")

    # ── Manual jog ────────────────────────────────────────────────────────────

    def _run_manual(self):
        clear()
        print("┌──────────────────────────────────────────────────┐")
        print("│  Manual mode                                     │")
        print("├──────────────────────────────────────────────────┤")
        print("│  ← →    move left / right (hold)                │")
        print("│  ↑ ↓    speed up / down                         │")
        print("│  I      sensor reading (latest UDP value)        │")
        print("│  Esc    back to menu                             │")
        print("└──────────────────────────────────────────────────┘\n")

        self.manual_active = True
        self.active_key    = None

        def on_press(key):
            char = None
            try: char = key.char.upper() if hasattr(key, "char") and key.char else None
            except Exception: pass

            if key == keyboard.Key.right:
                if self.limit_b: print("  ⚠  Right limit"); return
                if self.active_key != "R":
                    self.active_key = "R"; self.motor.send("R"); print("→ Moving right")
            elif key == keyboard.Key.left:
                if self.limit_a: print("  ⚠  Left limit"); return
                if self.active_key != "L":
                    self.active_key = "L"; self.motor.send("L"); print("← Moving left")
            elif key == keyboard.Key.up:
                self.motor.send("+"); print("↑ Speed up")
            elif key == keyboard.Key.down:
                self.motor.send("-"); print("↓ Speed down")
            elif char == "I":
                adc = self.sensor.latest_adc
                if adc is not None:
                    v, dist, alarm = adc_to_dist(adc)
                    ds = f"{dist:.3f}mm" if dist is not None else "ALARM"
                    print(f"  Sensor: adc={adc}  {v:.4f}V  {ds}")
                else:
                    print("  Sensor: no reading yet")
            elif key == keyboard.Key.esc:
                self.active_key = None; self.manual_active = False
                self.motor.send("S"); return False

        def on_release(key):
            if key in (keyboard.Key.right, keyboard.Key.left):
                self.active_key = None
                time.sleep(0.05); self.motor.send("S"); print("■ Stopped")

        def hold_loop():
            while self.manual_active:
                if   self.active_key == "R": self.motor.send("R")
                elif self.active_key == "L": self.motor.send("L")
                time.sleep(1.0 / 30)

        threading.Thread(target=hold_loop, daemon=True).start()
        with keyboard.Listener(on_press=on_press, on_release=on_release) as lst:
            lst.join()
        self.manual_active = False

    # ── Full scan ─────────────────────────────────────────────────────────────

    def _run_scan(self):
        clear()
        print("┌──────────────────────────────────────────────────┐")
        print(f"│  Full Scan  —  Gantry v{VERSION:<27}  │")
        print("│  Ctrl+C to abort                                 │")
        print("└──────────────────────────────────────────────────┘\n")
        print(f"  Scan RPM      : {self.scan_rpm:.1f}")
        print(f"  Sample steps  : {self.scan_sample_steps}  ({self._res_mm:.2f} mm/pt)")

        if self.use_oor:
            print(f"  OOR           : lo={adc_to_delta_dist(self.oor_lo):+.2f}mm  "
                  f"hi={adc_to_delta_dist(self.oor_hi):+.2f}mm  arm={self.oor_arm_count}")
        else:
            print("  OOR           : disabled")

        if input("\n  Start scan? [y/N]: ").strip().lower() != "y":
            return

        self.scan_data    = []
        self.scan_done    = False
        self.scan_aborted = False
        self.scan_active  = False
        self.scan_total   = 0

        self._push_config()
        time.sleep(0.01)
        print("  [SCAN] Homing to left limit...")
        self.motor.send("X")

        try:
            while not self.scan_done and not self.scan_aborted:
                time.sleep(0.005)
        except KeyboardInterrupt:
            print("\n  [ABORT] Ctrl+C")
            self.motor.send("A")
            self.scan_aborted = True

        time.sleep(0.1)   # let any trailing ST: lines arrive

        print(f"\n  Samples collected : {len(self.scan_data)}")

        # scan_done takes priority — OOR may fire on the last reading just as
        # the right limit is hit; SD: arriving means it's a complete scan.
        if self.scan_aborted and not self.scan_done:
            self._trim_to_oor_window()
            ans = input("\n  Save partial / OOR-trimmed scan? [Y/n]: ").strip().lower()
            if ans != "n":
                self._show_plot(); self._save_csv()
            input("\n  Press Enter.")
            return

        self._trim_to_oor_window()
        self._show_plot()
        self._save_csv()
        input("\n  Press Enter to return to menu.")

    # ── OOR window trim ───────────────────────────────────────────────────────

    def _trim_to_oor_window(self):
        if not self.use_oor or not self.scan_data:
            return

        oor_min_mm = adc_to_delta_dist(self.oor_lo)
        oor_max_mm = adc_to_delta_dist(self.oor_hi)

        def in_range(dist):
            if dist is None: return False
            return oor_min_mm <= (dist - SENSOR_CENTER_MM) <= oor_max_mm

        arm_index = None
        consec    = 0
        for i, (_, _, _, dist) in enumerate(self.scan_data):
            if in_range(dist):
                consec += 1
                if consec >= self.oor_arm_count:
                    arm_index = i - self.oor_arm_count + 1; break
            else:
                consec = 0

        if arm_index is None:
            print("  [Trim] Surface never entered range — keeping all data."); return

        disarm_index = len(self.scan_data)
        for i in range(arm_index + 1, len(self.scan_data)):
            if not in_range(self.scan_data[i][3]):
                disarm_index = i; break

        window = self.scan_data[arm_index:disarm_index]
        origin = window[0][1]
        self.scan_data = [(s, st - origin, v, d) for s, st, v, d in window]
        print(f"  [Trim] {len(self.scan_data)} pts  "
              f"start={origin/self._spmm:.2f}mm  "
              f"end={self.scan_data[-1][1]/self._spmm:.2f}mm")

    # ── Plot ──────────────────────────────────────────────────────────────────

    def _show_plot(self):
        try: import matplotlib.pyplot as plt
        except ImportError:
            print("  matplotlib not installed — skipping plot."); return

        pos  = [d[1] / self._spmm for d in self.scan_data]
        devs = [(d[3] - SENSOR_CENTER_MM) if d[3] is not None else float("nan")
                for d in self.scan_data]

        fig, ax = plt.subplots(figsize=(13, 5))
        fig.suptitle(f"Gantry Scan v{VERSION} — {len(self.scan_data)} pts  "
                     f"res={self._res_mm:.2f}mm  {self.scan_rpm:.0f}RPM", fontsize=12)

        if self.use_oor:
            lo = adc_to_delta_dist(self.oor_lo)
            hi = adc_to_delta_dist(self.oor_hi)
            ax.axhspan(lo, hi, color="#2ECC71", alpha=0.18, zorder=0,
                       label=f"OOR window [{lo:+.2f}, {hi:+.2f}] mm")
            ax.axhline(lo, color="#27AE60", lw=0.8, ls="--")
            ax.axhline(hi, color="#27AE60", lw=0.8, ls="--")

        ax.axhline(0.0, color="#888780", lw=0.8, ls=":", label="centre")
        ax.plot(pos, devs, color="#185FA5", lw=0.9, label="deviation")
        ax.set_ylabel("Deviation from centre (mm)")
        ax.set_xlabel("Gantry position (mm)")
        ax.set_ylim(-SENSOR_RANGE_MM - 0.5, SENSOR_RANGE_MM + 0.5)
        ax.set_yticks([-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5])
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
        plt.tight_layout()
        plt.show(block=False)
        print("  [Plot shown]")

    # ── CSV export ────────────────────────────────────────────────────────────

    def _save_csv(self):
        ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default = f"scan_{ts}.csv"
        print(f"\n  Save CSV  (Enter = {default})")
        print("  Tip: subdirectories are created automatically.")
        print("       e.g.  results/run1/scan  →  results/run1/scan.csv")
        fname = input("  Filename: ").strip() or default

        fname = fname.replace("\\", "/").strip("\"'")
        if not os.path.splitext(fname)[1]:
            fname += ".csv"

        dirpath = os.path.dirname(fname)
        if dirpath:
            try:
                os.makedirs(dirpath, exist_ok=True)
            except OSError as e:
                print(f"  ⚠  Could not create '{dirpath}': {e}")
                fname = os.path.basename(fname)
                print(f"  Saving to current directory: {fname}")

        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["seq", "step_count", "gantry_position_mm",
                        "sensor_voltage_V", "sensor_distance_mm",
                        "offset_from_center_mm", "alarm"])
            for seq, steps, v, dist in self.scan_data:
                pos_mm = steps / self._spmm
                alarm  = "0"
                volt_s = f"{v:.4f}" if v is not None else ""
                dist_s = off_s = ""
                if dist is None:
                    alarm = "1"
                else:
                    dist_s = f"{dist:.4f}"
                    off_s  = f"{dist - SENSOR_CENTER_MM:.4f}"
                w.writerow([seq, steps, f"{pos_mm:.4f}",
                            volt_s, dist_s, off_s, alarm])

        print(f"  Saved {len(self.scan_data)} rows → {os.path.abspath(fname)}")

    # ── Configure ─────────────────────────────────────────────────────────────

    def _run_config(self):
        clear()
        print("  ── Configuration ─────────────────────────────────\n")

        self.pulley_teeth      = prompt_int("GT2 pulley teeth", self.pulley_teeth)
        self.microstep         = prompt_int("Microstep divisor (1/2/4/8/16/32…)", self.microstep, 1, 128)

        hw = self._hw_max
        print(f"\n  Hardware max RPM at {self.microstep}× = {hw:.1f}")
        self.manual_rpm = prompt_float("Manual RPM", self.manual_rpm, MIN_RPM, hw)
        self.scan_rpm   = prompt_float("Scan   RPM", self.scan_rpm,   MIN_RPM, hw)

        print(f"\n  Resolution = sample_steps / {self._spmm:.1f} steps/mm")
        self.scan_sample_steps = prompt_int("SCAN_SAMPLE_STEPS", self.scan_sample_steps, 1, 200)

        print("\n  ── OOR Detection ─────────────────────────────────")
        print("  OOR is checked in Python on each ST:.")
        print("  Arms after N consecutive in-range readings.")
        print("  Once armed, first out-of-range reading sends A to motor.")

        use = input(f"  Enable OOR? [{'Y' if self.use_oor else 'N'}]: ").strip().lower()
        if use: self.use_oor = (use == "y")

        self.oor_lo = delta_dist_to_adc(prompt_int(
            "OOR lower threshold (mm from centre, e.g. -4)",
            int(adc_to_delta_dist(self.oor_lo)), -5, 0))
        self.oor_hi = delta_dist_to_adc(prompt_int(
            "OOR upper threshold (mm from centre, e.g. +4)",
            int(adc_to_delta_dist(self.oor_hi)), 0, 5))
        self.oor_arm_count = prompt_int(
            "Consecutive in-range readings to arm", self.oor_arm_count, 1, 1000)

        self._push_config()
        self._save_config()

        s_d = rpm_to_delay(self.scan_rpm, self._spr)
        print(f"\n  {self._spr} steps/rev   {self._spmm:.3f} steps/mm")
        print(f"  Resolution  : {self._res_mm:.3f} mm/point")
        print(f"  Scan delay  : {s_d}µs → {delay_to_rpm(s_d, self._spr):.1f} RPM actual")
        input("\n  Press Enter to continue.")

# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) == 3:
        motor_port = sys.argv[1]
        sensor_ip  = sys.argv[2]
    else:
        motor_port, sensor_ip = discover()

    app = Gantry(motor_port, sensor_ip)
    app.connect()
    app.run()
