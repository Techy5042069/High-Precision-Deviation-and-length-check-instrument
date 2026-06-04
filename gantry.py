#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# gantry.py  —  Single-Arduino Gantry Controller (host software)
#
# One Arduino handles both motor control and ADC sampling.
# Each scan sample is reported in a single ST:<seq>,<steps>,<adc> line —
# no TICK protocol, no bulk buffering, no TCP/WiFi.
#
# Usage:
#   python gantry.py                  # interactive port discovery
#   python gantry.py COM3             # specify port directly
#   python gantry.py /dev/ttyACM0    # Linux/Mac
# ─────────────────────────────────────────────────────────────────────────────

import csv
import datetime
import json
import os
import queue
import sys
import threading
import time

import serial
import serial.tools.list_ports
from pynput import keyboard

# ─────────────────────────────────────────────────────────────────────────────
# Constants  (mirror config.py — inlined to keep single-file)
# ─────────────────────────────────────────────────────────────────────────────

VERSION   = "1.0"

BAUD_RATE = 921600

SENSOR_CENTER_MM = 30.0
SENSOR_RANGE_MM  = 5.0
ADC_BITS         = 14
ADC_MAX          = (1 << ADC_BITS) - 1   # 16383 for 14-bit
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

CONFIG_FILE      = "gantry_config.json"
LAST_CONN_FILE   = "last_connection.json"

# ─────────────────────────────────────────────────────────────────────────────
# Physics helpers  (mirror physics.py — inlined)
# ─────────────────────────────────────────────────────────────────────────────

def adc_to_voltage(adc: int) -> float:
    return (adc / ADC_MAX) * ADC_REF_V

def voltage_to_dist(v: float) -> float:
    return (SENSOR_CENTER_MM - SENSOR_RANGE_MM) + (v / ADC_REF_V) * (2.0 * SENSOR_RANGE_MM)

def adc_to_dist(adc: int):
    """Return (voltage_V, distance_mm | None, alarm_bool)."""
    v = adc_to_voltage(adc)
    if adc >= ADC_MAX or adc <= 0:
        return v, None, True
    return v, voltage_to_dist(v), False

def calc_spr(microstep: int, full_steps: int = DEFAULT_FULL_STEPS) -> int:
    return full_steps * microstep

def calc_spmm(microstep: int, pulley_teeth: int,
              full_steps: int = DEFAULT_FULL_STEPS,
              pitch: float = DEFAULT_PITCH_MM) -> float:
    return calc_spr(microstep, full_steps) / (pulley_teeth * pitch)

def rpm_to_delay(rpm: float, spr: int) -> int:
    if rpm <= 0:
        return STEP_DELAY_MAX
    delay = int(1_000_000 / (2 * (rpm / 60.0) * spr))
    return max(STEP_DELAY_MIN, min(STEP_DELAY_MAX, delay))

def delay_to_rpm(delay_us: int, spr: int) -> float:
    return (1_000_000 / (2 * delay_us) / spr) * 60.0

def hw_max_rpm(microstep: int) -> float:
    spr = calc_spr(microstep)
    hw_rpm = (1_000_000 / (2 * STEP_DELAY_MIN) / spr) * 60.0
    return min(hw_rpm, MAX_RPM)

def resolution_mm(scan_sample_steps: int, microstep: int, pulley_teeth: int,
                  full_steps: int = DEFAULT_FULL_STEPS,
                  pitch: float = DEFAULT_PITCH_MM) -> float:
    return scan_sample_steps / calc_spmm(microstep, pulley_teeth, full_steps, pitch)

# ─────────────────────────────────────────────────────────────────────────────
# Terminal helpers
# ─────────────────────────────────────────────────────────────────────────────

def clear():
    os.system("cls" if os.name == "nt" else "clear")

def prompt_float(msg: str, default: float,
                 minv: float = None, maxv: float = None) -> float:
    while True:
        raw = input(f"  {msg} [{default}]: ").strip()
        if not raw:
            return default
        try:
            v = float(raw)
            if minv is not None and v < minv:
                print(f"  Min: {minv}"); continue
            if maxv is not None and v > maxv:
                print(f"  Max: {maxv}"); continue
            return v
        except ValueError:
            print("  Enter a number.")

def prompt_int(msg: str, default: int,
               minv: int = 1, maxv: int = None) -> int:
    while True:
        raw = input(f"  {msg} [{default}]: ").strip()
        if not raw:
            return default
        try:
            v = int(raw)
            if v < minv:
                print(f"  Min: {minv}"); continue
            if maxv is not None and v > maxv:
                print(f"  Max: {maxv}"); continue
            return v
        except ValueError:
            print("  Enter an integer.")

# ─────────────────────────────────────────────────────────────────────────────
# SerialNode  —  non-blocking USB-serial wrapper
# ─────────────────────────────────────────────────────────────────────────────

class SerialNode:
    def __init__(self, port: str, baud: int = BAUD_RATE):
        self.port      = port
        self.baud      = baud
        self.ser       = None
        self._lock     = threading.Lock()
        self.lines     = queue.Queue()
        self.connected = False

    def connect(self):
        print(f"  Connecting to Arduino on {self.port} at {self.baud} baud...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2.0)               # wait for Arduino DTR reset / bootloader
        self.ser.reset_input_buffer()
        self.connected = True
        print("  ✓ Arduino connected")
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def send(self, msg: str):
        with self._lock:
            try:
                self.ser.write((msg + "\n").encode())
            except Exception as e:
                print(f"  [Serial] send error: {e}")

    def _recv_loop(self):
        try:
            while True:
                line = self.ser.readline().decode(errors="replace").strip()
                if line:
                    self.lines.put(line)
        except Exception:
            pass
        self.connected = False
        self.lines.put("__DISCONNECTED__")

    def poll(self, timeout: float = 0.0):
        try:
            return self.lines.get(timeout=timeout)
        except queue.Empty:
            return None

# ─────────────────────────────────────────────────────────────────────────────
# Discovery
# ─────────────────────────────────────────────────────────────────────────────

def auto_detect_serial():
    keywords = ("arduino", "ch340", "cp210", "ftdi", "usb", "acm", "r4")
    candidates = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfg  = (p.manufacturer or "").lower()
        if any(k in desc or k in mfg for k in keywords):
            candidates.append(p.device)
    return candidates

def load_last_port():
    try:
        with open(LAST_CONN_FILE) as f:
            return json.load(f).get("port")
    except Exception:
        return None

def save_last_port(port: str):
    try:
        with open(LAST_CONN_FILE, "w") as f:
            json.dump({"port": port}, f, indent=2)
    except Exception as e:
        print(f"  Could not save last port: {e}")

def discover_port() -> str:
    """Interactive port selection. Returns the chosen port string."""
    print(f"\n  Gantry v{VERSION}  —  Single-Arduino Mode")
    print("  ── Port Discovery ─────────────────────────────\n")

    last = load_last_port()
    candidates = auto_detect_serial()

    if last:
        ans = input(f"  Use last port {last}? [Y/n]: ").strip().lower()
        if ans != "n":
            return last

    if candidates:
        print("  Detected ports:")
        for i, p in enumerate(candidates):
            print(f"    {i+1}. {p}")
        idx = input(f"  Select [1-{len(candidates)}] or Enter to type manually: ").strip()
        if idx.isdigit() and 1 <= int(idx) <= len(candidates):
            return candidates[int(idx) - 1]

    return input("  Port (e.g. COM3 / /dev/ttyACM0): ").strip()

# ─────────────────────────────────────────────────────────────────────────────
# GantrySingle  —  top-level controller
# ─────────────────────────────────────────────────────────────────────────────

class GantrySingle:
    """
    Single-Arduino gantry controller.

    Protocol changes vs multi-Arduino version:
      • ST: lines carry three fields:  ST:<seq>,<steps>,<adc>
        The ADC value is embedded — no TICK/BK: exchange needed.
      • OOR abort is self-contained on the Arduino: it emits OOR: then SA
        without waiting for the PC to send "A".  The PC just needs to
        handle the resulting SA and set scan_aborted.
      • No DUMP, no BULK:, no START: for sensor (START: still used for OOR params).
    """

    def __init__(self, port: str):
        self.arduino = SerialNode(port)

        # Config
        self.pulley_teeth      = DEFAULT_PULLEY_TEETH
        self.microstep         = DEFAULT_MICROSTEP
        self.full_steps        = DEFAULT_FULL_STEPS
        self.pitch_mm          = DEFAULT_PITCH_MM
        self.manual_rpm        = DEFAULT_MANUAL_RPM
        self.scan_rpm          = DEFAULT_SCAN_RPM
        self.scan_sample_steps = DEFAULT_SCAN_SAMPLE_STEPS

        # OOR — thresholds are plain mm deviations from sensor centre
        self.use_oor       = False
        self.oor_lo        = -SENSOR_RANGE_MM   # -5.0 mm (full range default)
        self.oor_hi        =  SENSOR_RANGE_MM   # +5.0 mm
        self.oor_arm_count = DEFAULT_OOR_ARM_COUNT

        # Live sensor display
        self.last_volt = None
        self.last_dist = None

        # Limit switch state
        self.limit_a = False
        self.limit_b = False

        # Per-scan data
        self.scan_data    = []   # list of (seq, steps, voltage|None, dist|None)
        self.scan_done    = False
        self.scan_aborted = False
        self.scan_active  = False
        self.scan_total   = 0

        # Manual mode
        self.manual_active = False
        self.active_key    = None

        self._load_config()

    # ── Derived properties ────────────────────────────────────────────────────

    @property
    def _spr(self) -> int:
        return calc_spr(self.microstep, self.full_steps)

    @property
    def _spmm(self) -> float:
        return calc_spmm(self.microstep, self.pulley_teeth,
                         self.full_steps, self.pitch_mm)

    @property
    def _hw_max(self) -> float:
        return hw_max_rpm(self.microstep)

    @property
    def _res_mm(self) -> float:
        return resolution_mm(self.scan_sample_steps, self.microstep,
                             self.pulley_teeth, self.full_steps, self.pitch_mm)

    # ── Config persistence ────────────────────────────────────────────────────

    def _config_dict(self) -> dict:
        return {
            "pulley_teeth":      self.pulley_teeth,
            "microstep":         self.microstep,
            "manual_rpm":        self.manual_rpm,
            "scan_rpm":          self.scan_rpm,
            "scan_sample_steps": self.scan_sample_steps,
            "use_oor":           self.use_oor,
            "oor_lo_mm":         round(self.oor_lo, 4),
            "oor_hi_mm":         round(self.oor_hi, 4),
            "oor_arm_count":     self.oor_arm_count,
        }

    def _load_config(self):
        if not os.path.exists(CONFIG_FILE):
            print(f"  [Config] No '{CONFIG_FILE}' — using defaults.")
            self._save_config()
            return
        try:
            with open(CONFIG_FILE) as f:
                d = json.load(f)
            self.pulley_teeth      = d.get("pulley_teeth",      self.pulley_teeth)
            self.microstep         = d.get("microstep",         self.microstep)
            self.manual_rpm        = d.get("manual_rpm",        self.manual_rpm)
            self.scan_rpm          = d.get("scan_rpm",          self.scan_rpm)
            self.scan_sample_steps = d.get("scan_sample_steps", self.scan_sample_steps)
            self.use_oor           = d.get("use_oor",           self.use_oor)
            # oor_lo_mm / oor_hi_mm are plain mm floats — no ADC conversion
            self.oor_lo        = float(d.get("oor_lo_mm", self.oor_lo))
            self.oor_hi        = float(d.get("oor_hi_mm", self.oor_hi))
            self.oor_arm_count     = d.get("oor_arm_count",     self.oor_arm_count)
            print(f"  [Config] Loaded '{CONFIG_FILE}'.")
        except Exception as e:
            print(f"  [Config] Load error: {e} — using defaults.")

    def _save_config(self):
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(self._config_dict(), f, indent=2)
            print(f"  [Config] Saved '{CONFIG_FILE}'.")
        except Exception as e:
            print(f"  [Config] Save error: {e}")

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self):
        self.arduino.connect()
        time.sleep(0.5)
        # Drain banner messages
        deadline = time.time() + 1.5
        while time.time() < deadline:
            line = self.arduino.poll(timeout=0.05)
            if line:
                print(f"  [Arduino] {line}")
        self._push_config()

    def _push_config(self):
        """Send motion parameters to the Arduino."""
        m_d = rpm_to_delay(self.manual_rpm, self._spr)
        s_d = rpm_to_delay(self.scan_rpm,   self._spr)
        self.arduino.send(f"V{m_d}")
        time.sleep(0.02)
        self.arduino.send(f"W{s_d}")
        time.sleep(0.02)
        self.arduino.send(f"B{self.scan_sample_steps}")

    # ── Message parser ────────────────────────────────────────────────────────

    def _parse(self, line: str):
        """
        Dispatch one line from the Arduino.

        New protocol — ST: carries three comma-separated fields:
            ST:<seq>,<steps>,<adc>

        OOR: is now purely informational on the PC side.  The Arduino has
        already stopped itself (abortScan()) before sending OOR:, so we just
        record scan_aborted here.  We do NOT send "A" back — the motor is
        already stopped.
        """

        # ── Scan telemetry ─────────────────────────────────────────────────
        if line.startswith("ST:"):
            if not self.scan_active:
                return
            parts = line[3:].split(",")
            if len(parts) == 3:
                try:
                    seq   = int(parts[0])
                    steps = int(parts[1])
                    adc   = int(parts[2])
                    v, dist, alarm = adc_to_dist(adc)
                    self.last_volt = v
                    self.last_dist = None if alarm else dist
                    self.scan_data.append((seq, steps, v, None if alarm else dist))
                except ValueError:
                    pass
            return

        # ── Scan done ──────────────────────────────────────────────────────
        if line.startswith("SD:"):
            try:
                self.scan_total = int(line[3:])
            except ValueError:
                self.scan_total = 0
            self.scan_active  = False
            self.scan_done    = True
            return

        # ── Scan start confirmed ───────────────────────────────────────────
        if line == "SS":
            self.scan_data    = []
            self.scan_active  = True
            self.scan_done    = False
            self.scan_aborted = False
            print("\n  [SCAN] Measuring...\n")
            return

        # ── Scan aborted ───────────────────────────────────────────────────
        if line == "SA":
            self.scan_active  = False
            self.scan_aborted = True
            return

        # ── OOR event — Arduino already self-stopped; SA follows immediately ─
        if line.startswith("OOR:"):
            # Firmware sends OOR:<seq>,<adc>,<deviation_mm>
            try:
                parts = line[4:].split(",")
                oar_seq = int(parts[0])
                oar_adc = int(parts[1])
                oar_dev = float(parts[2])
                print(f"\n  ⚠  [OOR FIRED] seq={oar_seq}  adc={oar_adc}  "
                      f"deviation={oar_dev:+.3f}mm — motor stopping")
            except Exception:
                print(f"\n  ⚠  [OOR FIRED] {line}")
            return

        # ── ADC live read ──────────────────────────────────────────────────
        if line.startswith("ADC:"):
            try:
                adc = int(line[4:])
                v, dist, alarm = adc_to_dist(adc)
                self.last_volt = v
                self.last_dist = None if alarm else dist
            except ValueError:
                pass
            return

        # ── Limit switches ─────────────────────────────────────────────────
        if line == "LA":
            self.limit_a = True
            print("  [LIMIT A] Left end ◄")
        elif line == "LB":
            self.limit_b = True
            print("  [LIMIT B] Right end ►")
        elif line == "CA":
            self.limit_a = False
            print("  [INFO] Left limit cleared")
        elif line == "CB":
            self.limit_b = False
            print("  [INFO] Right limit cleared")

        # ── Speed echo ─────────────────────────────────────────────────────
        elif line.startswith("SP:"):
            try:
                rpm = delay_to_rpm(int(line[3:]), self._spr)
                print(f"  [SPEED] {rpm:.1f} RPM")
            except ValueError:
                pass

        elif line.startswith("SS_STEPS:"):
            print(f"  [Arduino] SCAN_SAMPLE_STEPS={line[9:]}")
        elif line.startswith("WN:"):
            print(f"  ⚠  {line[3:]}")
        elif line.startswith("IN:"):
            msg = line[3:]
            if msg.startswith("OOR armed"):
                print(f"  ✔  [OOR ARMED] {msg}")
            elif msg.startswith("oor "):
                # Echo of START: config — always print so user can verify settings
                print(f"  [OOR config] {msg}")
            else:
                print(f"  [Arduino] {msg}")

    # ── Poll thread ───────────────────────────────────────────────────────────

    def _poll_loop(self):
        while True:
            line = self.arduino.poll(timeout=0.002)
            if line:
                self._parse(line)

    # ── Main menu / run ───────────────────────────────────────────────────────

    def run(self):
        threading.Thread(target=self._poll_loop, daemon=True).start()

        while True:
            clear()
            m_d      = rpm_to_delay(self.manual_rpm, self._spr)
            s_d      = rpm_to_delay(self.scan_rpm,   self._spr)
            m_actual = delay_to_rpm(m_d, self._spr)
            s_actual = delay_to_rpm(s_d, self._spr)

            oor_str = (
                f"ON  lo={self.oor_lo:+.2f}mm "
                f"hi={self.oor_hi:+.2f}mm "
                f"arm={self.oor_arm_count}"
                if self.use_oor else "OFF"
            )

            print("╔══════════════════════════════════════════════════════╗")
            print(f"║   Gantry v{VERSION}  —  Single-Arduino Mode               ║")
            print("╠══════════════════════════════════════════════════════╣")
            print(f"║  Port    : {self.arduino.port:<42}║")
            print("╠══════════════════════════════════════════════════════╣")
            print(f"║  Pulley  : {self.pulley_teeth:<5}T  Microstep : {self.microstep:<4}  "
                  f"SPR : {self._spr:<6}    ║")
            print(f"║  Steps/mm : {self._spmm:<8.2f}  Resolution : {self._res_mm:.3f} mm/pt     ║")
            print(f"║  Manual RPM : {self.manual_rpm:<6.1f}  ({m_actual:.1f} actual)            ║")
            print(f"║  Scan   RPM : {self.scan_rpm:<6.1f}  ({s_actual:.1f} actual)            ║")
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

        self.arduino.send("S")
        print("  Bye.")

    # ── Manual jog ────────────────────────────────────────────────────────────

    def _run_manual(self):
        clear()
        print("┌──────────────────────────────────────────────────┐")
        print("│  Manual mode                                     │")
        print("├──────────────────────────────────────────────────┤")
        print("│  ← →    move left / right (hold)                │")
        print("│  ↑ ↓    speed up / down                         │")
        print("│  I      sensor reading                           │")
        print("│  Esc    back to menu                             │")
        print("└──────────────────────────────────────────────────┘\n")

        self.manual_active = True
        self.active_key    = None
        SEND_HZ  = 30
        STOP_DLY = 0.05

        def on_press(key):
            char = None
            try:
                char = key.char.upper() if hasattr(key, "char") and key.char else None
            except Exception:
                pass

            if key == keyboard.Key.right:
                if self.limit_b:
                    print("  ⚠  Right limit active"); return
                if self.active_key != "R":
                    self.active_key = "R"
                    self.arduino.send("R")
                    print("→ Moving right")

            elif key == keyboard.Key.left:
                if self.limit_a:
                    print("  ⚠  Left limit active"); return
                if self.active_key != "L":
                    self.active_key = "L"
                    self.arduino.send("L")
                    print("← Moving left")

            elif key == keyboard.Key.up:
                self.arduino.send("+")
                print("↑ Speed up")

            elif key == keyboard.Key.down:
                self.arduino.send("-")
                print("↓ Speed down")

            elif char == "I":
                self.arduino.send("TICK")
                time.sleep(0.05)   # give Arduino a moment to reply
                if self.last_volt is not None:
                    ds = (f"{self.last_dist:.3f}mm"
                          if self.last_dist is not None
                          else "ALARM (ADC saturated)")
                    print(f"  Sensor: {self.last_volt:.4f}V → {ds}")
                else:
                    print("  Sensor: no reading yet")

            elif key == keyboard.Key.esc:
                self.active_key    = None
                self.manual_active = False
                self.arduino.send("S")
                return False

        def on_release(key):
            if key in (keyboard.Key.right, keyboard.Key.left):
                self.active_key = None
                time.sleep(STOP_DLY)
                self.arduino.send("S")
                print("■ Stopped")

        def hold_loop():
            iv = 1.0 / SEND_HZ
            while self.manual_active:
                if   self.active_key == "R": self.arduino.send("R")
                elif self.active_key == "L": self.arduino.send("L")
                time.sleep(iv)

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
        print(f"  Sample steps  : {self.scan_sample_steps}  ({self._res_mm:.2f} mm resolution)")

        if self.use_oor:
            print(f"  OOR           : lo={self.oor_lo:+.2f}mm  "
                  f"hi={self.oor_hi:+.2f}mm  "
                  f"arm={self.oor_arm_count}")
        else:
            print("  OOR           : disabled")

        if input("\n  Start scan? [y/N]: ").strip().lower() != "y":
            print("  Cancelled.")
            time.sleep(1)
            return

        # Reset per-scan state
        self.scan_data    = []
        self.scan_done    = False
        self.scan_aborted = False
        self.scan_active  = False
        self.scan_total   = 0

        # Send OOR config then kick off scan
        start_cmd = (
            f"START:"
            f"{1 if self.use_oor else 0},"
            f"{self.oor_lo:.4f},"
            f"{self.oor_hi:.4f},"
            f"{self.oor_arm_count}"
        )
        self.arduino.send(start_cmd)
        time.sleep(0.01)
        self._push_config()
        time.sleep(0.01)
        print("  [SCAN] Homing to left limit...")
        self.arduino.send("X")

        try:
            while not self.scan_done and not self.scan_aborted:
                time.sleep(0.005)
        except KeyboardInterrupt:
            print("\n  [ABORT] Ctrl+C — stopping...")
            self.arduino.send("A")
            self.scan_aborted = True

        # Give Arduino a moment to flush any last ST: lines
        time.sleep(0.1)

        print(f"\n  Samples collected : {len(self.scan_data)}")

        # ── Aborted path ──────────────────────────────────────────────────────
        # scan_done takes priority: OOR sometimes fires on the last reading just
        # as the gantry reaches the right limit; if SD: arrives, treat as complete.
        if self.scan_aborted and not self.scan_done:
            self._trim_to_oor_window()
            ans = input("\n  Save partial / OOR-trimmed scan? [Y/n]: ").strip().lower()
            if ans != "n":
                self._show_plot()
                self._save_csv()
            input("\n  Press Enter.")
            return

        # ── Normal completion ─────────────────────────────────────────────────
        self._trim_to_oor_window()
        self._show_plot()
        self._save_csv()
        input("\n  Press Enter to return to menu.")

    # ── OOR window trim ───────────────────────────────────────────────────────

    def _trim_to_oor_window(self):
        """
        When OOR is enabled, discard data outside the surface window and
        re-zero gantry position to the arm point.

        Phase 1 — find arm index: walk from start, count consecutive in-range
                  points; when count reaches oor_arm_count, arm is set.
        Phase 2 — find disarm index: first out-of-range point after arm.
        Re-zero: subtract step count at arm from all kept points.
        """
        if not self.use_oor or not self.scan_data:
            return

        def in_range(dist):
            if dist is None:
                return False
            return self.oor_lo <= (dist - SENSOR_CENTER_MM) <= self.oor_hi

        # Phase 1
        arm_index = None
        consec    = 0
        for i, (_, _, _, dist) in enumerate(self.scan_data):
            if in_range(dist):
                consec += 1
                if consec >= self.oor_arm_count:
                    arm_index = i - self.oor_arm_count + 1
                    break
            else:
                consec = 0

        if arm_index is None:
            print("  [Trim] OOR enabled but surface never entered range — keeping all data.")
            return

        # Phase 2
        disarm_index = len(self.scan_data)
        for i in range(arm_index + 1, len(self.scan_data)):
            if not in_range(self.scan_data[i][3]):
                disarm_index = i
                break

        window       = self.scan_data[arm_index:disarm_index]
        origin_steps = window[0][1]
        self.scan_data = [
            (seq, steps - origin_steps, v, dist)
            for seq, steps, v, dist in window
        ]
        print(f"  [Trim] {len(self.scan_data)} pts  "
              f"start={origin_steps / self._spmm:.2f}mm  "
              f"end={self.scan_data[-1][1] / self._spmm:.2f}mm")

    # ── Plot ──────────────────────────────────────────────────────────────────

    def _show_plot(self):
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("  matplotlib not installed — skipping plot.  (pip install matplotlib)")
            return

        steps_list = [d[1] for d in self.scan_data]
        pos_list   = [s / self._spmm for s in steps_list]

        # Deviation from centre: positive = farther, negative = closer.
        # None / alarm points become NaN so matplotlib draws a gap instead of
        # connecting across missing data.
        deviations = [
            (d[3] - SENSOR_CENTER_MM) if d[3] is not None else float("nan")
            for d in self.scan_data
        ]

        fig, ax = plt.subplots(figsize=(13, 5))
        fig.suptitle(
            f"Gantry Scan v{VERSION} — {len(self.scan_data)} pts  "
            f"res={self._res_mm:.2f}mm  {self.scan_rpm:.0f}RPM",
            fontsize=12,
        )

        # ── OOR acceptance band (green) — only when OOR is enabled ───────────
        if self.use_oor:
            ax.axhspan(
                self.oor_lo, self.oor_hi,
                color="#2ECC71", alpha=0.18, zorder=0,
                label=f"OOR window  [{self.oor_lo:+.2f}, {self.oor_hi:+.2f}] mm",
            )
            ax.axhline(self.oor_lo, color="#27AE60", lw=0.8, ls="--")
            ax.axhline(self.oor_hi, color="#27AE60", lw=0.8, ls="--")

        # ── Zero reference ────────────────────────────────────────────────────
        ax.axhline(0.0, color="#888780", lw=0.8, ls=":", label="centre (0 mm)")

        # ── Deviation trace ───────────────────────────────────────────────────
        ax.plot(pos_list, deviations, color="#185FA5", lw=0.9, label="deviation")

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
        fname = input("  Filename: ").strip() or default
        if not fname.endswith(".csv"):
            fname += ".csv"

        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "seq", "step_count", "gantry_position_mm",
                "sensor_voltage_V", "sensor_distance_mm",
                "offset_from_center_mm", "alarm",
            ])
            for seq, steps, v, dist in self.scan_data:
                pos_mm = steps / self._spmm
                alarm  = "0"
                dist_s = ""
                off_s  = ""
                volt_s = ""
                if v is not None:
                    volt_s = f"{v:.4f}"
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
        print("  ── Configuration ────────────────────────────────\n")

        self.pulley_teeth = prompt_int(
            "GT2 pulley teeth", self.pulley_teeth)

        self.microstep = prompt_int(
            "Microstep divisor (1/2/4/8/16/32…)", self.microstep, 1, 128)

        hw_max = self._hw_max
        print(f"\n  Hardware max RPM at {self.microstep}× = {hw_max:.1f}")
        self.manual_rpm = prompt_float("Manual RPM", self.manual_rpm, MIN_RPM, hw_max)
        self.scan_rpm   = prompt_float("Scan   RPM", self.scan_rpm,   MIN_RPM, hw_max)

        print(f"\n  Resolution = sample_steps / {self._spmm:.1f} steps/mm")
        self.scan_sample_steps = prompt_int(
            "SCAN_SAMPLE_STEPS", self.scan_sample_steps, 1, 200)

        print("\n  ── OOR Detection ──────────────────────────────────")
        print("  Arduino arms OOR after N consecutive in-range readings.")
        print("  Once armed, the first out-of-range reading stops the scan immediately.")

        use = input(
            f"  Enable OOR? [{'Y' if self.use_oor else 'N'}]: "
        ).strip().lower()
        if use:
            self.use_oor = (use == "y")

        self.oor_lo = prompt_float(
            "OOR lower threshold mm (offset from centre, e.g. -3.5)",
            self.oor_lo, -SENSOR_RANGE_MM, 0.0)

        self.oor_hi = prompt_float(
            "OOR upper threshold mm (offset from centre, e.g. +3.5)",
            self.oor_hi, 0.0, SENSOR_RANGE_MM)

        self.oor_arm_count = prompt_int(
            "Consecutive in-range readings to arm OOR",
            self.oor_arm_count, 1, 1000)

        self._push_config()
        self._save_config()

        # Summary
        s_d = rpm_to_delay(self.scan_rpm, self._spr)
        print(f"\n  {self._spr} steps/rev   {self._spmm:.3f} steps/mm")
        print(f"  Resolution  : {self._res_mm:.3f} mm/point")
        print(f"  Scan delay  : {s_d}µs → {delay_to_rpm(s_d, self._spr):.1f} RPM actual")
        input("\n  Press Enter to continue.")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    else:
        port = discover_port()

    save_last_port(port)

    app = GantrySingle(port)
    app.connect()
    app.run()
