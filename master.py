# ─────────────────────────────────────────────────────────────────────────────
# master.py
#
# GantryMaster — the top-level controller object.
#
# Responsibilities:
#   • Config: load/save parameters to gantry_config.json
#   • Connection: open motor (serial) and sensor (TCP) nodes
#   • Poll loop: drain incoming queues and dispatch to parsers
#   • Scan: coordinate motor homing, measurement pass, sensor TICKs
#   • Manual mode: keyboard-driven jog
#   • Data: join motor positions with sensor readings, plot, export CSV
#   • UI: text menus for all of the above
#
# Threading summary:
#   Thread             Owner        Purpose
#   ─────────────────────────────────────────────────────────────────
#   recv (serial)      SerialNode   read bytes from motor Arduino
#   recv (TCP)         TCPNode      read bytes from sensor Arduino
#   poll               GantryMaster drain both queues → call _parse_*
#   hold               _run_manual  re-send direction command at 30 Hz
#   main               OS           UI menus, user input, blocking waits
# ─────────────────────────────────────────────────────────────────────────────

import csv
import datetime
import json
import os
import threading
import time

from pynput import keyboard

from config import (
    MOTOR_BAUD, SENSOR_PORT,
    SENSOR_CENTER_MM, SENSOR_RANGE_MM,
    MIN_RPM,
    DEFAULT_PULLEY_TEETH, DEFAULT_MICROSTEP, DEFAULT_FULL_STEPS,
    DEFAULT_PITCH_MM, DEFAULT_MANUAL_RPM, DEFAULT_SCAN_RPM,
    DEFAULT_SCAN_SAMPLE_STEPS, DEFAULT_BULK_SIZE,
    DEFAULT_OOR_ARM_COUNT, CONFIG_FILE,
)
from physics import (
    adc_to_dist, calc_spr, calc_spmm,
    rpm_to_delay, delay_to_rpm, hw_max_rpm, resolution_mm, delta_dist_to_adc, adc_to_delta_dist
)
from nodes import SerialNode, TCPNode
from version import VERSION


# ══════════════════════════════════════════════════════════════════════════════
# Terminal helpers
# ══════════════════════════════════════════════════════════════════════════════

def clear():
    """Clear the terminal screen (works on Windows and Unix)."""
    os.system("cls" if os.name == "nt" else "clear")


def prompt_float(msg: str, default: float,
                 minv: float = None, maxv: float = None) -> float:
    """
    Prompt the user for a float value, looping until a valid entry is made.

    Args:
        msg:     Prompt text (displayed with the default in brackets).
        default: Returned immediately if the user presses Enter.
        minv:    Optional inclusive lower bound.
        maxv:    Optional inclusive upper bound.
    """
    while True:
        raw = input(f"  {msg} [{default}]: ").strip()
        if not raw:
            return default
        try:
            v = float(raw)
            if minv is not None and v < minv:
                print(f"  Min: {minv}")
                continue
            if maxv is not None and v > maxv:
                print(f"  Max: {maxv}")
                continue
            return v
        except ValueError:
            print("  Enter a number.")


def prompt_int(msg: str, default: int,
               minv: int = 1, maxv: int = None) -> int:
    """
    Prompt the user for an integer value, looping until valid.

    Args:
        msg:     Prompt text.
        default: Returned immediately if the user presses Enter.
        minv:    Inclusive lower bound (default 1).
        maxv:    Optional inclusive upper bound.
    """
    while True:
        raw = input(f"  {msg} [{default}]: ").strip()
        if not raw:
            return default
        try:
            v = int(raw)
            if v < minv:
                print(f"  Min: {minv}")
                continue
            if maxv is not None and v > maxv:
                print(f"  Max: {maxv}")
                continue
            return v
        except ValueError:
            print("  Enter an integer.")


# ══════════════════════════════════════════════════════════════════════════════
# GantryMaster
# ══════════════════════════════════════════════════════════════════════════════

class GantryMaster:
    """
    Top-level gantry controller.  One instance per session.

    Lifecycle:
        app = GantryMaster(motor_port, sensor_ip)
        app.connect()   # opens serial + TCP, pushes config to Arduinos
        app.run()       # blocks until user quits
    """

    def __init__(self, motor_port: str, sensor_ip: str):
        # ── Communication nodes ───────────────────────────────────────────────
        self.motor  = SerialNode("Motor",  motor_port, MOTOR_BAUD)
        self.sensor = TCPNode  ("Sensor", sensor_ip,  SENSOR_PORT)

        # ── Config (overwritten by _load_config if gantry_config.json exists) ─
        self.pulley_teeth         = DEFAULT_PULLEY_TEETH
        self.microstep            = DEFAULT_MICROSTEP
        self.full_steps           = DEFAULT_FULL_STEPS       # fixed hardware — not in config UI
        self.pitch_mm             = DEFAULT_PITCH_MM         # fixed hardware — not in config UI
        self.manual_rpm           = DEFAULT_MANUAL_RPM
        self.scan_rpm             = DEFAULT_SCAN_RPM
        self.scan_sample_steps    = DEFAULT_SCAN_SAMPLE_STEPS
        self.bulk_size            = DEFAULT_BULK_SIZE

        # ── Live sensor display (updated by _parse_sensor during any scan) ────
        self.last_volt = None   # most recent sensor voltage in V (float | None)
        self.last_dist = None   # most recent sensor distance in mm (float | None)

        # ── Limit switch state (updated by _parse_motor) ──────────────────────
        self.limit_a = False    # True while left  (A) limit switch is active
        self.limit_b = False    # True while right (B) limit switch is active

        # ── Per-scan data structures (reset in _run_scan before each scan) ────
        # Populated during the scan pass:
        self.motor_map  = {}    # seq_number → adjusted step count
        self.sensor_map = {}    # seq_number → raw ADC reading

        # Scan status flags (set by _parse_motor in the poll thread):
        self.scan_active  = False  # True from SS until SD/SA
        self.scan_done    = False  # True after SD: (right limit reached)
        self.scan_aborted = False  # True after SA or Ctrl+C
        self.scan_total   = 0      # total ST sequences the motor reported

        # Assembled output (populated by _match() after the scan):
        self.scan_data = []        # list of (seq, steps, voltage|None, dist|None)


        # ── Manual mode state ─────────────────────────────────────────────────
        self.manual_active = False  # True while the manual jog loop is running
        self.active_key    = None   # 'R', 'L', or None

        self.use_oor      = False
        self.oor_lo       = 0
        self.oor_hi       = 1023
        self.oor_arm_count = DEFAULT_OOR_ARM_COUNT  # consecutive in-range readings to arm OOR

        # Load saved config — overwrites all defaults above if the file exists
        self._load_config()

    # ══════════════════════════════════════════════════════════════════════════
    # Config persistence
    # ══════════════════════════════════════════════════════════════════════════

    def _config_dict(self) -> dict:
        """Serialise all user-tunable config fields to a plain dict for JSON."""
        return {
            "pulley_teeth":         self.pulley_teeth,
            "microstep":            self.microstep,
            "manual_rpm":           self.manual_rpm,
            "scan_rpm":             self.scan_rpm,
            "scan_sample_steps":    self.scan_sample_steps,
            "bulk_size":            self.bulk_size,
            "use_oor":             self.use_oor,
            "oor_lo":              self.oor_lo,
            "oor_hi":              self.oor_hi,
            "oor_arm_count":       self.oor_arm_count,
        }

    def _load_config(self):
        """
        Load config from CONFIG_FILE (gantry_config.json).
        If the file does not exist, write the current defaults so the file
        is ready for next launch.
        Missing keys in the file are silently skipped (keeps old values).
        """
        if not os.path.exists(CONFIG_FILE):
            print(f"  [Config] No '{CONFIG_FILE}' found — using defaults.")
            self._save_config()    # persist defaults for next run
            return
        try:
            with open(CONFIG_FILE, "r") as f:
                d = json.load(f)
            self.pulley_teeth         = d.get("pulley_teeth",         self.pulley_teeth)
            self.microstep            = d.get("microstep",            self.microstep)
            self.manual_rpm           = d.get("manual_rpm",           self.manual_rpm)
            self.scan_rpm             = d.get("scan_rpm",             self.scan_rpm)
            self.scan_sample_steps    = d.get("scan_sample_steps",    self.scan_sample_steps)
            self.bulk_size            = d.get("bulk_size",            self.bulk_size)
            self.use_oor             = d.get("use_oor", self.use_oor)
            self.oor_lo              = d.get("oor_lo", self.oor_lo)
            self.oor_hi              = d.get("oor_hi", self.oor_hi)
            self.oor_arm_count       = d.get("oor_arm_count", self.oor_arm_count)
            print(f"  [Config] Loaded from '{CONFIG_FILE}'.")
        except Exception as e:
            print(f"  [Config] Load error: {e} — using defaults.")

    def _save_config(self):
        """Write current config to CONFIG_FILE as formatted JSON."""
        try:
            with open(CONFIG_FILE, "w") as f:
                json.dump(self._config_dict(), f, indent=2)
            print(f"  [Config] Saved to '{CONFIG_FILE}'.")
        except Exception as e:
            print(f"  [Config] Save error: {e}")

    # ══════════════════════════════════════════════════════════════════════════
    # Derived properties
    # ══════════════════════════════════════════════════════════════════════════

    @property
    def _spr(self) -> int:
        """Steps per revolution for the current microstep setting."""
        return calc_spr(self.microstep, self.full_steps)

    @property
    def _spmm(self) -> float:
        """Steps per millimetre for the current pulley/microstep config."""
        return calc_spmm(self.microstep, self.pulley_teeth,
                         self.full_steps, self.pitch_mm)

    @property
    def _hw_max(self) -> float:
        """Hardware RPM ceiling — depends on microstep divisor."""
        return hw_max_rpm(self.microstep)

    @property
    def _res_mm(self) -> float:
        """Spatial resolution: mm between consecutive sensor samples."""
        return resolution_mm(self.scan_sample_steps, self.microstep,
                             self.pulley_teeth, self.full_steps, self.pitch_mm)

    # ══════════════════════════════════════════════════════════════════════════
    # Connection
    # ══════════════════════════════════════════════════════════════════════════

    def connect(self):
        """
        Open both hardware connections and push the current config.
        Call this once before run().
        """
        self.motor.connect()
        self.sensor.connect()
        time.sleep(0.5)
        # Drain startup banners from both Arduinos
        for node in (self.motor, self.sensor):
            deadline = time.time() + 1.5
            while time.time() < deadline:
                line = node.poll(timeout=0.05)
                if line:
                    print(f"  [{node.name}] {line}")
        self._push_config()

    def _push_config(self):
        """
        Send current parameters to both Arduinos.
        Called once on connect, and again after any config change.

        Motor receives:
          V<delay>  — manual step delay (µs)
          W<delay>  — scan step delay (µs)
          B<steps>  — scan sample steps (steps between ST: telemetry)

        Sensor receives:
          BULK:<n>  — readings to accumulate before sending a BK: packet
        """
        m_d = rpm_to_delay(self.manual_rpm, self._spr)
        s_d = rpm_to_delay(self.scan_rpm,   self._spr)

        self.motor.send(f"V{m_d}")                      # manual speed
        time.sleep(0.02)
        self.motor.send(f"W{s_d}")                      # scan speed
        time.sleep(0.02)
        self.motor.send(f"B{self.scan_sample_steps}")   # sample interval
        time.sleep(0.02)
        self.sensor.send(f"BULK:{self.bulk_size}")       # bulk packet size

    # ══════════════════════════════════════════════════════════════════════════
    # Message parsers
    # ══════════════════════════════════════════════════════════════════════════

    def _parse_motor(self, line: str):
        """
        Dispatch a single line received from the motor Arduino.

        Protocol (Arduino → PC):
          SS                   scan start confirmed (homed, now going right)
          ST:<seq>,<steps>     step telemetry — one per scan_sample_steps steps
          SD:<total>           scan done — motor hit right limit switch
          SA                   scan aborted (A command acknowledged)
          LA / LB              left / right limit switch triggered
          CA / CB              left / right limit switch released
          SP:<delay>           speed echo (µs delay)
          SS_STEPS:<n>         echo of current scan sample steps
          WN:<msg>             warning
          IN:<msg>             info / banner
        """
        if self.scan_aborted:
            return   # ignore any trailing messages after abort
        if line.startswith("ST:"):
            parts = line[3:].split(",")
            if len(parts) == 2 and self.scan_active:
                try:
                    seq   = int(parts[0])
                    steps = int(parts[1])
                    self.motor_map[seq] = steps
                    self.sensor.send("TICK")

                except ValueError:
                    pass

        elif line == "SS":
            # Motor has homed to left limit and is beginning the rightward pass
            self.motor_map   = {}
            self.sensor_map  = {}
            self.scan_active = True
            print("\n  [SCAN] Measuring...\n")

        elif line.startswith("SD:"):
            # Scan complete — motor reached the right limit switch
            try:
                self.scan_total = int(line[3:])
            except ValueError:
                self.scan_total = 0
            self.scan_active = False
            self.scan_done   = True

        elif line == "SA":
            # Abort acknowledged by motor
            self.scan_active  = False
            self.scan_aborted = True

        elif line == "LA":
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

        elif line.startswith("SP:"):
            try:
                rpm = delay_to_rpm(int(line[3:]), self._spr)
                print(f"  [SPEED] {rpm:.1f} RPM")
            except ValueError:
                pass

        elif line.startswith("SS_STEPS:"):
            print(f"  [Motor] SCAN_SAMPLE_STEPS={line[9:]}")
        elif line.startswith("WN:"):
            print(f"  ⚠  {line[3:]}")
        elif line.startswith("IN:"):
            print(f"  [Motor] {line[3:]}")

    def _parse_sensor(self, line: str):
        """
        Dispatch a single line received from the sensor Arduino.

        Protocol (Arduino → PC):
          BK:<seq_start>,<adc0>,<adc1>,...   bulk ADC readings
          IN:<msg>                             info / banner
          WN:<msg>                             warning
        """
        if line.startswith("BK:"):
            # ── Bulk ADC packet ────────────────────────────────────────────────
            # Format: BK:<first_seq>,<adc>,<adc>,...
            # The sensor assigns seq numbers matching the TICK commands it received,
            # so sensor seq N corresponds to motor ST seq N.
            parts = line[3:].split(",")
            if len(parts) >= 2:
                try:
                    seq_start = int(parts[0])
                    for i, adc_s in enumerate(parts[1:]):
                        adc = int(adc_s)
                        seq = seq_start + i
                        self.sensor_map[seq] = adc
                        # Update live display for manual mode 'I' key
                        v, dist, alarm = adc_to_dist(adc)
                        self.last_volt = v
                        self.last_dist = None if alarm else dist
                except ValueError:
                    pass   # malformed packet — skip

        elif line.startswith("IN:"):
            print(f"  [Sensor] {line[3:]}")
        elif line.startswith("WN:"):
            print(f"  ⚠  [Sensor] {line[3:]}")
        elif line.startswith("ADC:"):
            try:
                adc = int(line[4:])
                v, dist, alarm = adc_to_dist(adc)
                self.last_volt = v
                self.last_dist = None if alarm else dist
            except ValueError:
                pass
        elif line.startswith("OOR:"):

            print(f"\n  [OOR DETECTED] {line}")

            self.motor.send("A")

            self.scan_aborted = True

    # ══════════════════════════════════════════════════════════════════════════
    # Poll thread
    # ══════════════════════════════════════════════════════════════════════════

    def _poll_loop(self):
        """
        Long-lived background thread that drains both node queues.
        Runs as a daemon from run() until the process exits.

        The 2 ms timeout in poll() lets the thread yield frequently without
        burning 100% CPU in a spin loop.
        """
        while True:
            m = self.motor.poll(timeout=0.002)
            if m:
                self._parse_motor(m)
            s = self.sensor.poll(timeout=0.002)
            if s:
                self._parse_sensor(s)

    # ══════════════════════════════════════════════════════════════════════════
    # Data matching
    # ══════════════════════════════════════════════════════════════════════════

    def _match(self):
        """
        Join motor_map and sensor_map on their shared sequence number.

        For each seq in motor_map (sorted ascending):
          - If sensor_map[seq] exists → convert ADC to voltage + distance
          - Otherwise              → mark as missing sensor reading

        Populates self.scan_data as a list of tuples:
            (seq, adjusted_steps, voltage | None, distance | None)

        Prints a summary of how many points were matched vs missing.
        """
        self.scan_data = []
        missing = 0

        for seq in sorted(self.motor_map.keys()):
            steps = self.motor_map[seq]
            adc   = self.sensor_map.get(seq, None)

            if adc is not None:
                v, dist, alarm = adc_to_dist(adc)
                # dist is None when alarm (ADC saturated) — stored as None in scan_data
                self.scan_data.append((seq, steps, v, None if alarm else dist))
            else:
                # No sensor reading arrived for this seq
                self.scan_data.append((seq, steps, None, None))
                missing += 1

        total = len(self.scan_data)
        print(f"  Matched: {total} points  "
              f"({total - missing} with sensor data, {missing} missing)")

    # ══════════════════════════════════════════════════════════════════════════
    # Main menu
    # ══════════════════════════════════════════════════════════════════════════

    def run(self):
        """
        Main UI loop.  Starts the poll thread, then loops showing the menu
        until the user selects Quit.
        """
        threading.Thread(target=self._poll_loop, daemon=True).start()

        while True:
            clear()
            m_d      = rpm_to_delay(self.manual_rpm, self._spr)
            s_d      = rpm_to_delay(self.scan_rpm,   self._spr)
            m_actual = delay_to_rpm(m_d, self._spr)
            s_actual = delay_to_rpm(s_d, self._spr)

            print("╔══════════════════════════════════════════════════════╗")
            print(f"║   Gantry Master v{VERSION}  —  Main Menu                 ║")
            print("╠══════════════════════════════════════════════════════╣")
            print(f"║  Motor  : {self.motor.port:<15}  Serial {MOTOR_BAUD}        ║")
            print(f"║  Sensor : {self.sensor.host:<15}  port {SENSOR_PORT:<5}          ║")
            print("╠══════════════════════════════════════════════════════╣")
            print(f"║  Pulley : {self.pulley_teeth:<5}T   Microstep : {self.microstep:<4}  "
                  f"SPR : {self._spr:<6}     ║")
            print(f"║  Steps/mm : {self._spmm:<8.2f}  Resolution : {self._res_mm:.2f} mm/pt       ║")
            print(f"║  Manual RPM : {self.manual_rpm:<6.1f}  ({m_actual:.1f} actual)             ║")
            print(f"║  Scan   RPM : {self.scan_rpm:<6.1f}  ({s_actual:.1f} actual)             ║")
            print(f"║  Scan sample steps : {self.scan_sample_steps:<5}  "
                  f"Bulk size : {self.bulk_size:<5}           ║")
            print("╠══════════════════════════════════════════════════════╣")
            print("║  1.  Manual mode                                     ║")
            print("║  2.  Full Scan                                       ║")
            print("║  3.  Configure                                       ║")
            print("║  Q.  Quit                                            ║")
            print("╚══════════════════════════════════════════════════════╝")

            ch = input("\n  Select: ").strip().upper()
            if   ch == "1":
                self._run_manual()
            elif ch == "2":
                self._run_scan()
            elif ch == "3":
                self._run_config()
            elif ch == "Q":
                break

        # Graceful shutdown — stop motor and sensor before exit
        self.motor.send("S")
        self.sensor.send("STOP")
        print("  Bye.")

    # ══════════════════════════════════════════════════════════════════════════
    # Configure menu
    # ══════════════════════════════════════════════════════════════════════════

    def _run_config(self):
        """
        Interactive configuration.  All changes are immediately pushed to
        the Arduinos and saved to gantry_config.json.
        """
        clear()
        print("  ── Configuration ────────────────────────────────\n")

        self.pulley_teeth = prompt_int(
            "GT2 pulley teeth", self.pulley_teeth)

        self.microstep = prompt_int(
            "Microstep divisor (1/2/4/8/16/32…)", self.microstep, 1, 128)

        hw_max = self._hw_max
        print(f"\n  Hardware max RPM at {self.microstep}× microstep = {hw_max:.1f}")
        self.manual_rpm = prompt_float(
            "Manual RPM", self.manual_rpm, MIN_RPM, hw_max)
        self.scan_rpm = prompt_float(
            "Scan   RPM", self.scan_rpm, MIN_RPM, hw_max)

        print(f"\n  Resolution = SCAN_SAMPLE_STEPS / {self._spmm:.1f} steps/mm")
        self.scan_sample_steps = prompt_int(
            "SCAN_SAMPLE_STEPS", self.scan_sample_steps, 1, 200)

        self.bulk_size = prompt_int(
            "Sensor bulk packet size", self.bulk_size, 1, 100)

        print("\n  ── OOR Detection ─────────────────────────────")
        print("  Sensor arms OOR detection after seeing N consecutive")
        print("  in-range readings.  While not yet armed, out-of-range")
        print("  readings simply reset the counter (pre-object region).")
        print("  Once armed, the first out-of-range reading stops the scan.")

        use = input(
            f"  Enable OOR detection? "
            f"[{'Y' if self.use_oor else 'N'}]: "
        ).strip().lower()

        if use:
            self.use_oor = (use == "y")

        self.oor_lo = delta_dist_to_adc(prompt_int(
            "OOR lower threshold",
            adc_to_delta_dist(self.oor_lo),
            -5, 0
        ))

        self.oor_hi = delta_dist_to_adc(prompt_int(
            "OOR upper threshold",
            adc_to_delta_dist(self.oor_hi),
            0, 5
        ))

        self.oor_arm_count = prompt_int(
            "Consecutive in-range readings to arm OOR",
            self.oor_arm_count, 1, 1000
        )

        # Push to hardware and persist
        self._push_config()
        self._save_config()

        # Show derived summary
        s_d = rpm_to_delay(self.scan_rpm, self._spr)
        print(f"\n  {self._spr} steps/rev   {self._spmm:.3f} steps/mm")
        print(f"  Resolution  : {self._res_mm:.3f} mm/point")
        print(f"  Scan delay  : {s_d}µs → {delay_to_rpm(s_d, self._spr):.1f} RPM actual")
        input("\n  Press Enter to continue.")

    # ══════════════════════════════════════════════════════════════════════════
    # Manual mode
    # ══════════════════════════════════════════════════════════════════════════

    def _run_manual(self):
        """
        Keyboard-driven jog mode.

        Controls:
          ← →    Move left / right (hold key to keep moving)
          ↑ ↓    Increase / decrease speed
          I      Print current sensor voltage and distance
          Esc    Return to main menu

        The hold_loop thread re-sends the active direction command at 30 Hz
        so the motor keeps moving while the key is held — the Arduino treats
        each R/L command as "move one burst" without the hold loop.
        """
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
        SEND_HZ  = 30       # direction command re-send rate while key is held
        STOP_DLY = 0.05     # small delay before sending S command on key release

        def on_press(key):
            """Called by pynput on every key press."""
            char = None
            try:
                char = key.char.upper() if hasattr(key, "char") and key.char else None
            except Exception:
                pass

            if key == keyboard.Key.right:
                if self.limit_b:
                    print("  ⚠  Right limit active")
                    return
                if self.active_key != "R":
                    self.active_key = "R"
                    self.motor.send("R")
                    print("→ Moving right")

            elif key == keyboard.Key.left:
                if self.limit_a:
                    print("  ⚠  Left limit active")
                    return
                if self.active_key != "L":
                    self.active_key = "L"
                    self.motor.send("L")
                    print("← Moving left")

            elif key == keyboard.Key.up:
                self.motor.send("+")
                print("↑ Speed up")

            elif key == keyboard.Key.down:
                self.motor.send("-")
                print("↓ Speed down")

            elif char == "I":
                # Print the most recent sensor reading
                self.sensor.send("TICK")
                if self.last_volt is not None:
                    ds = (f"{self.last_dist:.3f}mm"
                          if self.last_dist is not None
                          else "ALARM (ADC saturated)")
                    print(f"  Sensor: {self.last_volt:.4f}V → {ds}")
                else:
                    print("  Sensor: no reading yet")

            elif key == keyboard.Key.esc:
                # Stop motor and exit manual mode
                self.active_key = None
                self.motor.send("S")
                self.manual_active = False
                return False   # signal pynput to stop the listener

        def on_release(key):
            """Stop the motor when a direction key is released."""
            if key in (keyboard.Key.right, keyboard.Key.left):
                self.active_key = None
                time.sleep(STOP_DLY)
                self.motor.send("S")
                print("■ Stopped")

        def hold_loop():
            """Re-send the active direction at SEND_HZ while manual_active."""
            iv = 1.0 / SEND_HZ
            while self.manual_active:
                if   self.active_key == "R":
                    self.motor.send("R")
                elif self.active_key == "L":
                    self.motor.send("L")
                time.sleep(iv)

        threading.Thread(target=hold_loop, daemon=True).start()
        with keyboard.Listener(on_press=on_press, on_release=on_release) as lst:
            lst.join()
        self.manual_active = False

    # ══════════════════════════════════════════════════════════════════════════
    # Full Scan
    # ══════════════════════════════════════════════════════════════════════════

    def _run_scan(self):
        """
        Execute a complete gantry scan pass.

        Sequence of events:
          1. Reset all scan state.
          2. Send START to sensor (resets its buffer and seq counter).
          3. Send X to motor → motor homes to left limit, then drives right.
          4. Motor sends SS when measurement pass begins.
          5. For each SCAN_SAMPLE_STEPS steps, motor sends ST:<seq>,<steps>.
          6. PC sends TICK to sensor → sensor reads ADC, assigns same seq.
          7. Sensor streams BK: bulk packets back to PC.
          8. Motor sends SD: when right limit is hit → scan_done = True.
          9. PC sends DUMP to flush any partial bulk buffer on sensor.
         10. _match() joins motor_map and sensor_map on seq.
         11. _show_plot() and _save_csv() present the data.

        Ctrl+C at any point sends an abort command to the motor and saves
        whatever partial data has been collected.
        """
        clear()
        print("┌──────────────────────────────────────────────────┐")
        print(f"│  Full Scan  —  Gantry Master v{VERSION:<20}  │")
        print("│  Ctrl+C to abort                                 │")
        print("└──────────────────────────────────────────────────┘\n")
        print(f"  Scan RPM       : {self.scan_rpm:.1f}")
        print(f"  Sample steps   : {self.scan_sample_steps}  ({self._res_mm:.2f}mm resolution)")
        print(f"  Bulk size      : {self.bulk_size}")

        if input("  Start scan? [y/N]: ").strip().lower() != "y":
            print("  Cancelled.")
            time.sleep(1)
            return

        # ── Reset all per-scan state ──────────────────────────────────────────
        self.motor_map                = {}
        self.sensor_map               = {}
        self.scan_data                = []
        self.scan_done                = False
        self.scan_aborted             = False
        self.scan_total               = 0

        # ── Start scan ────────────────────────────────────────────────────────
        # Send OOR arm count directly — sensor arms OOR after it has seen
        # oor_arm_count consecutive in-range readings, so arming is always
        # data-driven and never depends on a step-count estimate.
        start_cmd = (
            f"START:"
            f"{1 if self.use_oor else 0},"
            f"{self.oor_lo},"
            f"{self.oor_hi},"
            f"{self.oor_arm_count}"
        )

        self.sensor.send(start_cmd)    # sensor resets buffer, seq counter
        time.sleep(0.005)
        self._push_config()          # re-confirm config on both Arduinos
        time.sleep(0.005)
        print("  [SCAN] Homing to left limit...")
        self.motor.send("X")         # motor begins homing sequence

        try:
            # Wait for scan_done or scan_aborted (set by _parse_motor in poll thread)
            while not self.scan_done and not self.scan_aborted:
                time.sleep(0.005)
        except KeyboardInterrupt:
            print("\n  [ABORT] Ctrl+C — stopping motor...")
            self.motor.send("A")     # emergency stop
            self.scan_aborted = True
            time.sleep(0.3)          # let motor stop before sending STOP to sensor

        # stop motion
        self.motor.send("S")

        # force sensor to flush partial bulk packet
        self.sensor.send("DUMP")
        time.sleep(0.5)

        # stop sensor
        self.sensor.send("STOP")

        # ── Aborted path ──────────────────────────────────────────────────────
        if self.scan_aborted:
            print(f"\n  Motor STs received : {len(self.motor_map)}")
            print(f"  Sensor readings    : {len(self.sensor_map)}")
            self._match()
            ans = input("\n  Save partial aborted scan? [Y/n]: ").strip().lower()
            if ans != "n":
                self._show_plot()
                self._save_csv()
            input("\n  Press Enter.")
            return

        # ── Normal completion path ────────────────────────────────────────────
        print(f"\n  Motor STs received : {len(self.motor_map)}")
        print(f"  Sensor readings    : {len(self.sensor_map)}")

        # Request a full buffer dump from sensor.
        # The sensor may have readings sitting in a partial bulk packet that
        # it hasn't sent yet (e.g. 7 readings in a size-10 bulk).  DUMP flushes
        # them all out in BULK_SIZE chunks and ends with "IN:dump done count=N".
        print("  Requesting sensor buffer dump...")
        self.sensor.send("DUMP")
        time.sleep(2.0)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            line = self.sensor.poll(timeout=0.05)
            if line:
                self._parse_sensor(line)
                if "dump done" in line:
                    break

        print(f"  Sensor readings after dump : {len(self.sensor_map)}")

        if not self.motor_map:
            print("  No motor data — nothing to save.")
            input("  Press Enter.")
            return

        self._match()
        self._show_plot()
        self._save_csv()
        input("\n  Press Enter to return to menu.")

    # ══════════════════════════════════════════════════════════════════════════
    # Plot
    # ══════════════════════════════════════════════════════════════════════════

    def _show_plot(self):
        """
        Display a two-panel matplotlib figure:

          Top panel    — sensor distance (mm) vs gantry position (mm)
          Bottom panel — raw sensor voltage (V) vs gantry position (mm)

        Both panels share the same X axis (gantry position).
        NaN is used for missing/alarm points so matplotlib gaps them cleanly.

        Requires: pip install matplotlib
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("  matplotlib not installed — skipping plot.")
            print("  Run: pip install matplotlib")
            return

        # Build X and Y arrays from scan_data
        steps_list = [d[1] for d in self.scan_data]
        pos_list   = [s / self._spmm for s in steps_list]   # convert to mm
        dists      = [d[3] if d[3] is not None else float("nan") for d in self.scan_data]
        volts      = [d[2] if d[2] is not None else float("nan") for d in self.scan_data]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        fig.suptitle(
            f"Full Scan v{VERSION} — {len(self.scan_data)} points  "
            f"res={self._res_mm:.2f}mm  {self.scan_rpm:.0f}RPM",
            fontsize=12,
        )

        # ── Distance panel ────────────────────────────────────────────────────
        ax1.plot(pos_list, dists, color="#185FA5", lw=0.8, label="distance")
        ax1.axhline(SENSOR_CENTER_MM, color="#888780", lw=0.8, ls="--",
                    label=f"center {SENSOR_CENTER_MM}mm")
        ax1.set_ylabel("Sensor distance (mm)")
        ax1.set_ylim(SENSOR_CENTER_MM - SENSOR_RANGE_MM - 1,
                     SENSOR_CENTER_MM + SENSOR_RANGE_MM + 1)
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=9)

        # ── Voltage panel ─────────────────────────────────────────────────────
        ax2.plot(pos_list, volts, color="#1D9E75", lw=0.8)
        ax2.axhline(5.0, color="#993C1D", lw=0.6, ls="--",
                    label="5.0V = ADC saturated (sensor > 5V)")
        ax2.set_ylabel("Voltage (V)")
        ax2.set_xlabel("Gantry position (mm from measurement origin)")
        ax2.set_ylim(0, 5.5)
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show(block=False)
        print("  [Plot shown]")

    # ══════════════════════════════════════════════════════════════════════════
    # CSV export
    # ══════════════════════════════════════════════════════════════════════════

    def _save_csv(self):
        """
        Write scan_data to a CSV file.

        Columns:
          seq                    shared sequence number (motor ↔ sensor sync key)
          step_count             cumulative motor steps relative to measurement origin
          gantry_position_mm     linear position converted from steps
          sensor_voltage_V       ADC voltage (empty if sensor reading missing)
          sensor_distance_mm     converted distance (empty if alarm or missing)
          offset_from_center_mm  sensor_distance_mm − SENSOR_CENTER_MM
          alarm                  1 if ADC was saturated (at floor 0 or ceiling 1023)
          sensor_missing         1 if no BK: packet arrived for this seq

        Notes on alarm:
          When alarm=1, voltage is stored as 5.0000 V (or 0.0000 V).  That IS
          the value the ADC measured — it cannot represent higher/lower.
          The sensor wire was likely above 5 V (no object in range) but the
          Arduino clips at Vref.  alarm=1 is the reliable indicator; the
          voltage value is kept for completeness.
        """
        ts      = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        default = f"scan_{ts}.csv"
        print(f"\n  Save CSV  (Enter = {default})")
        fname = input("  Filename: ").strip()
        if not fname:
            fname = default
        if not fname.endswith(".csv"):
            fname += ".csv"

        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "seq",
                "step_count",
                "gantry_position_mm",
                "sensor_voltage_V",
                "sensor_distance_mm",
                "offset_from_center_mm",
                "alarm",
                "sensor_missing",
            ])
            for seq, steps, v, dist in self.scan_data:
                pos_mm  = steps / self._spmm
                missing = "1" if v is None else "0"
                alarm   = "0"
                dist_s  = ""
                off_s   = ""
                volt_s  = ""

                if v is not None:
                    volt_s = f"{v:.4f}"
                    if dist is None:
                        # ADC saturated — sensor output clipped by the ADC.
                        # No distance can be computed.  alarm=1 signals this.
                        alarm = "1"
                    else:
                        off    = dist - SENSOR_CENTER_MM
                        dist_s = f"{dist:.4f}"
                        off_s  = f"{off:.4f}"

                w.writerow([
                    seq, steps, f"{pos_mm:.4f}",
                    volt_s, dist_s, off_s, alarm, missing,
                ])

        print(f"  Saved {len(self.scan_data)} rows → {os.path.abspath(fname)}")
