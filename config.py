# ─────────────────────────────────────────────────────────────────────────────
# config.py
#
# All hardware constants and tunable parameter defaults live here.
# Nothing else in the codebase should hard-code a physical constant or
# default value — import from this file instead.
#
# Two categories:
#   CONSTANTS  — fixed by hardware; never changed at runtime.
#   DEFAULTS   — initial values; overwritten by gantry_config.json on startup,
#                or by the user via the Configure menu.
# ─────────────────────────────────────────────────────────────────────────────

from version import VERSION  # noqa: F401  (re-exported for convenience)

# ── Communication ─────────────────────────────────────────────────────────────

MOTOR_BAUD  = 115200   # USB-serial baud rate for the motor Arduino
SENSOR_PORT = 5001     # TCP port the sensor Arduino listens on

# ── ADC / Sensor physics ──────────────────────────────────────────────────────
# The distance sensor outputs 0–5 V linearly across its measurement range.
# The Arduino R4 samples this with a 10-bit ADC referenced to 5 V.

SENSOR_CENTER_MM = 30.0   # nominal mid-range distance the sensor is mounted at
SENSOR_RANGE_MM  = 5.0    # ± range from center: sensor covers 25–35 mm
ADC_MAX          = 1023   # 10-bit ADC full-scale count
ADC_REF_V        = 5.0    # Arduino analog reference voltage (V)

# ── Stepper motor limits ──────────────────────────────────────────────────────

MAX_RPM        = 180.0   # software ceiling — don't exceed driver/motor limits
MIN_RPM        = .1     # practical lower bound for stable motion
STEP_DELAY_MIN = 104     # µs half-period → ≈4800 full steps/s at 1x microstep
STEP_DELAY_MAX = 2500    # µs half-period → very slow crawl

# ── Default motion / scan parameters ─────────────────────────────────────────
# These are the values used on first launch (before a config file is created).
# After the user runs Configure → Save, gantry_config.json stores the chosen
# values and these defaults are no longer used.

DEFAULT_PULLEY_TEETH       = 80      # GT2 pulley tooth count
DEFAULT_MICROSTEP          = 8       # stepper driver microstep divisor (1/8)
DEFAULT_FULL_STEPS         = 200     # full steps per motor revolution (1.8°)
DEFAULT_PITCH_MM           = 2.0     # GT2 belt pitch (mm per tooth)
DEFAULT_MANUAL_RPM         = 40.0    # jog speed in manual mode
DEFAULT_SCAN_RPM           = 30.0    # traverse speed during a scan pass
DEFAULT_SCAN_SAMPLE_STEPS  = 10      # motor steps between sensor TICK requests
                                     #   resolution (mm) = sample_steps / steps_per_mm
DEFAULT_BULK_SIZE          = 10      # ADC readings per TCP bulk packet (BK:)
DEFAULT_STARTUP_IGNORE_STEPS = 0  # skip first N steps after homing
                                     #   covers motor acceleration ramp

# ── Config persistence ────────────────────────────────────────────────────────

CONFIG_FILE = "gantry_config.json"   # written/read in the working directory
