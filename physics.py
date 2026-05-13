# ─────────────────────────────────────────────────────────────────────────────
# physics.py
#
# Pure unit-conversion and kinematics helpers.
# No side effects, no I/O, no imports from other gantry modules.
# All functions are independently testable.
# ─────────────────────────────────────────────────────────────────────────────

from config import (
    SENSOR_CENTER_MM, SENSOR_RANGE_MM,
    ADC_MAX, ADC_REF_V,
    MAX_RPM, STEP_DELAY_MIN, STEP_DELAY_MAX,
)


# ── ADC ↔ Voltage ↔ Distance ─────────────────────────────────────────────────

def adc_to_voltage(adc: int) -> float:
    """
    Convert a raw 10-bit ADC count (0–1023) to voltage (0.0–5.0 V).

    Formula: V = (adc / ADC_MAX) × ADC_REF_V
    """
    return (adc / ADC_MAX) * ADC_REF_V


def voltage_to_adc(voltage: float) -> int:
    """
    Convert Voltage to adc
    """
    
    return voltage * ADC_MAX / ADC_REF_V


    

def voltage_to_dist(v: float) -> float:
    """
    Map sensor voltage linearly to distance (mm).

    The sensor output spans the full 0–5 V range across the measurement window:
        0 V  →  SENSOR_CENTER_MM − SENSOR_RANGE_MM  (closest measurable point)
        5 V  →  SENSOR_CENTER_MM + SENSOR_RANGE_MM  (farthest measurable point)

    This is only called when the ADC is NOT saturated (0 < adc < ADC_MAX).
    """
    return (SENSOR_CENTER_MM - SENSOR_RANGE_MM) + (v / 5.0) * (2.0 * SENSOR_RANGE_MM)

def delta_dist_to_adc(DeltaDist: float) -> int:
    return (DeltaDist + SENSOR_RANGE_MM) * (ADC_MAX/ (SENSOR_RANGE_MM * 2))
def adc_to_delta_dist(adc: int) -> float:
    return ((adc/ADC_MAX) * SENSOR_RANGE_MM * 2) - SENSOR_RANGE_MM

def adc_to_dist(adc: int) -> tuple:
    """
    Convert a raw ADC count to (voltage_V, distance_mm | None, alarm).

    alarm=True means the ADC is at its floor (≤ 0) or ceiling (≥ ADC_MAX).
    At ceiling, the sensor wire is above 5 V — object too far away or absent.
    At floor,   the sensor wire is at 0 V  — object too close or shorted.

    WHY we check `adc >= ADC_MAX` instead of `v >= 5.15`:
        The Arduino ADC clips at exactly 1023 when the sensor exceeds 5 V.
        After converting: 1023/1023 × 5.0 = 5.0000 V exactly.
        A threshold of 5.15 V never fires because the ADC cannot represent
        anything above 5.0 V.  Checking the raw count BEFORE conversion is
        the correct approach (fixed from v5).

    Returns:
        v      — voltage (always valid; 5.0 V when saturated — that IS what the ADC read)
        dist   — distance in mm, or None when alarm is True
        alarm  — True when reading is unreliable (ADC saturated)
    """
    v = adc_to_voltage(adc)
    if adc >= ADC_MAX or adc <= 0:
        return v, None, True       # saturated — sensor output out of ADC range
    dist = voltage_to_dist(v)
    return v, dist, False


# ── Stepper kinematics ────────────────────────────────────────────────────────

def calc_spr(microstep: int, full_steps: int = 200) -> int:
    """
    Steps per revolution.
    Formula: full_steps_per_rev × microstep_divisor
    e.g. 200 × 8 = 1600 steps/rev for 8× microstepping.
    """
    return full_steps * microstep


def calc_spmm(microstep: int, pulley_teeth: int,
              full_steps: int = 200, pitch: float = 2.0) -> float:
    """
    Steps per millimetre of linear belt travel.
    Formula: SPR / (pulley_teeth × pitch_mm)

    Example: 1600 / (80 × 2.0) = 10 steps/mm
    """
    return calc_spr(microstep, full_steps) / (pulley_teeth * pitch)


def rpm_to_delay(rpm: float, spr: int) -> int:
    """
    Convert a target RPM to stepper half-period delay in microseconds.

    The stepper driver generates one step pulse per HIGH→LOW toggle, so
    one full step takes 2 × delay µs.  The Arduino fires HIGH and LOW
    each separated by `stepDelay` µs.

    Formula: delay = 1_000_000 / (2 × steps_per_second)
             where steps_per_second = (rpm / 60) × spr

    Result is clamped to [STEP_DELAY_MIN, STEP_DELAY_MAX].
    """
    if rpm <= 0:
        return STEP_DELAY_MAX
    step_freq = (rpm / 60.0) * spr          # steps per second
    delay = int(1_000_000 / (2 * step_freq))
    return max(STEP_DELAY_MIN, min(STEP_DELAY_MAX, delay))


def delay_to_rpm(delay_us: int, spr: int) -> float:
    """
    Inverse of rpm_to_delay — reconstruct actual RPM from the stored delay.
    Used for display only (the Arduino may round the delay slightly).
    """
    return (1_000_000 / (2 * delay_us) / spr) * 60.0


def hw_max_rpm(microstep: int) -> float:
    """
    Practical hardware RPM ceiling for a given microstep divisor.
    Based on the minimum achievable step delay (STEP_DELAY_MIN), capped
    at the global MAX_RPM safety limit.
    """
    spr = calc_spr(microstep)
    # Derive RPM from the minimum step delay
    max_step_freq = 1_000_000 / (2 * STEP_DELAY_MIN)   # steps/second
    hw_rpm = (max_step_freq / spr) * 60.0
    return min(hw_rpm, MAX_RPM)


def resolution_mm(scan_sample_steps: int, microstep: int, pulley_teeth: int,
                  full_steps: int = 200, pitch: float = 2.0) -> float:
    """
    Spatial resolution: linear distance between consecutive sensor samples (mm/point).

    Formula: scan_sample_steps / steps_per_mm

    Lower is finer.  Example: 10 steps / 10 steps_per_mm = 1.0 mm per point.
    """
    spmm = calc_spmm(microstep, pulley_teeth, full_steps, pitch)
    return scan_sample_steps / spmm
