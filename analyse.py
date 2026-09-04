#!/usr/bin/env python3
"""
analyse.py — Post-scan CSV analysis and plotting.

Pipeline:
  1. Load scan CSV  →  rd(x)  (raw sensor deviation)
  2. Load td(x) CSV (optional; td(x)=0 if omitted)
  3. Rail correction  :  bd(x) = rd(x) + td(x)
  4. Tilt correction  :  corrected(x) = -x·sin(θ) + bd(x)·cos(θ)
                         where θ = arctan((bd[-1] - bd[0]) / length)
  5. Rolling σ band   :  ±N·σ shaded around corrected(x)
  6. Plot  +  save corrected CSV

Usage:
    python analyse.py [scan.csv]
                      [--td td_scan.csv]
                      [--window 10]
                      [--sigma 1]
                      [--no-tilt]
                      [--show-steps]
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

SENSOR_CENTER_MM = 30.0
SENSOR_RANGE_MM  = 5.0


# ─────────────────────────────────────────────────────────────────────────────
# CLI arguments
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Gantry scan analysis — rail correction, tilt removal, uncertainty band.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("scan",         nargs="?",      help="Scan CSV file path")
    p.add_argument("--td",         metavar="FILE", help="Rail deviation reference CSV (td(x)); omit to assume td(x)=0")
    p.add_argument("--window","-w",metavar="N",    type=int,   default=10,  help="Rolling window size for σ band (default: 10)")
    p.add_argument("--sigma", "-s",metavar="N",    type=float, default=1.0, help="Sigma multiplier for uncertainty band (default: 1)")
    p.add_argument("--no-tilt",    action="store_true",        help="Skip tilt correction")
    p.add_argument("--show-steps", action="store_true",        help="Overlay rd(x), bd(x), corrected(x) on same axes")
    p.add_argument("--offset",     action="store_true",        help="Shift curve so first point starts at 0 mm")
    return p.parse_args()


# ─────────────────────────────────────────────────────────────────────────────
# File picker fallback
# ─────────────────────────────────────────────────────────────────────────────

def pick_file(title="Select CSV"):
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk(); root.withdraw()
        path = filedialog.askopenfilename(
            title=title,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        root.destroy()
        return path or None
    except Exception:
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Terminal helpers
# ─────────────────────────────────────────────────────────────────────────────

def ask_yn(msg, default=True):
    label = "Y/n" if default else "y/N"
    raw = input(f"  {msg} [{label}]: ").strip().upper()
    return default if not raw else raw == "Y"

def ask_float(msg, default):
    while True:
        raw = input(f"  {msg} [{default}]: ").strip()
        if not raw: return default
        try:    return float(raw)
        except: print("  Enter a number.")

def section(title):
    print(f"\n  ── {title} {'─' * max(2, 44 - len(title))}")


# ─────────────────────────────────────────────────────────────────────────────
# CSV loading
# ─────────────────────────────────────────────────────────────────────────────

REQUIRED_COLS = {"gantry_position_mm", "sensor_distance_mm", "offset_from_center_mm"}

def load_scan_csv(path: str) -> pd.DataFrame:
    """
    Load a scan CSV, drop alarm rows and rows with missing sensor data.
    Returns DataFrame with at least: gantry_position_mm, offset_from_center_mm.
    """
    df = pd.read_csv(path)

    missing = REQUIRED_COLS - set(df.columns)
    if missing:
        print(f"\n  ERROR: CSV missing columns: {missing}")
        sys.exit(1)

    # Keep only valid (non-alarm) rows
    if "alarm" in df.columns:
        df = df[df["alarm"].astype(str).str.strip() == "0"]

    df["gantry_position_mm"]    = pd.to_numeric(df["gantry_position_mm"],    errors="coerce")
    df["sensor_distance_mm"]    = pd.to_numeric(df["sensor_distance_mm"],    errors="coerce")
    df["offset_from_center_mm"] = pd.to_numeric(df["offset_from_center_mm"], errors="coerce")
    df = df.dropna(subset=["gantry_position_mm", "offset_from_center_mm"])
    df = df.sort_values("gantry_position_mm").reset_index(drop=True)

    return df


# ─────────────────────────────────────────────────────────────────────────────
# Pipeline steps
# ─────────────────────────────────────────────────────────────────────────────

def load_td(td_path: str, scan_pos: np.ndarray) -> np.ndarray:
    """
    Load rail deviation reference CSV and interpolate onto scan positions.
    Returns td(x) array aligned with scan_pos.
    """
    df = load_scan_csv(td_path)
    td_pos = df["gantry_position_mm"].values
    td_dev = df["offset_from_center_mm"].values

    # Interpolate — fill outside range with nearest edge value
    td = np.interp(scan_pos, td_pos, td_dev,
                   left=td_dev[0], right=td_dev[-1])
    return td


def rail_correction(rd: np.ndarray, td: np.ndarray) -> np.ndarray:
    """
    bd(x) = rd(x) + td(x)
    When td=0 everywhere, bd = rd (no correction).
    """
    return rd + td


def tilt_correction(pos: np.ndarray, bd: np.ndarray):
    """
    Fit the tilt angle from the endpoints of bd(x), apply 2D rotation by -θ
    to level the data.

    θ  = arctan( (bd[-1] - bd[0]) / (pos[-1] - pos[0]) )

    Corrected deviation:
        corrected(x) = -x·sin(θ) + bd(x)·cos(θ)

    Returns (corrected, theta_rad, tilt_line)
    where tilt_line is the removed linear trend (for overlay display).
    """
    length = pos[-1] - pos[0]
    if abs(length) < 1e-9:
        return bd.copy(), 0.0, np.zeros_like(bd)

    theta = np.arctan((bd[-1] - bd[0]) / length)

    corrected  = -pos * np.sin(theta) + bd * np.cos(theta)

    # The tilt trend that was removed (for show-steps overlay)
    tilt_line  = bd[0] + (bd[-1] - bd[0]) * (pos - pos[0]) / length

    return corrected, theta, tilt_line


def rolling_sigma(data: np.ndarray, window: int) -> np.ndarray:
    """
    Rolling standard deviation with centred window.
    min_periods=1 avoids NaN at edges.
    """
    return (pd.Series(data)
              .rolling(window=window, center=True, min_periods=1)
              .std()
              .fillna(0)
              .values)


# ─────────────────────────────────────────────────────────────────────────────
# Plot
# ─────────────────────────────────────────────────────────────────────────────

def make_plot(pos, rd, bd, corrected, roll_std,
              theta_rad, tilt_line,
              sigma_mult, window, no_tilt, show_steps,
              use_pf, pf_tol,
              scan_path, td_path,
              offset_val=0.0):

    fig, ax = plt.subplots(figsize=(13, 6))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#16213e")
    for sp in ax.spines.values(): sp.set_edgecolor("#444466")
    ax.tick_params(colors="#ccccdd", labelsize=9)
    ax.xaxis.label.set_color("#ccccdd")
    ax.yaxis.label.set_color("#ccccdd")
    ax.title.set_color("#e0e0ff")
    ax.grid(True, color="#2a2a4a", linewidth=0.5, linestyle="--", zorder=0)

    # ── Step overlays (optional) ──────────────────────────────────────────────
    if show_steps:
        ax.plot(pos, rd, color="#555577", lw=0.7, zorder=1,
                label="rd(x)  raw reading")
        if td_path:
            ax.plot(pos, bd, color="#cc8800", lw=0.7, zorder=2,
                    label="bd(x)  after rail correction")
        if not no_tilt:
            ax.plot(pos, tilt_line, color="#884444", lw=0.7,
                    ls="--", zorder=2, label="tilt trend removed")

    # ── σ band ────────────────────────────────────────────────────────────────
    ax.fill_between(pos,
                    corrected - sigma_mult * roll_std,
                    corrected + sigma_mult * roll_std,
                    color="#4fc3f7", alpha=0.20, zorder=3,
                    label=f"±{sigma_mult:.0f}σ  (window={window} pts)")

    # ── Main corrected line ───────────────────────────────────────────────────
    ax.plot(pos, corrected, color="#4fc3f7", lw=0.9, zorder=4,
            label="corrected deviation")

    # ── Zero reference ────────────────────────────────────────────────────────
    ax.axhline(0.0, color="#888899", lw=0.8, ls=":", zorder=2, label="centre")

    # ── Pass / fail band ─────────────────────────────────────────────────────
    if use_pf and pf_tol is not None:
        ax.axhline( pf_tol, color="#66ff99", lw=1.1, ls="-.",
                    label=f"pass limit +{pf_tol:.2f} mm", zorder=5)
        ax.axhline(-pf_tol, color="#66ff99", lw=1.1, ls="-.",
                    label=f"pass limit −{pf_tol:.2f} mm", zorder=5)
        ax.axhspan(-pf_tol, pf_tol, color="#66ff99", alpha=0.06, zorder=1)
        ax.axhspan( pf_tol,  SENSOR_RANGE_MM + 0.5,
                   color="#ff4444", alpha=0.05, zorder=1)
        ax.axhspan(-SENSOR_RANGE_MM - 0.5, -pf_tol,
                   color="#ff4444", alpha=0.05, zorder=1)

        # Pass / fail summary
        passing = np.sum(np.abs(corrected) <= pf_tol)
        failing = len(corrected) - passing
        pct     = 100.0 * passing / max(1, len(corrected))
        result  = "PASS" if failing == 0 else "FAIL"
        col     = "#66ff99" if failing == 0 else "#ff6666"
        ax.scatter(pos[np.abs(corrected) > pf_tol],
                   corrected[np.abs(corrected) > pf_tol],
                   color="#ff4444", s=8, zorder=6, alpha=0.8,
                   label=f"FAIL pts ({failing})")
        ax.text(0.99, 0.97,
                f"{result}  {passing}/{len(corrected)} pts  ({pct:.1f}%)",
                transform=ax.transAxes, ha="right", va="top",
                fontsize=10, fontweight="bold", color=col,
                bbox=dict(boxstyle="round,pad=0.4", facecolor="#0d0d1a",
                          edgecolor=col, alpha=0.85))

    # ── Stats box ─────────────────────────────────────────────────────────────
    σ_mean = np.mean(roll_std)
    stats_lines = [
        f"n={len(corrected)}   "
        f"min={corrected.min():.3f}   max={corrected.max():.3f}   "
        f"mean={corrected.mean():.4f}   std={corrected.std():.4f} mm",
    ]
    if not no_tilt:
        stats_lines.append(
            f"tilt θ={np.degrees(theta_rad):.4f}°  "
            f"({(bd[-1]-bd[0]):.3f} mm over {pos[-1]-pos[0]:.1f} mm)"
        )
    if offset_val != 0.0:
        stats_lines.append(f"offset applied: {-offset_val:+.4f} mm")
    stats_lines.append(f"mean σ={σ_mean:.4f} mm  (window={window})")

    ax.text(0.01, 0.02, "\n".join(stats_lines),
            transform=ax.transAxes, ha="left", va="bottom",
            fontsize=8, color="#aaaacc",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="#0d0d1a",
                      edgecolor="#333355", alpha=0.8))

    # ── Labels / limits ───────────────────────────────────────────────────────
    ax.set_xlabel("Gantry position  (mm)", fontsize=10)
    ax.set_ylabel("Deviation  (mm)", fontsize=10)
    ax.set_xlim(pos[0], pos[-1])

    # Y-axis: ±sensor range, but expand if corrected data exceeds it
    y_extent = max(SENSOR_RANGE_MM + 0.5,
                   abs(corrected).max() + 0.5 * roll_std.mean() + 0.2)
    ax.set_ylim(-y_extent, y_extent)
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.xaxis.set_minor_locator(ticker.AutoMinorLocator())

    # ── Title ─────────────────────────────────────────────────────────────────
    parts = [os.path.basename(scan_path)]
    if td_path:
        parts.append(f"td={os.path.basename(td_path)}")
    if not no_tilt:
        parts.append("tilt-corrected")
    if offset_val != 0.0:
        parts.append(f"offset {-offset_val:+.3f} mm")
    parts.append(f"±{sigma_mult:.0f}σ")
    ax.set_title("  |  ".join(parts), fontsize=11, pad=10)

    legend = ax.legend(loc="upper left", fontsize=8,
                       facecolor="#0d0d1a", edgecolor="#333355",
                       labelcolor="#ccccdd", framealpha=0.85)

    fig.tight_layout()
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# Corrected CSV output
# ─────────────────────────────────────────────────────────────────────────────

def save_corrected_csv(df_orig, pos, rd, td, bd, corrected, roll_std,
                       theta_rad, sigma_mult, scan_path):
    """
    Write a new CSV alongside the original scan file with correction columns appended.
    """
    base    = os.path.splitext(scan_path)[0]
    out_path = f"{base}_corrected.csv"

    out = df_orig.copy()
    out["rd_mm"]              = rd
    out["td_mm"]              = td
    out["bd_mm"]              = bd
    out["corrected_mm"]       = corrected
    out["rolling_sigma_mm"]   = roll_std

    out.to_csv(out_path, index=False)
    print(f"  Saved corrected CSV → {os.path.abspath(out_path)}")
    return out_path


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    # ── Scan file ─────────────────────────────────────────────────────────────
    scan_path = args.scan
    if not scan_path:
        scan_path = pick_file("Select scan CSV")
    if not scan_path:
        scan_path = input("  Scan CSV path: ").strip().strip('"')
    if not os.path.exists(scan_path):
        print(f"  File not found: {scan_path}"); sys.exit(1)

    print(f"\n  Loading {os.path.basename(scan_path)}...")
    df = load_scan_csv(scan_path)
    pos = df["gantry_position_mm"].values
    rd  = df["offset_from_center_mm"].values

    print(f"  {len(df)} valid points  |  "
          f"x: {pos[0]:.1f} → {pos[-1]:.1f} mm  "
          f"({pos[-1]-pos[0]:.1f} mm span)")

    # ── Rail deviation td(x) ──────────────────────────────────────────────────
    td_path = args.td
    if td_path:
        if not os.path.exists(td_path):
            print(f"  --td file not found: {td_path}"); sys.exit(1)
        print(f"  Loading td(x) from {os.path.basename(td_path)}...")
        td = load_td(td_path, pos)
        print(f"  td(x) loaded and interpolated onto {len(td)} positions")
    else:
        print("  No td(x) file — assuming td(x) = 0 (no rail correction)")
        td = np.zeros_like(rd)

    # ── Rail correction ───────────────────────────────────────────────────────
    bd = rail_correction(rd, td)

    # ── Tilt correction ───────────────────────────────────────────────────────
    theta_rad = 0.0
    tilt_line = np.zeros_like(bd)

    if not args.no_tilt:
        corrected, theta_rad, tilt_line = tilt_correction(pos, bd)
        print(f"  Tilt θ = {np.degrees(theta_rad):.4f}°  "
              f"({bd[-1]-bd[0]:.3f} mm over {pos[-1]-pos[0]:.1f} mm)")
    else:
        corrected = bd.copy()
        print("  Tilt correction skipped (--no-tilt)")

    # ── Offset — shift first point to 0 ──────────────────────────────────────
    offset_val = 0.0
    if args.offset:
        offset_val  = corrected[0]
        rd_offset   = rd[0]
        corrected   = corrected - offset_val
        rd          = rd - rd_offset
        print(f"  Offset applied: corrected start → 0 (shifted {-offset_val:+.4f} mm)  "
              f"rd start → 0 (shifted {-rd_offset:+.4f} mm)")

    # ── Rolling σ ─────────────────────────────────────────────────────────────
    window   = max(1, args.window)
    roll_std = rolling_sigma(corrected, window)
    print(f"  Rolling σ: window={window} pts  "
          f"mean σ={np.mean(roll_std):.4f} mm  "
          f"max σ={np.max(roll_std):.4f} mm")

    # ── Pass / fail (interactive) ─────────────────────────────────────────────
    section("Pass / fail lines")
    use_pf = ask_yn("Show pass/fail lines?", default=True)
    pf_tol = None
    if use_pf:
        pf_tol = ask_float("Tolerance  ±mm", default=1.0)

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig = make_plot(
        pos, rd, bd, corrected, roll_std,
        theta_rad, tilt_line,
        args.sigma, window,
        args.no_tilt, args.show_steps,
        use_pf, pf_tol,
        scan_path, td_path,
        offset_val,
    )

    # ── Save corrected CSV ────────────────────────────────────────────────────
    section("Save")
    save_corrected_csv(df, pos, rd, td, bd, corrected, roll_std,
                       theta_rad, args.sigma, scan_path)

    if ask_yn("Save plot as PNG?", default=False):
        base    = os.path.splitext(scan_path)[0]
        out_png = f"{base}_corrected_plot.png"
        fig.savefig(out_png, dpi=180, bbox_inches="tight",
                    facecolor=fig.get_facecolor())
        print(f"  Saved → {out_png}")

    plt.show()


if __name__ == "__main__":
    main()