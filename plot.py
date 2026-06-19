#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# plot_csv.py
#
# Reproduce the Gantry Master deviation-from-center plot from a saved scan
# CSV (the file written by GantryMaster._save_csv).
#
# Usage:
#   python plot_csv.py scan_20260617_120000.csv
#   python plot_csv.py scan.csv --oor-lo -1.5 --oor-hi 2.0
#   python plot_csv.py scan.csv --oor-lo -1.5 --oor-hi 2.0 --out myplot.png
#
# The CSV does not store the OOR window bounds (those live in
# gantry_config.json / the live session, not in the export), so pass them
# explicitly with --oor-lo / --oor-hi if you want the green band drawn.
# Both flags are required together; omit both to plot without the band.
# ─────────────────────────────────────────────────────────────────────────────

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt

SENSOR_RANGE_MM = 5.0   # ± range from center — matches config.SENSOR_RANGE_MM


def load_csv(path: Path):
    """
    Read the scan CSV and return (positions_mm, deviations_mm).

    Expected columns (written by GantryMaster._save_csv):
      gantry_position_mm, offset_from_center_mm, alarm, sensor_missing

    offset_from_center_mm is blank whenever alarm=1 or sensor_missing=1 —
    those rows become NaN so matplotlib gaps them cleanly, same as the
    live plot.
    """
    positions = []
    deviations = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            positions.append(float(row["gantry_position_mm"]))
            off = row.get("offset_from_center_mm", "").strip()
            deviations.append(float(off) if off else math.nan)

    return positions, deviations


def main():
    ap = argparse.ArgumentParser(
        description="Replay the deviation-from-center scan plot from a CSV export."
    )
    ap.add_argument("csv_path", type=Path, help="Path to the scan CSV file")
    ap.add_argument("--oor-lo", type=float, default=None,
                     help="OOR window lower bound, mm offset from center")
    ap.add_argument("--oor-hi", type=float, default=None,
                     help="OOR window upper bound, mm offset from center")
    ap.add_argument("--out", type=Path, default=None,
                     help="Output PNG path (default: <csv name>_plot.png next to the CSV)")
    args = ap.parse_args()

    if (args.oor_lo is None) != (args.oor_hi is None):
        ap.error("--oor-lo and --oor-hi must be given together.")

    positions, deviations = load_csv(args.csv_path)

    out_path = args.out or args.csv_path.with_name(args.csv_path.stem + "_plot.png")

    fig, ax = plt.subplots(figsize=(13, 6))
    fig.suptitle(f"Scan Replay — {args.csv_path.name} — {len(positions)} points",
                 fontsize=12)

    if args.oor_lo is not None:
        ax.axhspan(args.oor_lo, args.oor_hi, color="green", alpha=0.12,
                   label=f"OOR window [{args.oor_lo:.2f}, {args.oor_hi:.2f}]mm")
        ax.axhline(args.oor_lo, color="green", lw=0.8, ls="--")
        ax.axhline(args.oor_hi, color="green", lw=0.8, ls="--")

    ax.plot(positions, deviations, color="#185FA5", lw=0.8, label="deviation")
    ax.axhline(0.0, color="#888780", lw=0.8, ls="--", label="center (0mm)")

    ax.set_ylabel("Deviation from center (mm)")
    ax.set_xlabel("Gantry position (mm from measurement origin)")
    ax.set_ylim(-SENSOR_RANGE_MM, SENSOR_RANGE_MM)
    ax.set_xlim(left=0)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    plt.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f"  Saved plot → {out_path.resolve()}")
    plt.show()


if __name__ == "__main__":
    main()
