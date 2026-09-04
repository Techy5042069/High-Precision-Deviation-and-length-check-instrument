#!/usr/bin/env python3
"""
Gantry beam straightness — uncertainty calculator (rail bow eliminated).

Takes N repeat-scan CSV files (same beam, N passes), each with columns:
    seq, step_count, gantry_position_mm, sensor_voltage_V,
    sensor_distance_mm, offset_from_center_mm, alarm, sensor_missing

Computes, per point (step_count):
    mean reading, sigma_rep, sigma_rep/sqrt(N), sigma_bd_raw,
    tilt-corrected profile, sigma_tilt, sigma_final, U(x)

Formulas (no rail bow — td(x) = 0, no interpolation term — reported only at measured points):
    bd(x)          = mean of N readings at x
    sigma_rep(x)   = std across N readings at x            (ddof=1, needs N>=2)
    sigma_rep_mean = sigma_rep(x) / sqrt(n_valid)           <- averaging N reduces this
    sigma_adc      = (R_mm / (2^bits - 1)) / sqrt(12)        constant, NOT reduced by N
    sigma_nonlin   = (P/100 * R_mm) / sqrt(3)                constant, NOT reduced by N
    sigma_bd_raw   = sqrt(sigma_adc^2 + sigma_nonlin^2 + sigma_rep_mean^2)

    theta          = arctan( (bd(L) - bd(0)) / L )
    corrected(x)   = -x'*sin(theta) + bd(x)*cos(theta)       x' = x - x[0]
    sigma_theta    = sqrt(sigma_bd_raw(0)^2 + sigma_bd_raw(L)^2) / L
    sigma_tilt(x)  = x' * sigma_theta

    sigma_final(x) = sqrt(sigma_bd_raw^2 + sigma_tilt^2)
    U(x)           = k * sigma_final(x)          (k=2 -> 95% confidence, GUM)

Usage:
    # one-time setup: generate a config template, then edit it
    python3 gantry_uncertainty.py --init-config
    #   -> edit gantry_config.json: bits, range_mm, nonlin_pct, k, input_dir, output, plot_output

    # every run after that (reads all *.csv from input_dir in the config)
    python3 gantry_uncertainty.py

    # pass the folder directly as an argument (overrides config's input_dir)
    python3 gantry_uncertainty.py ./scans2

    # override other settings on the command line if needed
    python3 gantry_uncertainty.py ./scans --k 3
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_CONFIG_PATH = "gantry_uncertainty_config.json"

CONFIG_TEMPLATE = {
    "bits": 14,
    "range_mm": 10.0,
    "nonlin_pct": 0.5,
    "k": 2.0,
    "input_dir": "./scans",
    "output": "beam_uncertainty.csv",
    "plot_output": "beam_plot.png",
}


def load_pass(path: str, pass_index: int) -> pd.DataFrame:
    """Load one repeat-scan CSV, drop rows flagged sensor_missing."""
    df = pd.read_csv(path)

    required = {"step_count", "gantry_position_mm", "offset_from_center_mm"}
    missing_cols = required - set(df.columns)
    if missing_cols:
        sys.exit(f"{path}: missing required column(s): {sorted(missing_cols)}")

    if "sensor_missing" in df.columns:
        missing_mask = df["sensor_missing"].astype(str).str.strip().str.lower().isin(
            {"1", "true", "yes"}
        )
    else:
        missing_mask = pd.Series(False, index=df.index)

    alarm_mask = pd.Series(False, index=df.index)
    if "alarm" in df.columns:
        alarm_mask = df["alarm"].astype(str).str.strip().str.lower().isin(
            {"1", "true", "yes"}
        )

    out = df.loc[~missing_mask, ["step_count", "gantry_position_mm", "offset_from_center_mm"]].copy()
    out = out.rename(columns={"offset_from_center_mm": f"reading_{pass_index}"})
    out[f"alarm_{pass_index}"] = alarm_mask.loc[out.index].values
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input_dir", nargs="?", default=None,
                     help="Folder containing scan CSVs (overrides input_dir in config)")
    ap.add_argument("--config", default=DEFAULT_CONFIG_PATH,
                     help=f"Path to JSON config file (default: {DEFAULT_CONFIG_PATH})")
    ap.add_argument("--init-config", action="store_true",
                     help="Write a template config file to --config path and exit")
    ap.add_argument("--bits", type=int, default=None, help="ADC bit depth (e.g. 14)")
    ap.add_argument("--range-mm", type=float, default=None, dest="range_mm",
                     help="Sensor full-scale range R_mm (mm)")
    ap.add_argument("--nonlin-pct", type=float, default=None, dest="nonlin_pct",
                     help="Sensor nonlinearity spec, %% of full range (e.g. 0.5)")
    ap.add_argument("--k", type=float, default=None, help="Coverage factor (default 2 -> 95%% CI)")
    ap.add_argument("--output", default=None, help="Output CSV path")
    ap.add_argument("--plot-output", default=None, dest="plot_output",
                     help="If given, also save a PNG plot of position vs corrected profile ±U")
    args = ap.parse_args()

    if args.init_config:
        Path(args.config).write_text(json.dumps(CONFIG_TEMPLATE, indent=2) + "\n")
        print(f"Wrote template config to {args.config} — edit the values, then rerun.")
        return

    # --- load config file, if present ---
    config_path = Path(args.config)
    config = {}
    if config_path.exists():
        config = json.loads(config_path.read_text())
    elif args.config != DEFAULT_CONFIG_PATH:
        sys.exit(f"Config file not found: {args.config}")

    def resolve(name, builtin_default=None):
        cli_val = getattr(args, name)
        if cli_val is not None:
            return cli_val
        if name in config:
            return config[name]
        return builtin_default

    bits = resolve("bits")
    range_mm = resolve("range_mm")
    nonlin_pct = resolve("nonlin_pct")
    k = resolve("k", 2.0)
    output = resolve("output", "beam_uncertainty.csv")
    plot_output = resolve("plot_output")
    input_dir = resolve("input_dir")

    missing = [n for n, v in [("bits", bits), ("range_mm", range_mm), ("nonlin_pct", nonlin_pct)] if v is None]
    if missing:
        sys.exit(f"Missing required parameter(s): {missing}. Set them in {args.config} "
                  f"(run with --init-config to generate a template) or pass as CLI flags.")

    # --- resolve input file list (only .csv files, case-insensitive) ---
    if not input_dir:
        sys.exit("No input directory: pass one as an argument, or set input_dir in the config.")
    input_dir_path = Path(input_dir)
    output_path = input_dir_path / Path(output).name
    plot_path = (input_dir_path / Path(plot_output).name) if plot_output else None

    file_list = sorted(
        str(p) for p in input_dir_path.iterdir()
        if p.is_file() and p.suffix.lower() == ".csv" and p.name != output_path.name
    )
    if not file_list:
        sys.exit(f"No .csv files found in input_dir: {input_dir}")

    n = len(file_list)
    if n < 2:
        sys.exit("Need at least 2 repeat-scan files to estimate sigma_rep. "
                  "For a single scan, sigma_rep must be supplied externally (see chat).")

    # --- load and merge all passes on step_count ---
    passes = [load_pass(f, i + 1) for i, f in enumerate(file_list)]
    merged = passes[0][["step_count", "gantry_position_mm"]].copy()
    for i, p in enumerate(passes):
        merged = merged.merge(
            p[["step_count", f"reading_{i+1}", f"alarm_{i+1}"]],
            on="step_count", how="outer"
        )
    merged = merged.sort_values("step_count").reset_index(drop=True)

    reading_cols = [f"reading_{i+1}" for i in range(n)]
    alarm_cols = [f"alarm_{i+1}" for i in range(n)]

    readings = merged[reading_cols].to_numpy(dtype=float)
    n_valid = np.sum(~np.isnan(readings), axis=1)
    if np.any(n_valid < 2):
        bad = merged.loc[n_valid < 2, "step_count"].tolist()
        print(f"Warning: {len(bad)} point(s) have <2 valid readings "
              f"(sigma_rep undefined there): step_count {bad[:10]}{'...' if len(bad) > 10 else ''}",
              file=sys.stderr)

    mean_reading = np.nanmean(readings, axis=1)
    sigma_rep = np.nanstd(readings, axis=1, ddof=1)          # NaN where n_valid < 2
    sigma_rep_mean = sigma_rep / np.sqrt(np.maximum(n_valid, 1))

    x = merged["gantry_position_mm"].to_numpy(dtype=float)
    x0 = x - x[0]
    L = x[-1] - x[0]

    # --- fixed instrument terms ---
    adc_max = (2 ** bits) - 1
    sigma_adc = (range_mm / adc_max) / np.sqrt(12)
    sigma_nonlin = (nonlin_pct / 100.0 * range_mm) / np.sqrt(3)

    sigma_bd_raw = np.sqrt(sigma_adc ** 2 + sigma_nonlin ** 2 + sigma_rep_mean ** 2)

    # --- tilt correction ---
    bd0, bdL = mean_reading[0], mean_reading[-1]
    theta = np.arctan2(bdL - bd0, L)
    corrected = -x0 * np.sin(theta) + mean_reading * np.cos(theta)

    # shift so the profile starts at 0: deviation is measured from the beam
    # surface at x=0, not from the laser sensor's zero/center position
    corrected = corrected - corrected[0]

    sigma_theta = np.sqrt(sigma_bd_raw[0] ** 2 + sigma_bd_raw[-1] ** 2) / L
    sigma_tilt = x0 * sigma_theta

    # --- combine ---
    sigma_final = np.sqrt(sigma_bd_raw ** 2 + sigma_tilt ** 2)
    U = k * sigma_final

    any_alarm = merged[alarm_cols].fillna(False).to_numpy(dtype=bool).any(axis=1)

    out = pd.DataFrame({
        "step_count": merged["step_count"],
        "x_mm": x,
        **{c: merged[c] for c in reading_cols},
        "n_valid": n_valid,
        "mean_reading_mm": mean_reading,
        "sigma_rep_mm": sigma_rep,
        "sigma_rep_mean_mm": sigma_rep_mean,
        "sigma_adc_mm": sigma_adc,
        "sigma_nonlin_mm": sigma_nonlin,
        "sigma_bd_raw_mm": sigma_bd_raw,
        "theta_rad": theta,
        "corrected_mm": corrected,
        "sigma_tilt_mm": sigma_tilt,
        "sigma_final_mm": sigma_final,
        "U_mm": U,
        "corrected_minus_U_mm": corrected - U,
        "corrected_plus_U_mm": corrected + U,
        "any_alarm": any_alarm,
    })

    formula_header = [
        f"# Gantry beam uncertainty output. Run parameters: bits={bits}, range_mm={range_mm}, "
        f"nonlin_pct={nonlin_pct}, k={k}, passes N={n}, input_dir={input_dir}",
        "# Rail bow eliminated (td(x)=0). No interpolation term. corrected(x) shifted so corrected[0]=0",
        "# (deviation measured from the beam surface at x=0, not the laser sensor's zero/center).",
        "#",
        "# Column: formula",
        "# step_count: point index along the scan (from gantry.py)",
        "# x_mm: gantry position along the beam, x (mm)",
        "# reading_i: raw sensor reading (offset_from_center_mm) for repeat pass i",
        "# n_valid: number of non-missing readings across the N passes at this point",
        "# mean_reading_mm: bd(x) = mean of valid readings across passes",
        "# sigma_rep_mm: sigma_rep(x) = std dev across passes at this point (ddof=1)",
        "# sigma_rep_mean_mm: sigma_rep(x) / sqrt(n_valid)  [uncertainty of the mean reading]",
        "# sigma_adc_mm: (range_mm / (2^bits - 1)) / sqrt(12)  [ADC quantization, constant]",
        "# sigma_nonlin_mm: (nonlin_pct/100 * range_mm) / sqrt(3)  [sensor nonlinearity, constant]",
        "# sigma_bd_raw_mm: sqrt(sigma_adc^2 + sigma_nonlin^2 + sigma_rep_mean^2)",
        "# theta_rad: tilt angle = arctan2(mean_reading[L] - mean_reading[0], L); same value every row",
        "# corrected_mm: -x'*sin(theta) + mean_reading*cos(theta), then shifted so corrected[0]=0; x' = x - x[0]",
        "# sigma_tilt_mm: x' * sigma_theta, where sigma_theta = sqrt(sigma_bd_raw[0]^2 + sigma_bd_raw[L]^2) / L",
        "# sigma_final_mm: sqrt(sigma_bd_raw^2 + sigma_tilt^2)",
        f"# U_mm: k * sigma_final(x), k={k} (k=2 -> ~95% confidence, GUM convention)",
        "# corrected_minus_U_mm / corrected_plus_U_mm: corrected(x) -+ U(x), the reported uncertainty band",
        "# any_alarm: True if any pass flagged an out-of-range alarm at this point",
        "#",
    ]

    with open(output_path, "w", newline="") as f:
        f.write("\n".join(formula_header) + "\n")
    out.to_csv(output_path, mode="a", index=False)
    print(f"Wrote {len(out)} points ({n} passes) to {output_path}")
    print(f"theta = {theta:.6f} rad ({np.degrees(theta):.4f} deg)")
    print(f"sigma_adc = {sigma_adc:.6f} mm, sigma_nonlin = {sigma_nonlin:.6f} mm")
    print(f"mean U(x) = {U.mean():.4f} mm, max U(x) = {U.max():.4f} mm")

    if plot_path:
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.fill_between(x, corrected - U, corrected + U,
                         color="tab:blue", alpha=0.2, label=f"\u00b1U(x), k={k:g}")
        ax.plot(x, corrected, color="tab:blue", linewidth=1.5, label="corrected(x)")
        ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
        ax.set_xlabel("Gantry position, x (mm)")
        ax.set_ylabel("Beam deviation (mm)")
        ax.set_ylim(-5, 5)
        ax.set_title("Beam straightness profile with expanded uncertainty")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(plot_path, dpi=150)
        print(f"Wrote plot to {plot_path}")


if __name__ == "__main__":
    main()