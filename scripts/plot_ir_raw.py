#!/usr/bin/env python3
"""
Simple IR Sensor Raw ADC vs Distance Plotter with Polynomial Overlay

Usage:
    python scripts/plot_ir_raw.py --FR scripts/logs/ir/FR.txt --R scripts/logs/ir/R.txt
    python scripts/plot_ir_raw.py --R scripts/logs/ir/R.txt --poly3 --r-gain 1.0 --r-offset 0.0
    python scripts/plot_ir_raw.py --FR scripts/logs/ir/FR.txt --R scripts/logs/ir/R.txt --r-gain 1.05 --r-offset -2.5
"""

import argparse
import os
import sys
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt

# Distinct colors & markers for each sensor
SENSOR_CONFIG = {
    'L':  {'name': 'Left (L)',        'color': '#1f77b4', 'marker': 'o'},
    'FL': {'name': 'Front-Left (FL)', 'color': '#9467bd', 'marker': '^'},
    'FR': {'name': 'Front-Right (FR)','color': '#d62728', 'marker': 's'},
    'R':  {'name': 'Right (R)',       'color': '#2ca02c', 'marker': 'D'},
}

# Baseline Polynomial Degree 3 coefficients (d = p3*x^3 + p2*x^2 + p1*x + p0, x = raw*gain/1000)
# Calibrated for monotonic range (40 mm to 250 mm)
DEFAULT_POLY3_COEFFS = {
    'R':  [-3.375607e+01, +2.229431e+02, -5.078527e+02, +4.934922e+02],
    'FR': [-2.451884e+01, +1.556967e+02, -3.501339e+02, +3.710465e+02],
}


def parse_sensor_file(file_path, sensor_key):
    """
    Parses a log file for a specific sensor.
    Supports:
      - Standard 6-column logs: Dist_mm, Sample, L, FL, FR, R
      - 2-column logs: Dist, Raw
      - 3-column logs: Dist, Sample, Raw
    """
    if not os.path.exists(file_path):
        print(f"[!] Error: File not found: {file_path}")
        return None

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    col_map = {'l': 2, 'fl': 3, 'fr': 4, 'r': 5}
    dist_data = defaultdict(list)

    # Detect header if present
    for line in lines:
        line_clean = line.strip()
        if not line_clean or line_clean.startswith('#'):
            continue

        parts = [p.strip() for p in line_clean.split(',')]
        if len(parts) < 2:
            continue

        # Check if this line is header
        lower_parts = [p.lower() for p in parts]
        if 'dist_mm' in lower_parts or 'sample' in lower_parts or 'fr' in lower_parts:
            for idx, p in enumerate(lower_parts):
                if p in col_map:
                    col_map[p] = idx
            continue

        # Parse numeric values
        try:
            dist = float(parts[0])
        except ValueError:
            continue

        target_idx = None
        if len(parts) == 2:
            target_idx = 1
        elif len(parts) == 3:
            target_idx = 2
        else:
            target_idx = col_map.get(sensor_key.lower(), 4)

        if target_idx < len(parts):
            try:
                val = float(parts[target_idx])
                dist_data[dist].append(val)
            except ValueError:
                pass

    if not dist_data:
        print(f"[!] Warning: No valid data found for sensor {sensor_key} in {file_path}")
        return None

    sorted_dists = sorted(dist_data.keys())
    means = np.array([np.mean(dist_data[d]) for d in sorted_dists])
    mins = np.array([np.min(dist_data[d]) for d in sorted_dists])
    maxs = np.array([np.max(dist_data[d]) for d in sorted_dists])

    return {
        'distances': np.array(sorted_dists),
        'means': means,
        'mins': mins,
        'maxs': maxs,
        'raw_dict': dist_data
    }


def main():
    parser = argparse.ArgumentParser(
        description="Plot Raw ADC vs Distance on a single chart with optional Poly3 model overlay."
    )
    parser.add_argument("file", nargs="?", default=None, help="Optional single file containing all sensors")
    parser.add_argument("--L", "-L", "--l", "-l", dest="L", default=None, help="Path to log file for Left sensor")
    parser.add_argument("--FL", "-FL", "--fl", "-fl", dest="FL", default=None, help="Path to log file for Front-Left sensor")
    parser.add_argument("--FR", "-FR", "--fr", "-fr", dest="FR", default=None, help="Path to log file for Front-Right sensor")
    parser.add_argument("--R", "-R", "--r", "-r", dest="R", default=None, help="Path to log file for Right sensor")

    # Poly 3 Tuning parameters
    parser.add_argument(
        "--poly3",
        action="store_true",
        help="Plot the Polynomial Degree 3 curve (d = poly3(raw * GAIN / 1000) + OFFSET)"
    )
    parser.add_argument(
        "--r-gain", "--gain",
        dest="r_gain",
        type=float,
        default=None,
        help="R_GAIN multiplier for Right sensor (default: 1.0)"
    )
    parser.add_argument(
        "--r-offset", "--offset",
        dest="r_offset",
        type=float,
        default=None,
        help="R_DISTANCE_OFFSET_MM in mm for Right sensor (default: 0.0)"
    )
    parser.add_argument(
        "--coeffs",
        type=str,
        default=None,
        help="Custom Poly3 coefficients as 'p3,p2,p1,p0' (e.g. '-33.756,222.94,-507.85,493.4922')"
    )

    # General plot options
    parser.add_argument("--save", type=str, default=None, help="Save chart image to path (e.g. plot.png)")
    parser.add_argument("--no-plot", action="store_true", help="Do not open an interactive window")
    parser.add_argument("--spread", action="store_true", default=True, help="Show min-max spread shading (default: True)")
    parser.add_argument("--no-spread", action="store_false", dest="spread", help="Disable min-max spread shading")

    # Preprocess sys.argv to allow negative values in --coeffs without triggering option parser errors
    cleaned_args = []
    i = 1
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg in ['--coeffs', '-coeffs'] and i + 1 < len(sys.argv) and not sys.argv[i+1].startswith('--'):
            cleaned_args.append(f"{arg}={sys.argv[i+1]}")
            i += 2
        else:
            cleaned_args.append(arg)
            i += 1

    args = parser.parse_args(cleaned_args)

    sensors_to_load = {}
    if args.L:  sensors_to_load['L'] = args.L
    if args.FL: sensors_to_load['FL'] = args.FL
    if args.FR: sensors_to_load['FR'] = args.FR
    if args.R:  sensors_to_load['R'] = args.R

    # If no specific flags given but positional file provided, plot all 4 from that file
    if not sensors_to_load and args.file:
        for s in ['L', 'FL', 'FR', 'R']:
            sensors_to_load[s] = args.file

    # Check if poly3 was requested directly or via gain/offset
    plot_poly3 = args.poly3 or (args.r_gain is not None) or (args.r_offset is not None) or (args.coeffs is not None)

    if not sensors_to_load and not plot_poly3:
        parser.print_help()
        print("\n[!] Error: Please specify at least one sensor file or --poly3, e.g.:")
        print("    python scripts/plot_ir_raw.py --R scripts/logs/ir/R.txt --poly3 --r-gain 1.05 --r-offset -2.0")
        sys.exit(1)

    plt.figure(figsize=(11, 6.5))

    plotted_count = 0
    loaded_data = {}
    for s_key, f_path in sensors_to_load.items():
        data = parse_sensor_file(f_path, s_key)
        if data is None:
            continue
        loaded_data[s_key] = data

        cfg = SENSOR_CONFIG[s_key]
        label = f"{cfg['name']} ({os.path.basename(f_path)})"

        # Plot main mean curve
        plt.plot(
            data['distances'],
            data['means'],
            color=cfg['color'],
            marker=cfg['marker'],
            markersize=5,
            linewidth=2,
            label=label
        )

        # Plot min-max spread shading if enabled
        if args.spread and not np.array_equal(data['mins'], data['maxs']):
            plt.fill_between(
                data['distances'],
                data['mins'],
                data['maxs'],
                color=cfg['color'],
                alpha=0.15
            )

        plotted_count += 1

    # --- Plot Poly3 Model Curve if requested ---
    if plot_poly3:
        r_gain = args.r_gain if args.r_gain is not None else 1.0
        r_offset = args.r_offset if args.r_offset is not None else 0.0

        # Determine coefficients
        if args.coeffs:
            try:
                poly_coeffs = [float(x.strip()) for x in args.coeffs.split(',')]
                if len(poly_coeffs) != 4:
                    raise ValueError("Must provide 4 coefficients: p3,p2,p1,p0")
            except Exception as e:
                print(f"[!] Error parsing --coeffs: {e}")
                sys.exit(1)
        else:
            # Default to R coefficients
            poly_coeffs = DEFAULT_POLY3_COEFFS['R']

        # Determine raw range to evaluate
        if 'R' in loaded_data:
            r_means = loaded_data['R']['means']
            raw_min = max(float(np.min(r_means)) - 50.0, 300.0)
            raw_max = float(np.max(r_means)) + 50.0
        else:
            raw_min = 400.0
            raw_max = 3200.0

        # Dense raw ADC points
        raw_grid = np.linspace(raw_min, raw_max, 400)
        x_norm = (raw_grid * r_gain) * 0.001
        dist_pred = np.polyval(poly_coeffs, x_norm) + r_offset

        # Filter points within sensible physical range (e.g. 20mm to 270mm)
        valid_mask = (dist_pred >= 20.0) & (dist_pred <= 270.0)

        poly_label = (
            f"Poly3 Model: R_GAIN={r_gain:.4f}, "
            f"R_OFFSET={r_offset:+.2f} mm"
        )

        plt.plot(
            dist_pred[valid_mask],
            raw_grid[valid_mask],
            color='#ff7f0e',
            linestyle='--',
            linewidth=2.5,
            label=poly_label,
            zorder=5
        )
        plotted_count += 1

        print(f"[*] Plotted Poly3 Model:")
        print(f"    R_GAIN               = {r_gain:.4f}")
        print(f"    R_DISTANCE_OFFSET_MM = {r_offset:+.2f} mm")
        print(f"    Coefficients [p3..p0]= {poly_coeffs}")

    if plotted_count == 0:
        print("[!] No data could be plotted.")
        sys.exit(1)

    plt.title("IR Sensors: Raw ADC vs Distance", fontsize=14, fontweight='bold', pad=12)
    plt.xlabel("Distance (mm)", fontsize=12, fontweight='bold')
    plt.ylabel("Raw ADC Reading", fontsize=12, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.minorticks_on()
    plt.grid(True, which='minor', linestyle=':', alpha=0.3)
    plt.legend(fontsize=10.5, frameon=True, loc='best')
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=200)
        print(f"[✓] Plot saved to: {args.save}")

    if not args.no_plot:
        plt.show()


if __name__ == "__main__":
    main()
