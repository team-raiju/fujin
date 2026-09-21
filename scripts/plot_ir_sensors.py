#!/usr/bin/env python3
"""
IR Sensor Log Plotter

Plots IR sensor data (L, FL, FR, R) from robot log files.
Supports single log visualization, dual log comparison, and plotting against time or distance.

Usage:
    python scripts/plot_ir_sensors.py logs/2026-09-16/log_00-05-03.txt
    python scripts/plot_ir_sensors.py log1.txt log2.txt
    python scripts/plot_ir_sensors.py log.txt --grid
    python scripts/plot_ir_sensors.py log.txt --combined
    python scripts/plot_ir_sensors.py log.txt --vs-dist
"""

import argparse
import os
import sys
import matplotlib.pyplot as plt
import numpy as np

# --- Wall Control & Detection Constants ---
# Default reference distances (mm) when robot is centered in the maze cell
IR_WALL_DIST_REF_LEFT = 180.0   # Left reference distance (mm)
IR_WALL_DIST_REF_RIGHT = 165.0  # Right reference distance (mm)

# Thresholds (mm) below which wall control is valid (must be greater than reference distance)
IR_WALL_CONTROL_TH_LEFT = 225.0  # Left wall control threshold (mm)
IR_WALL_CONTROL_TH_RIGHT = 225.0 # Right wall control threshold (mm)
IR_SLOPE_THRESHOLD = 40.0        # Wall control is valid when abs(slope) is below this value
IR_MAX_DISTANCE_MM = 270.0       # Maximum sensor distance (mm)

HEADER_KEY_MAP = {
    't': 'time', 'time': 'time', 'time(ms)': 'time',
    'vel': 'lin_vel_act', 'actuallinearvel': 'lin_vel_act', 'lin_vel_act': 'lin_vel_act',
    'tgtvel': 'lin_vel_tgt', 'targetlinearvel': 'lin_vel_tgt', 'lin_vel_tgt': 'lin_vel_tgt',
    'angvel': 'ang_vel_act', 'actualangularvel': 'ang_vel_act', 'ang_vel_act': 'ang_vel_act',
    'tgtangvel': 'ang_vel_tgt', 'targetangularvel': 'ang_vel_tgt', 'ang_vel_tgt': 'ang_vel_tgt',
    'pwm_l': 'pwm_left', 'pwml': 'pwm_left', 'pwm_left': 'pwm_left',
    'pwm_r': 'pwm_right', 'pwmr': 'pwm_right', 'pwm_right': 'pwm_right',
    'imudiff': 'imu_diff', 'imu_diff': 'imu_diff',
    'posx': 'pos_x', 'posx(m)': 'pos_x', 'pos_x': 'pos_x',
    'posy': 'pos_y', 'posy(m)': 'pos_y', 'pos_y': 'pos_y',
    'angle': 'angle', 'angle(rad)': 'angle', 'dist': 'dist',
    'sensl': 'sens_l', 'sensl(mm)': 'sens_l', 'sens_l': 'sens_l',
    'sensfl': 'sens_fl', 'sensfl(mm)': 'sens_fl', 'sens_fl': 'sens_fl',
    'sensfr': 'sens_fr', 'sensfr(mm)': 'sens_fr', 'sens_fr': 'sens_fr',
    'sensr': 'sens_r', 'sensr(mm)': 'sens_r', 'sens_r': 'sens_r',
    'velp': 'vel_p', 'vel_p': 'vel_p', 'veli': 'vel_i', 'vel_i': 'vel_i',
    'angp': 'ang_p', 'ang_p': 'ang_p', 'angi': 'ang_i', 'ang_i': 'ang_i',
    'rotff': 'rotation_ff', 'rotationff': 'rotation_ff', 'rotation_ff': 'rotation_ff',
    'linff': 'linear_ff', 'linearff': 'linear_ff', 'linear_ff': 'linear_ff',
    'batt_mv': 'battery', 'battery(mv)': 'battery', 'battery': 'battery', 'batt': 'battery',
    'rawvell': 'raw_vel_l', 'rawvelr': 'raw_vel_r', 'rawangvel': 'raw_ang_vel'
}

def parse_header_line(header_line, expected_count):
    """Matches semicolon-separated column headers to internal keys."""
    parts = header_line.split(';')
    keys = []
    for p in parts:
        clean = p.split('(')[0].strip().lower()
        if clean in HEADER_KEY_MAP:
            keys.append(HEADER_KEY_MAP[clean])
        else:
            clean_nopunct = ''.join(c for c in p.strip().lower() if c.isalnum() or c == '_')
            if clean_nopunct in HEADER_KEY_MAP:
                keys.append(HEADER_KEY_MAP[clean_nopunct])
            else:
                return None
    if len(keys) == expected_count:
        return keys
    return None

def parse_log_file(file_path):
    """Reads a log file and returns a structured dictionary of data arrays."""
    if not os.path.exists(file_path):
        alt_path = os.path.join(os.path.dirname(__file__), file_path)
        if os.path.exists(alt_path):
            file_path = alt_path
        else:
            print(f"Error: Log file '{file_path}' not found.")
            return None

    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    if len(lines) <= 1:
        print(f"Log file '{file_path}' is empty or missing data rows.")
        return None

    header_lines = []
    first_data_line = ""
    in_param = False
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        if stripped.startswith("general_params = {"):
            in_param = True
            continue
        if in_param:
            if stripped.startswith("};") or stripped == "}":
                in_param = False
            continue
        try:
            [float(x) for x in stripped.split(';')]
            first_data_line = stripped
            break
        except ValueError:
            header_lines.append(stripped)

    if not first_data_line:
        print(f"No valid numerical data rows found in '{file_path}'.")
        return None

    cols_count = len(first_data_line.split(';'))
    keys = None
    for h in reversed(header_lines):
        matched = parse_header_line(h, cols_count)
        if matched:
            keys = matched
            break

    if keys is None:
        if cols_count == 16:
            keys = [
                'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                'pwm_left', 'pwm_right', 'imu_diff', 'pos_x', 'pos_y', 'angle', 'dist',
                'sens_l', 'sens_fl', 'sens_fr', 'sens_r'
            ]
        elif cols_count == 15:
            keys = [
                'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i', 'rotation_ff', 'linear_ff', 'battery'
            ]
        elif cols_count == 14:
            keys = [
                'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i', 'rotation_ff', 'linear_ff'
            ]
        elif cols_count == 13:
            keys = [
                'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                'pwm_left', 'pwm_right', 'imu_diff', 'battery', 'pos_x', 'pos_y', 'angle', 'dist'
            ]
        else:
            print(f"Unknown column count ({cols_count}) in '{file_path}'.")
            return None

    all_keys = list(dict.fromkeys(keys + ['sens_l', 'sens_fl', 'sens_fr', 'sens_r', 'dist', 'time', 'lin_vel_act']))
    data_dict = {k: [] for k in all_keys}

    in_param_block = False
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith("general_params = {"):
            in_param_block = True
            continue
        if in_param_block:
            if line.startswith("};") or line == "}":
                in_param_block = False
            continue
        try:
            fields = [float(x) for x in line.split(';')]
            for idx, field_val in enumerate(fields):
                if idx < len(keys):
                    data_dict[keys[idx]].append(field_val)
        except ValueError:
            continue

    if not data_dict['time']:
        print(f"No valid numerical data could be parsed from '{file_path}'.")
        return None

    return data_dict

# SENSORS METADATA
SENSOR_META = {
    'sens_l':  {'label': 'Left (L)',        'color': '#1f77b4', 'linestyle': '-'},
    'sens_fl': {'label': 'Front-Left (FL)', 'color': '#9467bd', 'linestyle': '-'},
    'sens_fr': {'label': 'Front-Right (FR)','color': '#d62728', 'linestyle': '-'},
    'sens_r':  {'label': 'Right (R)',       'color': '#2ca02c', 'linestyle': '-'},
}

def resolve_file_path(path):
    """Resolves file path relative to cwd or script directory."""
    if os.path.exists(path):
        return path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    rel_path = os.path.join(script_dir, path)
    if os.path.exists(rel_path):
        return rel_path
    return path

def compute_cumulative_distance(time_ms, lin_vel=None, fallback_dist=None):
    """
    Computes cumulative traveled distance in millimeters by integrating linear velocity over time.
    Falls back to integrating incremental travel distance if linear velocity is absent.
    """
    t_arr = np.array(time_ms, dtype=float)
    if len(t_arr) == 0:
        return np.array([])

    if lin_vel is not None and len(lin_vel) == len(t_arr):
        v = np.array(lin_vel, dtype=float)  # m/s
        t_sec = t_arr / 1000.0              # seconds
        dt = np.diff(t_sec, prepend=t_sec[0])
        dt = np.maximum(dt, 0.0)
        # v in m/s * dt in s = meters * 1000 = millimeters
        cum_dist_mm = np.cumsum(np.abs(v) * dt) * 1000.0
        if cum_dist_mm[-1] > 0.1:
            return cum_dist_mm

    # Fallback: integrate distance increments from dist column if available
    if fallback_dist is not None and len(fallback_dist) == len(t_arr):
        d = np.array(fallback_dist, dtype=float)
        diffs = np.diff(d, prepend=d[0])
        # If dist was recorded in meters (typically max < 50), scale to mm
        scale = 1000.0 if np.nanmax(np.abs(d)) < 50.0 else 1.0
        increments = np.where(diffs > 0, diffs * scale, 0.0)
        cum_dist_mm = np.cumsum(increments)
        if cum_dist_mm[-1] > 0.1:
            return cum_dist_mm

    return np.zeros(len(t_arr))

def compute_sensor_slope(values):
    """
        Computes the 4-point weighted slope used by the firmware:
            (Sensor(0) - Sensor(-3)) * 4 + (Sensor(-1) - Sensor(-2)) * 1
    """
    s = np.array(values, dtype=float)
    if len(s) == 0:
                return np.array([])

    # Lagged readings for slope filter: S(0), S(-1), S(-2), S(-3)
    s_m1 = np.roll(s, 1)
    s_m1[0] = s[0]
    s_m2 = np.roll(s, 2)
    s_m2[:2] = s[0]
    s_m3 = np.roll(s, 3)
    s_m3[:3] = s[0]

    # Slope = (Sensor(0) - Sensor(-3) * 4) + (Sensor(-1) - Sensor(-2) * 1)
    slope = 4.0 * (s - s_m3) + 1.0 * (s_m1 - s_m2)

    return slope

def compute_control_validity(values, control_threshold):
    """Returns the firmware-matching control-valid mask and its weighted slope."""
def compute_control_validity(values, control_threshold, dist=None, hysteresis_dist=20.0):
    """Returns the firmware-matching control-valid mask and its weighted slope with distance hysteresis."""
    distances = np.array(values, dtype=float)
    slope = compute_sensor_slope(distances)
    valid = (distances < control_threshold) & (np.abs(slope) < IR_SLOPE_THRESHOLD)
    raw_valid = (distances < control_threshold) & (np.abs(slope) < IR_SLOPE_THRESHOLD)
    break_cond = (distances > control_threshold) | (slope > IR_SLOPE_THRESHOLD)

    if dist is None or len(dist) != len(distances):
        return raw_valid, slope

    valid = np.zeros_like(raw_valid, dtype=bool)
    confirmed = False
    acc_dist = 0.0
    for i in range(len(distances)):
        delta_d = (dist[i] - dist[i-1]) if i > 0 else 0.0
        if delta_d < 0:
            delta_d = 0.0

        if break_cond[i]:
            confirmed = False
            acc_dist = 0.0
        elif raw_valid[i]:
            if not confirmed:
                acc_dist += delta_d
                if acc_dist >= hysteresis_dist:
                    confirmed = True
        else:
            confirmed = False
            acc_dist = 0.0

        valid[i] = confirmed

    return valid, slope

def shade_invalid_regions(ax, x_axis, valid, color):
    """Shade contiguous regions where wall control is invalid."""
    if len(x_axis) == 0 or len(valid) != len(x_axis):
        return

    invalid = ~valid
    start = None
    for index, is_invalid in enumerate(invalid):
        if is_invalid and start is None:
            start = index
        elif not is_invalid and start is not None:
            ax.axvspan(x_axis[start], x_axis[index - 1], color=color, alpha=0.22, lw=0)
            start = None
    if start is not None:
        ax.axvspan(x_axis[start], x_axis[-1], color=color, alpha=0.22, lw=0)

def ir_side_wall_error(left_dist, right_dist,
                       ref_left=IR_WALL_DIST_REF_LEFT,
                       ref_right=IR_WALL_DIST_REF_RIGHT,
                       th_left=IR_WALL_CONTROL_TH_LEFT,
                       th_right=IR_WALL_CONTROL_TH_RIGHT,
                       dist=None,
                       hysteresis_dist=20.0):
    """
    Computes side wall error matching firmware ir_side_wall_error():
        int32_t left_error = ir_distances[LEFT] - ir_wall_dist_ref_left;
        int32_t right_error = ir_distances[RIGHT] - ir_wall_dist_ref_right;

        if (ir_wall_control_valid(LEFT) && ir_wall_control_valid(RIGHT)) {
            ir_error = right_error - left_error;
        } else if (ir_wall_control_valid(LEFT)) {
            ir_error = -2.0 * left_error;
        } else if (ir_wall_control_valid(RIGHT)) {
            ir_error = 2.0 * right_error;
        } else {
            ir_error = 0;
        }
    """
    l_arr = np.array(left_dist, dtype=float)
    r_arr = np.array(right_dist, dtype=float)

    left_error = l_arr - ref_left
    right_error = r_arr - ref_right

    valid_left, _ = compute_control_validity(l_arr, th_left, dist=dist, hysteresis_dist=hysteresis_dist)
    valid_right, _ = compute_control_validity(r_arr, th_right, dist=dist, hysteresis_dist=hysteresis_dist)

    num_valid_l = int(np.sum(valid_left))
    num_valid_r = int(np.sum(valid_right))
    if num_valid_l == 0 and num_valid_r == 0 and len(l_arr) > 0:
        print(f"\n[!] Notice: No wall readings met the control thresholds:")
        print(f"    Left sensor:  min={np.nanmin(l_arr):.1f} mm, mean={np.nanmean(l_arr):.1f} mm  (Threshold: {th_left:.1f} mm)")
        print(f"    Right sensor: min={np.nanmin(r_arr):.1f} mm, mean={np.nanmean(r_arr):.1f} mm  (Threshold: {th_right:.1f} mm)")
        print(f"    -> All readings exceed thresholds, so wall error evaluated to 0 everywhere.")
        print(f"    -> To match this run, set IR_WALL_CONTROL_TH_LEFT/RIGHT > readings (e.g. 220) or use --th-left/--th-right.\n")

    ir_error = np.zeros_like(l_arr)

    # Both walls valid
    both = valid_left & valid_right
    ir_error[both] = right_error[both] - left_error[both]

    # Only left valid
    only_left = valid_left & (~valid_right)
    ir_error[only_left] = -2.0 * left_error[only_left]

    # Only right valid
    only_right = (~valid_left) & valid_right
    ir_error[only_right] = 2.0 * right_error[only_right]

    return ir_error

def print_sensor_stats(data_dict, file_name):
    """Prints summary statistics for the IR sensors in the dataset."""
    print(f"\n--- IR Sensor Statistics: {file_name} ---")
    print(f"{'Sensor':<18} | {'Min (mm)':<10} | {'Max (mm)':<10} | {'Mean (mm)':<10} | {'Std (mm)':<10}")
    print("-" * 68)
    for key, meta in SENSOR_META.items():
        vals = data_dict.get(key, [])
        if len(vals) > 0:
            arr = np.array(vals)
            print(f"{meta['label']:<18} | {arr.min():<10.1f} | {arr.max():<10.1f} | {arr.mean():<10.1f} | {arr.std():<10.1f}")
        else:
            print(f"{meta['label']:<18} | {'No Data':<10} | {'-':<10} | {'-':<10} | {'-':<10}")
    print("-" * 68 + "\n")

def plot_single_combined(data_dict, x_axis, x_label, title):
    """Plots all 4 sensors on a single axis."""
    fig, ax = plt.subplots(figsize=(12, 6))
    for key, meta in SENSOR_META.items():
        if len(data_dict.get(key, [])) == len(x_axis):
            ax.plot(x_axis, data_dict[key], label=meta['label'], color=meta['color'], lw=1.5)
    ax.set_title(f'IR Sensors - {title}', fontsize=14)
    ax.set_xlabel(x_label)
    ax.set_ylabel('Distance (mm)')
    ax.legend(loc='best')
    ax.grid(True)
    fig.tight_layout()
    plt.show()

def plot_single_grid(data_dict, x_axis, x_label, title):
    """
    Plots each sensor in a 2x2 grid.
    On each subplot:
      - Left y-axis: Sensor distance reading (mm).
      - Right y-axis: weighted slope and its ± threshold lines.
    """
    fig, axs = plt.subplots(2, 2, figsize=(16, 11), sharex=True)
    grid_map = [
        ('sens_l',  axs[0, 0]),
        ('sens_r',  axs[0, 1]),
        ('sens_fl', axs[1, 0]),
        ('sens_fr', axs[1, 1]),
    ]
    for key, ax in grid_map:
        meta = SENSOR_META[key]
        vals = np.array(data_dict.get(key, []), dtype=float)
        if len(vals) == len(x_axis):
            # 1. Sensor reading on primary axis
            line1 = ax.plot(x_axis, vals, label=f"{meta['label']} Reading", color=meta['color'], lw=1.6)
            control_threshold = IR_WALL_CONTROL_TH_LEFT if key == 'sens_l' else (
                IR_WALL_CONTROL_TH_RIGHT if key == 'sens_r' else None)
            control_line = []
            if control_threshold is not None:
                control_line = [ax.axhline(control_threshold, color='#008c95', linestyle='--', lw=1.1,
                                           label=f'Control threshold {control_threshold:.0f} mm')]
                dist_vals = np.array(data_dict.get('dist', []), dtype=float) if 'dist' in data_dict else None
                valid, _ = compute_control_validity(vals, control_threshold, dist=dist_vals)
                invalid_color = '#1f77b4' if key == 'sens_l' else '#2ca02c'
                shade_invalid_regions(ax, x_axis, valid, invalid_color)
            ax.set_ylabel('Distance (mm)', color=meta['color'])
            ax.tick_params(axis='y', labelcolor=meta['color'])

            # 2. Weighted slope on secondary axis
            slope = compute_sensor_slope(vals)
            ax2 = ax.twinx()
            line2 = ax2.plot(x_axis, slope, label='Slope: (S0-S-3)*4 + (S-1-S-2)*1', color='#ff0000', linestyle='-.', lw=1.3, alpha=0.9)
            threshold_upper = ax2.axhline(IR_SLOPE_THRESHOLD, color='#d95f02', linestyle=':', lw=1.2,
                                           label=f'Slope +{IR_SLOPE_THRESHOLD:.0f}')
            threshold_lower = ax2.axhline(-IR_SLOPE_THRESHOLD, color='#d95f02', linestyle=':', lw=1.2,
                                           label=f'Slope -{IR_SLOPE_THRESHOLD:.0f}')
            ax2.axhline(0, color='gray', linestyle=':', alpha=0.5, lw=0.8)
            ax2.set_ylabel('Slope (Δ mm)', color='#d95f02')
            ax2.tick_params(axis='y', labelcolor='#d95f02')

            # Combined legend
            lines = line1 + control_line + line2 + [threshold_upper, threshold_lower]
            labels = [l.get_label() for l in lines]
            ax.legend(lines, labels, loc='upper right', fontsize='small')
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', transform=ax.transAxes)

        ax.set_title(meta['label'])
        ax.grid(True)

    axs[1, 0].set_xlabel(x_label)
    axs[1, 1].set_xlabel(x_label)
    fig.suptitle(f'IR Sensors, Control Validity & Slope - {title}', fontsize=15)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()

def plot_single_default(data_dict, x_axis, x_label, title,
                        ref_left=IR_WALL_DIST_REF_LEFT,
                        ref_right=IR_WALL_DIST_REF_RIGHT,
                        th_left=IR_WALL_CONTROL_TH_LEFT,
                        th_right=IR_WALL_CONTROL_TH_RIGHT):
    """
    Default view (two vertically stacked charts):
      - Top chart: Side sensors (L vs R) & IR Side Wall Error.
      - Bottom chart: Front sensors (FL vs FR) & Diff (FL - FR).
    (All 4 sensors combined view is available via --combined)
    """
    fig, (ax_side, ax_front) = plt.subplots(2, 1, figsize=(15, 9), sharex=True)

    # 1. Top Chart: Side Sensors (Left vs Right & Side Wall Error)
    has_l = len(data_dict.get('sens_l', [])) == len(x_axis)
    has_r = len(data_dict.get('sens_r', [])) == len(x_axis)
    if has_l and has_r:
        left_values = np.array(data_dict['sens_l'], dtype=float)
        right_values = np.array(data_dict['sens_r'], dtype=float)
        dist_vals = np.array(data_dict.get('dist', []), dtype=float) if 'dist' in data_dict else None
        ax_side.plot(x_axis, left_values, label=SENSOR_META['sens_l']['label'], color=SENSOR_META['sens_l']['color'], lw=1.5)
        ax_side.plot(x_axis, right_values, label=SENSOR_META['sens_r']['label'], color=SENSOR_META['sens_r']['color'], lw=1.5)
        ax_side.axhline(th_left, color=SENSOR_META['sens_l']['color'], linestyle=':', lw=1.0,
                        label=f'Left control threshold ({th_left:.0f} mm)')
        ax_side.axhline(th_right, color=SENSOR_META['sens_r']['color'], linestyle=':', lw=1.0,
                        label=f'Right control threshold ({th_right:.0f} mm)')
        left_valid, _ = compute_control_validity(left_values, th_left, dist=dist_vals)
        right_valid, _ = compute_control_validity(right_values, th_right, dist=dist_vals)
        shade_invalid_regions(ax_side, x_axis, left_valid, '#1f77b4')
        shade_invalid_regions(ax_side, x_axis, right_valid, '#2ca02c')
        
        # Calculate side wall error using firmware formula
        wall_error = ir_side_wall_error(data_dict['sens_l'], data_dict['sens_r'],
                                        ref_left=ref_left, ref_right=ref_right,
                                        th_left=th_left, th_right=th_right,
                                        dist=dist_vals)
        ax_side.plot(x_axis, wall_error, label=f'IR Side Wall Error (refL={ref_left:.0f}, refR={ref_right:.0f})', color='#e7298a', linestyle='--', lw=1.5)
        ax_side.axhline(0, color='gray', linestyle=':', alpha=0.5, lw=0.8)

    ax_side.set_title('Side Sensors (L vs R) & IR Side Wall Error', fontsize=13)
    ax_side.set_ylabel('Distance / Error (mm)')
    ax_side.legend(loc='upper right')
    ax_side.grid(True)

    # 2. Bottom Chart: Front Sensors (Front-Left vs Front-Right & Diff)
    has_fl = len(data_dict.get('sens_fl', [])) == len(x_axis)
    has_fr = len(data_dict.get('sens_fr', [])) == len(x_axis)
    if has_fl and has_fr:
        ax_front.plot(x_axis, data_dict['sens_fl'], label=SENSOR_META['sens_fl']['label'], color=SENSOR_META['sens_fl']['color'], lw=1.5)
        ax_front.plot(x_axis, data_dict['sens_fr'], label=SENSOR_META['sens_fr']['label'], color=SENSOR_META['sens_fr']['color'], lw=1.5)
        fl_arr = np.array(data_dict['sens_fl'])
        fr_arr = np.array(data_dict['sens_fr'])
        diff_front = fl_arr - fr_arr
        ax_front.plot(x_axis, diff_front, label='Diff (FL - FR)', color='gray', linestyle='--', alpha=0.8, lw=1.2)
        ax_front.axhline(0, color='gray', linestyle=':', alpha=0.5, lw=0.8)

    ax_front.set_title('Front Sensors (FL vs FR) & Difference', fontsize=13)
    ax_front.set_xlabel(x_label)
    ax_front.set_ylabel('Distance (mm)')
    ax_front.legend(loc='upper right')
    ax_front.grid(True)

    fig.suptitle(f'IR Sensor Analysis: {title}', fontsize=15)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()

def plot_comparison(d1, d2, name1, name2, use_dist=False, offset=0.0):
    """Compares IR sensors between two log files."""
    if use_dist:
        x1 = compute_cumulative_distance(d1['time'], d1.get('lin_vel_act'), d1.get('dist'))
        x2 = compute_cumulative_distance(d2['time'], d2.get('lin_vel_act'), d2.get('dist'))
        x_label = 'Cumulative Distance Travelled (mm)'
    else:
        x1 = d1['time']
        x2 = [t + offset for t in d2['time']]
        x_label = f'Time (ms)' + (f' [Offset: {offset:+.1f} ms]' if offset != 0.0 else '')

    fig, axs = plt.subplots(2, 2, figsize=(15, 10), sharex=True)
    sensor_map = [
        ('sens_l',  axs[0, 0], 'Left (L)'),
        ('sens_r',  axs[0, 1], 'Right (R)'),
        ('sens_fl', axs[1, 0], 'Front-Left (FL)'),
        ('sens_fr', axs[1, 1], 'Front-Right (FR)'),
    ]

    for key, ax, label in sensor_map:
        if len(d1.get(key, [])) == len(x1):
            ax.plot(x1, d1[key], label=f'{label} ({name1})', color='#1f77b4', lw=1.5)
        if len(d2.get(key, [])) == len(x2):
            ax.plot(x2, d2[key], label=f'{label} ({name2})', color='#ff7f0e', linestyle='--', lw=1.5)
        ax.set_title(label)
        ax.set_ylabel('Distance (mm)')
        ax.grid(True)
        ax.legend(loc='best')

    axs[1, 0].set_xlabel(x_label)
    axs[1, 1].set_xlabel(x_label)
    fig.suptitle(f'IR Sensor Comparison: {name1} vs {name2}', fontsize=15)
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()

def main():
    parser = argparse.ArgumentParser(description="Plot IR sensor distances from log files.")
    parser.add_argument('logs', nargs='+', help="Path to one or two log files.")
    parser.add_argument('--vs-dist', action='store_true', help="Plot against integrated cumulative travel distance (mm) instead of time.")
    parser.add_argument('--combined', action='store_true', help="Plot all 4 sensors on a single plot.")
    parser.add_argument('--grid', '--grud', action='store_true', dest='grid',
                        help="Plot each sensor in a 2x2 grid with control thresholds and weighted slope.")
    parser.add_argument('--offset', type=float, default=0.0, help="Timeline offset (ms) for comparison mode.")
    parser.add_argument('--ref-left', type=float, default=IR_WALL_DIST_REF_LEFT,
                        help=f"Reference left wall distance in mm (default: {IR_WALL_DIST_REF_LEFT})")
    parser.add_argument('--ref-right', type=float, default=IR_WALL_DIST_REF_RIGHT,
                        help=f"Reference right wall distance in mm (default: {IR_WALL_DIST_REF_RIGHT})")
    parser.add_argument('--th-left', type=float, default=IR_WALL_CONTROL_TH_LEFT,
                        help=f"Left wall control validity threshold in mm (default: {IR_WALL_CONTROL_TH_LEFT})")
    parser.add_argument('--th-right', type=float, default=IR_WALL_CONTROL_TH_RIGHT,
                        help=f"Right wall control validity threshold in mm (default: {IR_WALL_CONTROL_TH_RIGHT})")

    args = parser.parse_args()

    file1 = resolve_file_path(args.logs[0])
    data1 = parse_log_file(file1)
    if not data1:
        sys.exit(1)

    # Check if any sensor data is present
    total_sensor_pts = sum(len(data1.get(k, [])) for k in SENSOR_META)
    if total_sensor_pts == 0:
        print(f"[!] Warning: No IR sensor columns (SensL, SensFL, SensFR, SensR) found in '{file1}'.")
        print("    Ensure the firmware was logging sensor data (CONTROL_LOG_MODE 0).")
        sys.exit(1)

    name1 = os.path.basename(file1)
    print_sensor_stats(data1, name1)

    # Comparison Mode (2 files)
    if len(args.logs) >= 2:
        file2 = resolve_file_path(args.logs[1])
        data2 = parse_log_file(file2)
        if not data2:
            sys.exit(1)
        name2 = os.path.basename(file2)
        print_sensor_stats(data2, name2)
        plot_comparison(data1, data2, name1, name2, use_dist=args.vs_dist, offset=args.offset)
        return

    # Single File Mode
    if args.vs_dist:
        cum_dist = compute_cumulative_distance(data1['time'], data1.get('lin_vel_act'), data1.get('dist'))
        x_axis = cum_dist
        x_label = 'Cumulative Distance Travelled (mm)'
    else:
        x_axis = data1['time']
        x_label = 'Time (ms)'

    if args.combined:
        plot_single_combined(data1, x_axis, x_label, name1)
    elif args.grid:
        plot_single_grid(data1, x_axis, x_label, name1)
    else:
        plot_single_default(data1, x_axis, x_label, name1,
                            ref_left=args.ref_left, ref_right=args.ref_right,
                            th_left=args.th_left, th_right=args.th_right)

if __name__ == '__main__':
    main()
