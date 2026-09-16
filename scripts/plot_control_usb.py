"""
Robot telemetry log analyzer.

Reads semicolon-delimited telemetry lines — either live from a serial
connection or from a saved log file — figures out which columns are
present, and plots them.

Two log "modes" are auto-detected from the columns present:
    control : PID / feed-forward tuning data (no position/sensor columns)
    sensor  : odometry + IR sensor data (no PID columns)

Usage:
    python log_analyzer.py                          -> live capture from serial
    python log_analyzer.py file.txt                  -> plot a single log
    python log_analyzer.py a.txt b.txt --offset 50    -> compare two logs, time-
                                                          shifting the second by 50ms
Run with -h for the full list of options.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

import matplotlib.pyplot as plt
import serial

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
READ_TIMEOUT = 1.0

# Both of these have command-line equivalents (--plot-imu-diff, --mode) set
# in main(); the values here are just the defaults used when the module is
# imported directly rather than run as a script.
PLOT_IMU_DIFF = False   # True: show the IMU/encoder-diff panel; False: its per-mode alternative
DEFAULT_MODE = 'sensor'  # Fallback guess used for live capture format, and when a
                          # file's own columns/header don't reveal its mode.

# ---------------------------------------------------------------------------
# Column identity
# ---------------------------------------------------------------------------
# Every header spelling seen across firmware/log revisions, mapped to one
# canonical field name. This is the single source of truth for what a
# column *is* — everything below (fixed-width fallback layouts, mode
# detection, plot panels) just refers to these canonical names instead of
# re-describing the data.
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
    'rawvell': 'raw_vel_l', 'rawvelr': 'raw_vel_r', 'rawangvel': 'raw_ang_vel',
}

# Columns that make a log unambiguously one mode or the other.
CONTROL_ONLY_KEYS = {'vel_p', 'ang_p', 'rotation_ff'}
SENSOR_ONLY_KEYS = {'sens_l', 'sens_fl', 'pos_x'}

# Fixed column layouts, used only when a file has no header line we can
# parse. column_count -> [(mode, [canonical keys in order]), ...].
# 13 columns is inherently ambiguous between the two historical layouts, so
# both candidates are listed and disambiguated using whatever header text
# is available (see _resolve_columns).
FALLBACK_LAYOUTS = {
    16: [('sensor', ['time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                      'pwm_left', 'pwm_right', 'imu_diff', 'pos_x', 'pos_y', 'angle', 'dist',
                      'sens_l', 'sens_fl', 'sens_fr', 'sens_r'])],
    15: [('control', ['time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                       'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i',
                       'rotation_ff', 'linear_ff', 'battery'])],
    14: [('control', ['time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                       'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i',
                       'rotation_ff', 'linear_ff'])],
    13: [('sensor', ['time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                      'pwm_left', 'pwm_right', 'imu_diff', 'battery', 'pos_x', 'pos_y', 'angle', 'dist']),
         ('control', ['time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
                       'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i',
                       'rotation_ff'])],
}

# Layout + header text used for a live serial capture, keyed by mode.
LIVE_CAPTURE_LAYOUTS = {
    'control': FALLBACK_LAYOUTS[15][0][1],
    'sensor': FALLBACK_LAYOUTS[16][0][1],
}
LIVE_CAPTURE_HEADERS = {
    'control': (
        "Time(ms);ActualLinearVel;TargetLinearVel;ActualAngularVel;TargetAngularVel;"
        "PWML;PWMR;ImuDiff;VelP;VelI;AngP;AngI;RotationFF;LinearFF;Battery(mV)"
    ),
    'sensor': (
        "Time(ms);ActualLinearVel;TargetLinearVel;ActualAngularVel;TargetAngularVel;"
        "PWML;PWMR;ImuDiff;PosX(m);PosY(m);Angle(rad);Dist;"
        "SensL(mm);SensFL(mm);SensFR(mm);SensR(mm)"
    ),
}


# ---------------------------------------------------------------------------
# Parsed log data
# ---------------------------------------------------------------------------
@dataclass
class LogData:
    mode: str                                    # 'control' or 'sensor'
    series: dict = field(default_factory=dict)    # canonical_key -> [float, ...]

    def get(self, key: str) -> list:
        return self.series.get(key, [])

    def has(self, key: str) -> bool:
        """True if `key` has data aligned one-to-one with the time column."""
        values = self.series.get(key, [])
        return bool(values) and len(values) == len(self.series.get('time', []))


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
def _normalize_token(token: str) -> Optional[str]:
    clean = token.split('(')[0].strip().lower()
    if clean in HEADER_KEY_MAP:
        return HEADER_KEY_MAP[clean]
    clean_nopunct = ''.join(c for c in token.strip().lower() if c.isalnum() or c == '_')
    return HEADER_KEY_MAP.get(clean_nopunct)


def _parse_header_line(header_line: str, expected_count: int) -> Optional[list]:
    """Maps a semicolon-separated header line to canonical keys, or returns
    None if it doesn't fully resolve (wrong length, or an unknown column)."""
    keys = [_normalize_token(p) for p in header_line.split(';')]
    if len(keys) == expected_count and all(keys):
        return keys
    return None


def _split_header_and_data(lines: list) -> tuple:
    """Separates comment/header lines from numeric data lines, skipping any
    `general_params = { ... };` block."""
    header_lines, data_lines = [], []
    in_param_block = False
    for line in lines:
        if line.startswith("general_params = {"):
            in_param_block = True
            continue
        if in_param_block:
            if line.startswith("};") or line == "}":
                in_param_block = False
            continue
        try:
            [float(x) for x in line.split(';')]
            data_lines.append(line)
        except ValueError:
            header_lines.append(line)
    return header_lines, data_lines


def _resolve_columns(header_lines: list, cols_count: int) -> Optional[list]:
    """Figures out the canonical key for every column, preferring an actual
    header line (closest to the data first) and falling back to a known
    fixed layout for that column count."""
    for h in reversed(header_lines):
        keys = _parse_header_line(h, cols_count)
        if keys:
            return keys

    layouts = FALLBACK_LAYOUTS.get(cols_count)
    if not layouts:
        return None
    if len(layouts) == 1:
        return layouts[0][1]

    # Ambiguous column count (13): use whatever header text is present as a hint,
    # falling back to DEFAULT_MODE if there's no hint either.
    looks_positional = any('pos' in h.lower() or 'dist' in h.lower() for h in header_lines)
    wanted_mode = 'sensor' if looks_positional else DEFAULT_MODE
    for mode, keys in layouts:
        if mode == wanted_mode:
            return keys
    return layouts[0][1]


def _detect_mode(keys: list) -> str:
    key_set = set(keys)
    if key_set & SENSOR_ONLY_KEYS:
        return 'sensor'
    if key_set & CONTROL_ONLY_KEYS:
        return 'control'
    return DEFAULT_MODE


def parse_log_file(path: str) -> Optional[LogData]:
    """Reads a log file and returns its parsed data, or None on failure."""
    if not os.path.exists(path):
        alt_path = os.path.join(os.path.dirname(__file__), path)
        path = alt_path if os.path.exists(alt_path) else path
    if not os.path.exists(path):
        print(f"Error: Log file '{path}' not found.")
        return None

    with open(path) as f:
        lines = [line.strip() for line in f if line.strip()]
    if not lines:
        print(f"Log file '{path}' is empty.")
        return None

    header_lines, data_lines = _split_header_and_data(lines)
    if not data_lines:
        print(f"No valid numerical data rows found in '{path}'.")
        return None

    cols_count = len(data_lines[0].split(';'))
    keys = _resolve_columns(header_lines, cols_count)
    if keys is None:
        print(f"Unknown column layout ({cols_count} columns) in '{path}'.")
        return None

    series = {k: [] for k in keys}
    for line in data_lines:
        try:
            values = [float(x) for x in line.split(';')]
        except ValueError:
            continue
        for key, value in zip(keys, values):
            series[key].append(value)

    if not series.get('time'):
        print(f"No valid numerical data could be parsed from '{path}'.")
        return None

    return LogData(mode=_detect_mode(keys), series=series)


def save_log_to_disk(header: str, data_lines: list) -> None:
    """Saves raw log lines into a date-organized folder under ./logs."""
    date_folder = os.path.join("logs", datetime.now().strftime("%Y-%m-%d"))
    os.makedirs(date_folder, exist_ok=True)
    log_path = os.path.join(date_folder, f"log_{datetime.now().strftime('%H-%M-%S')}.txt")

    looks_like_header = any(
        ';' in line and not line.startswith('general_params')
        and any(col in line.lower() for col in ('vel', 'tgt', 'ang', 'time'))
        for line in data_lines
    )
    try:
        with open(log_path, "w") as f:
            if not looks_like_header and header:
                f.write(header + "\n")
            f.write("\n".join(data_lines))
        print(f"\n[Success] Log safely stored to: {log_path}")
    except OSError as e:
        print(f"Error saving log file: {e}")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
@dataclass
class Series:
    key: str
    label: str
    style: str = '-'            # matplotlib linestyle
    optional: bool = False      # if missing, just skip this line rather than the panel
    dash_for_other: bool = False  # in a comparison plot, draw this dashed for every
                                   # dataset after the first (helps tell same-styled
                                   # lines like left/right apart when color alone isn't enough)


@dataclass
class Panel:
    title: str
    ylabel: str
    series: list
    xlabel: str = 'Time (ms)'
    kind: str = 'time'          # 'time': x=time, y=each series | 'xy': series = [x_series, y_series]
    fallback: Optional["Panel"] = None   # used if this panel's data isn't available at all

    def is_available(self, data: LogData) -> bool:
        required = [s for s in self.series if not s.optional]
        if required:
            return all(data.has(s.key) for s in required)
        return any(data.has(s.key) for s in self.series)


def _velocity_panels() -> list:
    return [
        Panel('Linear Velocity', 'm/s', [
            Series('lin_vel_act', 'Actual Linear Velocity'),
            Series('lin_vel_tgt', 'Target', style='--'),
        ]),
        Panel('Angular Velocity', 'rad/s', [
            Series('ang_vel_act', 'Actual Angular Velocity'),
            Series('ang_vel_tgt', 'Target', style='--'),
        ]),
    ]


def _imu_or_alternate_panel(mode: str) -> Panel:
    if PLOT_IMU_DIFF:
        return Panel('IMU & Encoder Variance', 'Difference', [Series('imu_diff', 'IMU Encoder Diff')])
    if mode == 'control':
        return Panel('Velocity PID Terms', 'Value', [
            Series('vel_p', 'Velocity P Term'),
            Series('vel_i', 'Velocity I Term'),
        ])
    return Panel('Distance over Time', 'Distance', [Series('dist', 'Distance')])


def _pwm_panel() -> Panel:
    return Panel('PWM Signals', 'Duty Cycle (0-1000)', [
        Series('pwm_left', 'PWM Left'),
        Series('pwm_right', 'PWM Right', dash_for_other=True),
    ])


def get_panel_grid(mode: str, context: str) -> list:
    """Builds the 3x2 grid of panels for a log `mode` ('control'/'sensor')
    rendered in a given `context` ('single'/'comparison')."""
    row1 = _velocity_panels()
    row3_left = _pwm_panel()

    if mode == 'control':
        row2 = [
            _imu_or_alternate_panel(mode),
            Panel('Angular Velocity PID Terms', 'Value', [
                Series('ang_p', 'Angular P Term'),
                Series('ang_i', 'Angular I Term'),
            ]),
        ]
        row3_right = Panel('Feed Forward Terms', 'Value', [
            Series('rotation_ff', 'Rotation Feedforward'),
            Series('linear_ff', 'Linear Feedforward', optional=True),
        ])
    else:
        if context == 'comparison':
            second_panel = Panel(
                'Spatial Odometry Tracking', 'Y Position (m)',
                [Series('pos_x', 'Path'), Series('pos_y', 'Path')],
                xlabel='X Position (m)', kind='xy',
            )
        else:
            second_panel = Panel('Angle over Time', 'Angle (rad)', [Series('angle', 'Angle')])
        row2 = [_imu_or_alternate_panel(mode), second_panel]

        row3_right = Panel('Sensor Distances (mm)', 'Distance (mm)', [
            Series('sens_l', 'Sens Left', dash_for_other=True),
            Series('sens_fl', 'Sens Front Left', style='--', optional=True),
            Series('sens_fr', 'Sens Front Right', style=':', optional=True),
            Series('sens_r', 'Sens Right', optional=True, dash_for_other=True),
        ])
        row3_right.fallback = Panel('Battery Voltage over Time', 'Voltage (mV)',
                                     [Series('battery', 'Battery Voltage')])

    return [row1, row2, [row3_left, row3_right]]


def _resolve_panel(panel: Panel, datasets: list) -> Panel:
    """Swaps in `panel.fallback` if none of the datasets have the panel's data."""
    if panel.fallback and not any(panel.is_available(d) for d in datasets):
        return panel.fallback
    return panel


def _render_panel(ax, panel: Panel, datasets: list) -> None:
    """`datasets` is a list of (label, color, LogData). color=None lets
    matplotlib auto-assign colors."""
    multi = len(datasets) > 1

    if panel.kind == 'xy':
        x_key, y_key = panel.series[0].key, panel.series[1].key
        for name, color, data in datasets:
            xs, ys = data.get(x_key), data.get(y_key)
            if not xs or not ys:
                continue
            label = f'Path ({name})' if multi else 'Path'
            ax.plot(xs, ys, color=color, label=label, marker='o', markersize=2, alpha=0.7)
        ax.axis('equal')
    else:
        for idx, (name, color, data) in enumerate(datasets):
            times = data.get('time')
            for s in panel.series:
                values = data.get(s.key)
                if not values or len(values) != len(times):
                    continue
                style = '--' if (multi and idx > 0 and s.dash_for_other) else s.style
                label = f'{s.label} ({name})' if multi else s.label
                ax.plot(times, values, style, color=color, label=label)

    ax.set_title(panel.title)
    ax.set_xlabel(panel.xlabel)
    ax.set_ylabel(panel.ylabel)
    ax.tick_params(labelbottom=True)
    ax.grid(True)
    if ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize='small')


def plot_single(data: LogData, title_suffix: str = "") -> None:
    grid = get_panel_grid(data.mode, context='single')
    fig, axs = plt.subplots(3, 2, figsize=(15, 12), sharex=True)

    mode_title = 'Motion Control Performance' if data.mode == 'control' else 'System and Sensor Data'
    fig.suptitle(f'{mode_title} {title_suffix}', fontsize=16)

    for row_panels, ax_row in zip(grid, axs):
        for panel, ax in zip(row_panels, ax_row):
            resolved = _resolve_panel(panel, [data])
            _render_panel(ax, resolved, [('', None, data)])

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


def plot_comparison(file1: str, file2: str, offset: float = 0.0) -> None:
    d1, d2 = parse_log_file(file1), parse_log_file(file2)
    if not d1 or not d2:
        return

    name1, name2 = os.path.basename(file1), os.path.basename(file2)
    if offset:
        d2.series['time'] = [t + offset for t in d2.get('time')]

    mode = d1.mode  # if the two logs disagree, defer to the first file's mode
    grid = get_panel_grid(mode, context='comparison')
    datasets = [(name1, 'tab:blue', d1), (name2, 'tab:green', d2)]

    resolved_grid = [[_resolve_panel(panel, [d1, d2]) for panel in row] for row in grid]

    fig, axs = plt.subplots(3, 2, figsize=(16, 12))
    offset_msg = f' (Shifted by {offset:+.1f} ms)' if offset else ''
    fig.suptitle(f'Comparison: {name1} vs {name2}{offset_msg}', fontsize=16)

    # Link the x-axis across every time-based panel (not the xy odometry plot).
    time_axes = [axs[r][c] for r in range(3) for c in range(2) if resolved_grid[r][c].kind == 'time']
    for ax in time_axes[1:]:
        ax.sharex(time_axes[0])

    for row_panels, ax_row in zip(resolved_grid, axs):
        for panel, ax in zip(row_panels, ax_row):
            _render_panel(ax, panel, datasets)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


# ---------------------------------------------------------------------------
# Live serial capture
# ---------------------------------------------------------------------------
def collect_print_and_plot_data() -> None:
    """Connects to the serial port, streams a burst of telemetry to disk,
    then plots it once the stream ends."""
    mode = DEFAULT_MODE
    keys = LIVE_CAPTURE_LAYOUTS[mode]
    header = LIVE_CAPTURE_HEADERS[mode]

    series = {k: [] for k in keys}
    data_lines = []

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT) as ser:
            ser.flush()
            print(f"Successfully connected to {SERIAL_PORT}")
            print("Waiting for data burst... Press physical button or use Ctrl+C to exit.")

            line = ""
            while not line:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
            print("Data reception started...")

            while line:
                data_lines.append(line)
                try:
                    values = [float(x) for x in line.split(';')]
                    for key, value in zip(keys, values):
                        series[key].append(value)
                except ValueError:
                    pass
                line = ser.readline().decode('utf-8', errors='ignore').strip()

            print(f"Data collection complete. Received {len(series['time'])} data points.")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
        return

    if not series['time']:
        print("No data was collected. Exiting.")
        return

    save_log_to_disk(header, data_lines)
    plot_single(LogData(mode=mode, series=series), title_suffix="(Live Session)")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def _signal_handler(sig, frame):
    print("\nCtrl+C detected! Closing program.")
    sys.exit(0)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Parse and plot robot telemetry logs, live from serial or from saved files.",
    )
    parser.add_argument(
        'files', nargs='*', metavar='FILE',
        help="Log file(s) to plot. None: live serial capture. One: plot that log. "
             "Two: compare them side by side.",
    )
    parser.add_argument(
        '--offset', type=float, default=0.0, metavar='MS',
        help="Time offset in milliseconds applied to the second file when comparing "
             "two logs. Default: 0.",
    )
    parser.add_argument(
        '--plot-imu-diff', action='store_true', default=False,
        help="Show the IMU/encoder-diff panel. If not set, that panel slot instead shows "
             "its per-mode alternative (velocity PID terms for control logs, distance-over-"
             "time for sensor logs). Default: off.",
    )
    parser.add_argument(
        '--mode', choices=['control', 'sensor'], default='sensor',
        help="Log format to use for live serial capture, and the fallback guess when a "
             "file's mode can't be determined from its columns or header. Default: sensor.",
    )
    return parser


def main() -> None:
    global PLOT_IMU_DIFF, DEFAULT_MODE

    signal.signal(signal.SIGINT, _signal_handler)
    args = _build_arg_parser().parse_args()

    PLOT_IMU_DIFF = args.plot_imu_diff
    DEFAULT_MODE = args.mode

    if len(args.files) == 1:
        data = parse_log_file(args.files[0])
        if data:
            plot_single(data, title_suffix=f"({os.path.basename(args.files[0])})")
    elif len(args.files) >= 2:
        plot_comparison(args.files[0], args.files[1], offset=args.offset)
    else:
        collect_print_and_plot_data()


if __name__ == "__main__":
    main()