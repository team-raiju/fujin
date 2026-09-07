#!/usr/bin/env python3
"""
Velocity Filter Comparison & Evaluation
Analyzes raw encoder velocities and benchmarks the TOP 5 digital filters:
  1. Current Firmware (EMA 16Hz) [Baseline]
  2. Hybrid Kinematic Alpha-Beta + Light EMA [Top Performer: 2.85ms lag, 71% jitter reduction]
  3. Butterworth 2nd-Order LPF (fc=35Hz Biquad) [-40 dB/decade roll-off, 6.35ms lag]
  4. Cascaded 2-Stage EMA (alpha=0.10) [Ultra-clean 2-pole smoothing in 2 lines of code]
  5. Butterworth 2nd-Order LPF (fc=20Hz Biquad) [Ultra-smooth derivative: 0.63 mm/s jitter]

Plots individual pairs against Target Velocity and ranks performance.
"""

import sys
import os
import math
import argparse
import numpy as np
import scipy.signal as signal
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

# Import log parser from plot_control_usb if available, else define fallback
try:
    from plot_control_usb import parse_log_file
except ImportError:
    def parse_log_file(file_path):
        if not os.path.exists(file_path):
            print(f"Error: Log file '{file_path}' not found.")
            return None
        with open(file_path, 'r') as f:
            lines = f.readlines()
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i',
            'rotation_ff', 'linear_ff', 'raw_vel_l', 'raw_vel_r'
        ]
        data_dict = {k: [] for k in keys}
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
            return None
        return data_dict


# ==============================================================================
# Filter Classes: 2nd-Order Butterworth (Direct Form II Biquad)
# ==============================================================================

class Butterworth2ndOrderLPF:
    """
    2nd-Order Butterworth Low-Pass Filter (Direct Form II Biquad).
    Steep -40 dB/decade roll-off.
    """
    def __init__(self, cutoff_hz=35.0, sample_rate_hz=2000.0):
        self.fs = float(sample_rate_hz)
        self.fc = float(cutoff_hz)
        self.reset()

    def reset(self):
        self.x1 = self.x2 = 0.0
        self.y1 = self.y2 = 0.0
        self._compute_coefficients()

    def _compute_coefficients(self):
        K = math.tan(math.pi * self.fc / self.fs)
        K2 = K * K
        sqrt2_K = math.sqrt(2.0) * K
        a0 = 1.0 + sqrt2_K + K2
        self.b0 = K2 / a0
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (K2 - 1.0) / a0
        self.a2 = (1.0 - sqrt2_K + K2) / a0

    def process(self, x):
        y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
        self.x2 = self.x1
        self.x1 = x
        self.y2 = self.y1
        self.y1 = y
        return y


# ==============================================================================
# Top 5 Filter Implementations
# ==============================================================================

def filter_current_firmware(raw):
    """
    Filter 1: Current Firmware (1st-order EMA 16Hz) [Baseline Reference]
    y[k] = 0.051 * x[k] + 0.949 * y[k-1]
    """
    y = np.zeros_like(raw)
    for i in range(1, len(raw)):
        y[i] = raw[i] * 0.051 + y[i-1] * 0.949
    return y


def filter_hybrid_alpha_beta_ema(raw, dt_s=0.0005, alpha=0.030, beta=0.00025, alpha_post=0.30):
    """
    Filter 2: Hybrid Kinematic Alpha-Beta + Light Post-EMA [Top Performer]
    Tracks acceleration to eliminate ramp lag (2.85 ms) while post-smoothing
    eliminates 71% of derivative jitter.
    """
    y = np.zeros_like(raw)
    v_est = 0.0
    a_est = 0.0
    s_post = 0.0
    for i in range(len(raw)):
        v_pred = v_est + a_est * dt_s
        res = raw[i] - v_pred
        v_est = v_pred + alpha * res
        a_est = a_est + (beta / dt_s) * res
        s_post += alpha_post * (v_est - s_post)
        y[i] = s_post
    return y


def filter_butterworth_biquad(raw, cutoff_hz=35.0, sample_rate_hz=2000.0):
    """
    Filter 3: Butterworth 2nd-Order LPF (fc = 35Hz Biquad)
    Classical -40 dB/decade frequency-domain attenuation.
    """
    lpf = Butterworth2ndOrderLPF(cutoff_hz=cutoff_hz, sample_rate_hz=sample_rate_hz)
    return np.array([lpf.process(x) for x in raw])


def filter_cascaded_2x_ema(raw, alpha=0.10):
    """
    Filter 4: Cascaded 2-Stage EMA (alpha = 0.10)
    Two 1-pole EMA stages in series giving -40 dB/decade roll-off in 2 lines of C++.
    """
    y = np.zeros_like(raw)
    s1, s2 = 0.0, 0.0
    for i in range(len(raw)):
        s1 += alpha * (raw[i] - s1)
        s2 += alpha * (s1 - s2)
        y[i] = s2
    return y


def filter_butterworth_20hz(raw, sample_rate_hz=2000.0):
    """
    Filter 5: Butterworth 2nd-Order LPF (fc = 20Hz Biquad)
    Provides ultra-smooth derivative filtering (0.63 mm/s jitter, 75% smoother than baseline).
    """
    return filter_butterworth_biquad(raw, cutoff_hz=20.0, sample_rate_hz=sample_rate_hz)


# ==============================================================================
# Metric Computation & Ranking Engine
# ==============================================================================

def compute_metrics(t, raw, filtered_dict, ref_ground_truth):
    """
    Computes performance metrics for the filters:
      - Delay (ms): phase lag during acceleration ramp (100 to 350 ms)
      - Ramp Peak-to-Peak (mm/s): maximum ripple envelope during ramp
      - Ramp Diff Jitter (mm/s): sample-to-sample delta standard deviation on ramp
      - Steady-State Ripple Std Dev & Peak-to-Peak (mm/s)
      - Dynamic Tracking RMSE (mm/s)
      - Composite Ranking Score (lower is better)
    """
    mask_ramp = (t >= 100) & (t <= 350)
    mask_ss = (t >= 700) & (t <= 900)

    t_ramp = t[mask_ramp]
    ref_ramp = ref_ground_truth[mask_ramp]
    shifts = np.linspace(-2.0, 15.0, 341)

    results = {}

    for name, y in filtered_dict.items():
        y = np.array(y)
        # 1. Phase Lag / Delay during acceleration
        interp = interp1d(t, y, bounds_error=False, fill_value='extrapolate')
        rmses = [np.sqrt(np.mean((interp(t_ramp + s) - ref_ramp) ** 2)) for s in shifts]
        delay_ms = shifts[np.argmin(rmses)]

        # 2. Ramp Ripple Metrics (detrended with 2nd order polynomial)
        p = np.polyfit(t_ramp, y[mask_ramp], 2)
        ramp_fit = np.polyval(p, t_ramp)
        ramp_ptp_mms = float(np.ptp(y[mask_ramp] - ramp_fit) * 1000.0)
        ramp_diff_mms = float(np.std(np.diff(y[mask_ramp])) * 1000.0)

        # 3. Steady-State Ripple (mm/s)
        y_ss = y[mask_ss]
        ss_std_mms = float(np.std(y_ss - np.mean(y_ss)) * 1000.0)
        ss_ptp_mms = float(np.ptp(y_ss) * 1000.0)

        # 4. Dynamic Tracking RMSE (mm/s)
        dyn_rmse_mms = float(np.sqrt(np.mean((y - ref_ground_truth) ** 2)) * 1000.0)

        results[name] = {
            'y': y,
            'delay_ms': delay_ms,
            'ramp_ptp_mms': ramp_ptp_mms,
            'ramp_diff_mms': ramp_diff_mms,
            'ss_std_mms': ss_std_mms,
            'ss_ptp_mms': ss_ptp_mms,
            'dyn_rmse_mms': dyn_rmse_mms,
        }

    # Composite score prioritizing low delay, low ramp ripple, and low jitter
    all_delays = [max(0.0, r['delay_ms']) for r in results.values()]
    all_ramp_ptp = [r['ramp_ptp_mms'] for r in results.values()]
    all_diffs = [r['ramp_diff_mms'] for r in results.values()]

    max_delay = max(all_delays) if max(all_delays) > 0 else 1.0
    max_ramp_ptp = max(all_ramp_ptp) if max(all_ramp_ptp) > 0 else 1.0
    max_diff = max(all_diffs) if max(all_diffs) > 0 else 1.0

    for name, r in results.items():
        norm_delay = max(0.0, r['delay_ms']) / max_delay
        norm_ramp = r['ramp_ptp_mms'] / max_ramp_ptp
        norm_diff = r['ramp_diff_mms'] / max_diff
        r['score'] = (0.35 * norm_delay) + (0.35 * norm_ramp) + (0.30 * norm_diff)

    return results


def print_ranking_table(results):
    """Prints ranked statistics on the terminal."""
    sorted_items = sorted(results.items(), key=lambda x: x[1]['score'])

    print("\n" + "=" * 122)
    print("                      TOP 5 VELOCITY FILTERS BENCHMARK & PERFORMANCE RANKING")
    print("=" * 122)
    header = (
        f"{'Rank':<5} | {'Filter Name':<38} | {'Delay':>9} | {'Ramp Pk-Pk':>11} | "
        f"{'Ramp Jitter':>11} | {'SS Std':>10} | {'Dyn RMSE':>10} | {'Score':>7}"
    )
    print(header)
    units = (
        f"{'':<5} | {'':<38} | {'(ms)':>9} | {'(mm/s)':>11} | "
        f"{'(mm/s)':>11} | {'(mm/s)':>10} | {'(mm/s)':>10} | {'(0-1)':>7}"
    )
    print(units)
    print("-" * 122)

    baseline_name = "Current Firmware (EMA 16Hz)"
    base_res = results.get(baseline_name)

    for rank, (name, r) in enumerate(sorted_items, 1):
        is_baseline = (name == baseline_name)
        prefix = f"#{rank:<3}"
        marker = " (Base)" if is_baseline else ""
        name_str = (name + marker)[:38]

        line = (
            f"{prefix} | {name_str:<38} | {r['delay_ms']:9.2f} | {r['ramp_ptp_mms']:11.2f} | "
            f"{r['ramp_diff_mms']:11.2f} | {r['ss_std_mms']:10.2f} | {r['dyn_rmse_mms']:10.2f} | {r['score']:7.3f}"
        )
        print(line)

    print("-" * 122)
    if base_res:
        best_name, best_res = sorted_items[0]
        print(f"\n[SUMMARY] Top Ranked Filter: '{best_name}'")
        if best_name != baseline_name:
            delay_improv = (base_res['delay_ms'] - best_res['delay_ms'])
            diff_improv = ((base_res['ramp_diff_mms'] - best_res['ramp_diff_mms']) / base_res['ramp_diff_mms']) * 100.0
            ptp_improv = ((base_res['ramp_ptp_mms'] - best_res['ramp_ptp_mms']) / base_res['ramp_ptp_mms']) * 100.0
            print(f"  * Delay Improvement:   {delay_improv:+.2f} ms faster ({best_res['delay_ms']:.2f} ms vs {base_res['delay_ms']:.2f} ms)")
            print(f"  * Ramp Peak-to-Peak:   {ptp_improv:+.1f}% ripple envelope reduction")
            print(f"  * Derivative Jitter:   {diff_improv:+.1f}% smoother derivative on ramp")
    print("=" * 122 + "\n")


def print_cpp_code_snippets():
    """Prints ready-to-use C++ snippets for firmware implementation."""
    print("=" * 122)
    print("              C++ CODE SNIPPETS FOR STM32 FIRMWARE IMPLEMENTATION (encoders.cpp)")
    print("=" * 122)

    print("""// ============================================================================
// 1. Hybrid Kinematic Alpha-Beta + Light Post-EMA (Rank #1 Champion)
// Low lag (2.85 ms) + 71% derivative jitter reduction on acceleration ramps
// ============================================================================
float update_filter_hybrid_alpha_beta_ema(float linear_vel_m_s) {
    static float v_est = 0.0f;
    static float a_est = 0.0f;
    static float filtered_out = 0.0f;

    constexpr float dt = 0.0005f;                 // 0.5 ms (2 kHz)
    constexpr float alpha = 0.030f;
    constexpr float beta_div_dt = 0.5000f;        // 0.00025 / 0.0005
    constexpr float alpha_post = 0.30f;

    float v_pred = v_est + a_est * dt;
    float residual = linear_vel_m_s - v_pred;

    v_est = v_pred + alpha * residual;
    a_est = a_est + beta_div_dt * residual;
    filtered_out += alpha_post * (v_est - filtered_out);
    return filtered_out;
}

// ============================================================================
// 2. Butterworth 2nd-Order Low-Pass Filter (Direct Form II Biquad)
// Pure -40 dB/decade frequency-domain attenuation
// ============================================================================

// --- 35 Hz Cutoff (6.35 ms lag, 0.97 mm/s derivative jitter) ---
float update_filter_butterworth_35hz(float linear_vel_m_s) {
    static float x1 = 0.0f, x2 = 0.0f;
    static float y1 = 0.0f, y2 = 0.0f;

    constexpr float b0 = 0.00280210f;
    constexpr float b1 = 0.00560419f;
    constexpr float b2 = 0.00280210f;
    constexpr float a1 = -1.84477841f;
    constexpr float a2 = 0.85598679f;

    float y0 = b0 * linear_vel_m_s + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

    x2 = x1;
    x1 = linear_vel_m_s;
    y2 = y1;
    y1 = y0;
    return y0;
}

// --- 20 Hz Cutoff (11.35 ms lag, ultra-smooth 0.63 mm/s derivative jitter) ---
float update_filter_butterworth_20hz(float linear_vel_m_s) {
    static float x1 = 0.0f, x2 = 0.0f;
    static float y1 = 0.0f, y2 = 0.0f;

    constexpr float b0 = 0.00094469f;
    constexpr float b1 = 0.00188938f;
    constexpr float b2 = 0.00094469f;
    constexpr float a1 = -1.91119707f;
    constexpr float a2 = 0.91497583f;

    float y0 = b0 * linear_vel_m_s + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

    x2 = x1;
    x1 = linear_vel_m_s;
    y2 = y1;
    y1 = y0;
    return y0;
}

// ============================================================================
// 3. Cascaded 2-Stage EMA (alpha = 0.10, -40 dB/decade roll-off)
// Simplest possible 2-pole low-pass filter: only 2 lines of C++!
// ============================================================================
float update_filter_cascaded_2x_ema(float linear_vel_m_s) {
    static float s1 = 0.0f, s2 = 0.0f;
    constexpr float alpha = 0.10f;
    s1 += alpha * (linear_vel_m_s - s1);
    s2 += alpha * (s1 - s2);
    return s2;
}
""")
    print("=" * 122 + "\n")


# ==============================================================================
# Plotting Function
# ==============================================================================

def plot_pairs_grid(t, raw, tgt, results, save_path=None):
    """
    Plots separate charts for each of the 5 filters paired with target velocity,
    plus a 6th combined plot containing all filters together.
    All subplots share x and y axes for synchronous zoom and pan.
    """
    fig, axs = plt.subplots(3, 2, figsize=(16, 12), sharex=True, sharey=True)
    fig.suptitle('Top 5 Velocity Filters: Target Velocity vs Filtered Velocity', fontsize=15, fontweight='bold')

    palette = ['#212121', '#00897b', '#e65100', '#1e88e5', '#8e24aa']
    flat_axs = axs.flatten()
    items = list(results.items())

    # First 5 subplots: Individual filter pairs
    for idx, (name, r) in enumerate(items):
        if idx >= 5:
            break
        ax = flat_axs[idx]
        c = palette[idx % len(palette)]

        # Background raw pulses
        ax.plot(t, raw, label='Raw Velocity', color='#b0bec5', alpha=0.35, linewidth=0.8)
        # Target velocity
        ax.plot(t, tgt, label='Target Velocity', color='#2e7d32', linestyle='--', linewidth=1.8)
        # Filtered output
        ax.plot(t, r['y'], label=f'Filtered: {name}', color=c, linewidth=1.7)

        title_str = (f"{name}\n"
                     f"Delay: {r['delay_ms']:+.2f} ms | "
                     f"Ramp Pk-Pk: {r['ramp_ptp_mms']:.1f} mm/s | "
                     f"Jitter: {r['ramp_diff_mms']:.2f} mm/s")
        ax.set_title(title_str, fontsize=10.0, fontweight='semibold')
        ax.set_ylabel('Velocity (m/s)')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='lower right', fontsize=8, framealpha=0.9)

    # 6th Subplot: All Filters Combined vs Target Velocity
    ax_all = flat_axs[5]
    ax_all.plot(t, raw, label='Raw Velocity', color='#b0bec5', alpha=0.3, linewidth=0.7)
    ax_all.plot(t, tgt, label='Target Velocity', color='#2e7d32', linestyle=':', linewidth=2.2)

    for idx, (name, r) in enumerate(items):
        c = palette[idx % len(palette)]
        lw = 1.6 if idx == 0 else 1.3
        ls = '--' if idx == 0 else '-'
        ax_all.plot(t, r['y'], label=name, color=c, linestyle=ls, linewidth=lw)

    ax_all.set_title('All 5 Filters Combined vs Target Velocity', fontsize=11, fontweight='bold')
    ax_all.set_ylabel('Velocity (m/s)')
    ax_all.grid(True, alpha=0.3)
    ax_all.legend(loc='lower right', fontsize=7.5, framealpha=0.9)

    flat_axs[4].set_xlabel('Time (ms)')
    flat_axs[5].set_xlabel('Time (ms)')

    fig.tight_layout(rect=[0, 0.02, 1, 0.96])

    if save_path:
        plt.savefig(save_path, dpi=200)
        print(f"[Success] Plot saved to: {save_path}")

    return fig


# ==============================================================================
# Main Program
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="Evaluate and compare top 5 velocity filters on raw velocity logs.")
    parser.add_argument("log_path", nargs="?", default="logs/2026-09-07/log_12-01-28.txt",
                        help="Path to log file containing raw velocity columns.")
    parser.add_argument("--save", dest="save_path", default=None,
                        help="Path to save output figure (e.g. filter_results.png).")
    parser.add_argument("--no-gui", action="store_true", help="Run without opening GUI window.")
    args = parser.parse_args()

    # Locate log file
    log_path = args.log_path
    if not os.path.exists(log_path):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        cand_path = os.path.join(script_dir, log_path)
        if os.path.exists(cand_path):
            log_path = cand_path
        else:
            print(f"Error: Log file '{args.log_path}' not found.")
            sys.exit(1)

    print(f"[*] Loading log data from: {log_path}")
    data = parse_log_file(log_path)
    if not data:
        print("Failed to parse log file.")
        sys.exit(1)

    t = np.array(data['time'])
    if 'raw_vel_l' not in data or 'raw_vel_r' not in data or len(data['raw_vel_l']) == 0:
        print("Error: Log file does not contain 'raw_vel_l' and 'raw_vel_r' data columns.")
        sys.exit(1)

    raw_l = np.array(data['raw_vel_l'])
    raw_r = np.array(data['raw_vel_r'])
    raw = (raw_l + raw_r) / 2.0
    tgt = np.array(data['lin_vel_tgt'])
    dt_ms = np.median(np.diff(t))
    fs = 1000.0 / dt_ms
    dt_s = dt_ms / 1000.0
    wheel_radius_m = 0.01275  # 12.75 mm wheel radius from encoders.cpp

    print(f"[*] Total samples: {len(t)} | Duration: {t[-1] - t[0]:.1f} ms | dt: {dt_ms:.2f} ms (Fs = {fs:.0f} Hz)")

    # Ground truth reference (zero-phase Butterworth)
    b_ref, a_ref = signal.butter(4, 20.0 / (fs / 2.0), btype='low')
    ref_ground_truth = signal.filtfilt(b_ref, a_ref, raw)

    # --------------------------------------------------------------------------
    # The 5 Selected Best Filters
    # --------------------------------------------------------------------------
    signals = {}
    signals['Current Firmware (EMA 16Hz)'] = filter_current_firmware(raw)
    signals['Hybrid Alpha-Beta + EMA'] = filter_hybrid_alpha_beta_ema(raw, dt_s=dt_s)
    signals['Butterworth 2nd (fc=35Hz)'] = filter_butterworth_biquad(raw, cutoff_hz=35.0, sample_rate_hz=fs)
    signals['Cascaded 2xEMA (a=0.10)'] = filter_cascaded_2x_ema(raw, alpha=0.10)
    signals['Butterworth 2nd (fc=20Hz)'] = filter_butterworth_biquad(raw, cutoff_hz=20.0, sample_rate_hz=fs)

    # Compute metrics, print table and C++ snippets
    results = compute_metrics(t, raw, signals, ref_ground_truth)
    print_ranking_table(results)
    print_cpp_code_snippets()

    # Plot single focused window with the 5 filter pairs + combined chart
    save_fig = args.save_path if args.save_path else "filter_top5_result.png"
    plot_pairs_grid(t, raw, tgt, results, save_path=save_fig)

    if not args.no_gui:
        print("[*] Displaying interactive plot window (close window to exit)...")
        plt.show()


if __name__ == '__main__':
    main()
