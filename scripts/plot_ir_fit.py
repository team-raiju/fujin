#!/usr/bin/env python3
"""
plot_ir_calibration.py

Reads IR distance sensor calibration data (CSV/semicolon-separated lines of
the form: Dist_mm, Sample, L, FL, FR, R) and, for each sensor, computes the
mean ADC response per distance. For every sensor it fits:

  1. Ideal Polynomial Degree 3:   d = poly3(raw / 1000)          (best possible fit, per sensor)
  2. Ideal Empirical Logarithmic: d = a / ln(raw + c) - b        (best possible fit, per sensor)

It also discovers a single shared "base" shape for each model across ALL
sensors passed in, on the assumption that different sensors (or the same
sensor on a different day / mounting angle / wall brightness) share the same
underlying response shape, just scaled and shifted differently:

  Polynomial Degree 3:    distance = base_poly3((raw * GAIN) / 1000) + OFFSET_MM
  Empirical Logarithmic:  distance = a / ln(raw + BASE_C) - b
                          (BASE_C is the shared shape term; a, b are cheap
                           to solve once BASE_C is fixed)

For each sensor, the per-sensor knobs (GAIN/OFFSET_MM, or a/b) are then
solved from just 2 reference points (default 60 mm and 150 mm) — mimicking
a quick field calibration on competition day — and the reconstructed curves
are compared against that sensor's own ideal fit, so you can see how good an
approximation 2 points actually give you.

The base_poly3 coefficients and BASE_C are meant to be hard-coded into
firmware once; each sensor then only needs its own GAIN/OFFSET_MM (or a/b),
recalibrated on the day from 2 quick measurements.

Pass one -s SENSOR FILE pair per sensor (one file per sensor).

Usage:
    # Single sensor (base curve degenerates to that sensor's own ideal fit)
    python plot_ir_calibration.py -s FR logs/ir/FR.txt

    # Discover a shared base curve from FR and R, then 2-point-calibrate each
    python plot_ir_calibration.py -s FR logs/ir/FR.txt -s R logs/ir/R.txt --min-dist 40

    # Use different reference distances for the 2-point calibration demo
    python plot_ir_calibration.py -s FR logs/ir/FR.txt -s R logs/ir/R.txt --ref1 60 --ref2 150
"""

import argparse
import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit, minimize_scalar, least_squares

SENSOR_NAMES = ['L', 'FL', 'FR', 'R']
SENSOR_COLUMN = {'L': 2, 'FL': 3, 'FR': 4, 'R': 5}  # index within a parsed line
SENSOR_LABELS = {
    'L': 'Left (L)',
    'FL': 'Front-Left (FL)',
    'FR': 'Front-Right (FR)',
    'R': 'Right (R)',
}

IR_MAX_DISTANCE_MM = 270.0  # Maximum sensor distance (mm)


# ==============================================================================
# PARSING & STATISTICS
# ==============================================================================

def parse_sensor_column(lines, sensor):
    """
    Parses calibration lines and extracts raw ADC readings for a single sensor.
    Expected line format: Dist_mm, Sample, L, FL, FR, R  (comma or semicolon separated)
    Returns: dict of distance_mm -> list of raw ADC readings
    """
    col_idx = SENSOR_COLUMN[sensor]
    data = {}

    for line in lines:
        line = line.strip()
        if not line:
            continue

        delimiter = ',' if ',' in line else (';' if ';' in line else None)
        if not delimiter:
            continue

        parts = [p.strip() for p in line.split(delimiter)]
        if len(parts) <= col_idx:
            continue

        try:
            dist_mm = float(parts[0])
            raw_val = float(parts[col_idx])
        except ValueError:
            continue

        data.setdefault(dist_mm, []).append(raw_val)

    return data


def compute_mean_response(data):
    """Returns sorted distances and the mean raw ADC reading at each distance."""
    distances = sorted(data.keys())
    means = np.array([np.mean(data[d]) for d in distances], dtype=np.float64)
    return distances, means


# ==============================================================================
# CURVE FITTING: Distance = f(ADC)
# ==============================================================================

def calculate_fit_metrics(y_true, y_pred):
    """Calculates R^2, RMSE, MAE, Max Absolute Error."""
    residuals = y_true - y_pred
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    ss_res = float(np.sum(residuals ** 2))
    return {
        'r2': float(1.0 - ss_res / ss_tot) if ss_tot > 0 else 0.0,
        'rmse': float(np.sqrt(np.mean(residuals ** 2))),
        'mae': float(np.mean(np.abs(residuals))),
        'max_err': float(np.max(np.abs(residuals))),
        'residuals': residuals,
    }


def fit_polynomial_degree3(dists, adcs):
    """Fits d = poly3(ADC / 1000). This is the 'ideal' per-sensor fit."""
    d = np.array(dists, dtype=np.float64)
    x_norm = adcs / 1000.0

    coeffs = np.polyfit(x_norm, d, 3)
    fn = lambda a: np.polyval(coeffs, np.asarray(a, dtype=np.float64) / 1000.0)
    pred = fn(adcs)

    return {
        'name': 'Polynomial Degree 3',
        'type': 'd = p3*x^3 + p2*x^2 + p1*x + p0  (x = ADC/1000)',
        'coeffs': coeffs,
        'fn': fn,
        'metrics': calculate_fit_metrics(d, pred),
        'color': '#1f77b4',
    }


def fit_empirical_log(dists, adcs):
    """Fits d = a / ln(raw + c) - b."""
    d = np.array(dists, dtype=np.float64)

    lower_c = -float(np.min(adcs)) + 2.0  # guarantees raw + c >= 2 so ln(raw + c) > 0

    def log_inv_fn(raw, a, b, c):
        return a / np.log(np.maximum(raw + c, 2.0)) - b

    popt, _ = curve_fit(
        log_inv_fn, adcs, d,
        p0=[4000.0, 400.0, 10.0],
        bounds=([0.0, -np.inf, lower_c], [np.inf, np.inf, 1e6]),
        maxfev=25000,
    )
    a_mm, b_mm, c_val = popt

    fn = lambda a: log_inv_fn(np.asarray(a, dtype=np.float64), a_mm, b_mm, c_val)
    pred = fn(adcs)

    return {
        'name': 'Empirical Logarithmic',
        'type': f'd = {a_mm:.4f} / ln(raw {"+" if c_val >= 0 else "-"} {abs(c_val):.4f}) '
                f'{"-" if b_mm >= 0 else "+"} {abs(b_mm):.4f}  [mm]',
        'params': {'a': a_mm, 'b': b_mm, 'c': c_val},
        'fn': fn,
        'metrics': calculate_fit_metrics(d, pred),
        'color': '#17becf',
    }


def fit_all_models(dists, adcs):
    """Fits both ideal candidate models. Returns a dict; entries missing on failure."""
    models = {}

    try:
        models['Polynomial Degree 3'] = fit_polynomial_degree3(dists, adcs)
    except Exception as e:
        print(f"[!] Warning: Polynomial Degree 3 fit failed: {e}")

    try:
        models['Empirical Logarithmic'] = fit_empirical_log(dists, adcs)
    except Exception as e:
        print(f"[!] Warning: Empirical Logarithmic fit failed: {e}")

    return models


# ==============================================================================
# BASE CURVE DISCOVERY & 2-POINT FIELD CALIBRATION
# ==============================================================================

def discover_base_curve(entries):
    """
    Jointly discovers one shared 'base' Polynomial Degree 3 shape, plus a
    per-sensor GAIN and OFFSET_MM, such that for every sensor i:

        distance = base_poly3((raw_i * GAIN_i) / 1000) + OFFSET_MM_i

    approximates that sensor's own full calibration data. The first sensor
    is used as the reference (GAIN=1, OFFSET_MM=0) to anchor the base
    curve's scale — otherwise GAIN and the polynomial coefficients could
    trade off against each other with no unique solution.

    Returns: base_p3 (ndarray of 4 coefficients)
    """
    combined_d = np.concatenate([e['d_fit'] for e in entries])
    combined_adc = np.concatenate([e['adc_fit'] for e in entries])
    base0 = np.polyfit(combined_adc / 1000.0, combined_d, 3)

    n = len(entries)
    if n == 1:
        return base0

    x0 = np.concatenate([base0, np.ones(n - 1), np.zeros(n - 1)])

    def unpack(x):
        base = x[:4]
        gains = np.concatenate([[1.0], x[4:4 + (n - 1)]])
        offsets = np.concatenate([[0.0], x[4 + (n - 1):4 + 2 * (n - 1)]])
        return base, gains, offsets

    def residuals(x):
        base, gains, offsets = unpack(x)
        res = []
        for e, gain, offset in zip(entries, gains, offsets):
            pred = np.polyval(base, (e['adc_fit'] * gain) / 1000.0) + offset
            res.append(e['d_fit'] - pred)
        return np.concatenate(res)

    result = least_squares(residuals, x0, method='lm', max_nfev=20000)
    base, _, _ = unpack(result.x)
    return base


def solve_poly_gain_offset(base_p3, d1, r1, d2, r2):
    """
    Solves for GAIN and OFFSET_MM so that:
        distance = base_poly3((raw * GAIN) / 1000) + OFFSET_MM
    passes through two known reference points (d1, r1) and (d2, r2).

    GAIN is found by matching the *shape* between the two points (the
    difference the base curve predicts between them should equal the true
    distance difference); OFFSET_MM then shifts the curve to match absolute
    distance at point 1.
    """
    def poly_dist(raw, gain):
        return np.polyval(base_p3, (raw * gain) / 1000.0)

    def cost(gain):
        return ((poly_dist(r2, gain) - poly_dist(r1, gain)) - (d2 - d1)) ** 2

    res = minimize_scalar(cost, bounds=(0.3, 3.0), method='bounded')
    gain = float(res.x)
    offset = float(d1 - poly_dist(r1, gain))
    return gain, offset


def build_two_point_model(base_p3, gain, offset, dists, adcs, ref_points):
    """Builds a model dict for the base curve tuned via GAIN/OFFSET from 2 reference points."""
    d = np.array(dists, dtype=np.float64)
    fn = lambda a: np.polyval(base_p3, (np.asarray(a, dtype=np.float64) * gain) / 1000.0) + offset
    pred = fn(adcs)

    return {
        'name': 'Base Curve (2-Point Tuned)',
        'type': f'd = base_poly3((raw * {gain:.4f}) / 1000) {offset:+.2f} mm',
        'gain': gain,
        'offset': offset,
        'fn': fn,
        'metrics': calculate_fit_metrics(d, pred),
        'color': '#2ca02c',
        'ref_points': ref_points,
    }


# ------------------------------------------------------------------------------
# Empirical Logarithmic base curve: d = a / ln(raw + c) - b
#
# For a *fixed* c, this formula is linear in a and b (let u = 1/ln(raw + c),
# then d = a*u - b). That means c is the only real "shape" parameter — once
# it's fixed, a and b drop out of a simple 2-point solve (or a linear
# regression, if more points are available). So the base curve here is really
# just a shared BASE_C; a and b are always (re)computed from whatever real
# data is available for that sensor.
# ------------------------------------------------------------------------------

def _log_u(raw, c_val):
    """u = 1 / ln(raw + c), the linearizing substitution for the log model."""
    return 1.0 / np.log(np.maximum(np.asarray(raw, dtype=np.float64) + c_val, 2.0))


def discover_base_log_curve(entries):
    """
    Jointly discovers one shared BASE_C for the empirical logarithmic model,
    plus nominal BASE_A/BASE_B fit on the pooled data of all sensors (a
    generic starting curve before any per-sensor calibration is applied).

    For a given c, each sensor's best-fit a/b is just a linear regression in
    u = 1/ln(raw + c), so BASE_C is found by a 1-D search that, at each
    candidate c, fits every sensor's own best a/b and sums the residuals.

    Returns: (base_a, base_b, base_c)
    """
    all_adc = np.concatenate([e['adc_fit'] for e in entries])
    lower_c = -float(np.min(all_adc)) + 2.0  # guarantees raw + c >= 2 for every sensor

    def total_sse(c_val):
        sse = 0.0
        for e in entries:
            u = _log_u(e['adc_fit'], c_val)
            slope, intercept = np.polyfit(u, e['d_fit'], 1)  # d = slope*u + intercept
            pred = slope * u + intercept
            sse += float(np.sum((e['d_fit'] - pred) ** 2))
        return sse

    if len(entries) == 1:
        c0 = lower_c + 400.0
    else:
        # Seed the search from each sensor's own ideal c, averaged
        c0 = lower_c + 400.0

    res = minimize_scalar(total_sse, bounds=(lower_c, lower_c + 5000.0), method='bounded')
    base_c = float(res.x)

    # Nominal base a/b: pool ALL sensors' data together and fit once, just to
    # report a complete "generic" starting curve alongside BASE_C.
    all_d = np.concatenate([e['d_fit'] for e in entries])
    u_all = _log_u(all_adc, base_c)
    slope, intercept = np.polyfit(u_all, all_d, 1)
    base_a, base_b = float(slope), float(-intercept)

    return base_a, base_b, base_c


def solve_log_ab_two_points(base_c, d1, r1, d2, r2):
    """
    Solves for a and b (keeping c fixed at the shared base value) so that
        distance = a / ln(raw + base_c) - b
    passes exactly through two known reference points (d1, r1) and (d2, r2).
    This is a closed-form 2-equation/2-unknown solve (linear in a, b once c
    is fixed).
    """
    u1 = float(_log_u(r1, base_c))
    u2 = float(_log_u(r2, base_c))
    denom = u2 - u1
    if abs(denom) < 1e-12:
        a_calc = 4000.0
    else:
        a_calc = (d2 - d1) / denom
    b_calc = a_calc * u1 - d1
    return float(a_calc), float(b_calc)


def build_two_point_log_model(c_val, a, b, dists, adcs, ref_points,
                               name='Base Log (2-Point Tuned)', color='#9467bd'):
    """Builds a model dict for a log curve (fixed c, tuned a/b) from 2 reference points."""
    d = np.array(dists, dtype=np.float64)
    fn = lambda raw: a / np.log(np.maximum(np.asarray(raw, dtype=np.float64) + c_val, 2.0)) - b
    pred = fn(adcs)

    return {
        'name': name,
        'type': f'd = {a:.4f} / ln(raw {"+" if c_val >= 0 else "-"} {abs(c_val):.4f}) '
                f'{"-" if b >= 0 else "+"} {abs(b):.4f}  [mm]',
        'a': a,
        'b': b,
        'c': c_val,
        'fn': fn,
        'metrics': calculate_fit_metrics(d, pred),
        'color': color,
        'ref_points': ref_points,
    }


# ==============================================================================
# REPORTING
# ==============================================================================

def print_fit_results(models, label, min_dist, max_dist):
    """Prints a ranked comparison table plus detailed parameters for each model."""
    ranked = sorted(models.items(), key=lambda item: item[1]['metrics']['rmse'])

    print("\n" + "=" * 94)
    print(f" CURVE FITTING & MODEL COMPARISON: Distance = f(ADC) for {label.upper()}")
    print(f" Fitting Range: {min_dist:.0f} mm to {max_dist:.0f} mm")
    print("=" * 94)
    print(f"{'Rank':<5} | {'Model Name':<24} | {'R² Score':<9} | {'RMSE (mm)':<10} | {'MAE (mm)':<9} | {'Max Err (mm)':<12}")
    print("-" * 94)
    for i, (key, m) in enumerate(ranked, 1):
        met = m['metrics']
        print(f"{i:<5} | {m['name']:<24} | {met['r2']:<9.4f} | {met['rmse']:<10.2f} | {met['mae']:<9.2f} | {met['max_err']:<12.2f}")
    print("=" * 94)

    if 'Polynomial Degree 3' in models:
        m = models['Polynomial Degree 3']
        coeffs_str = ", ".join([f"{c:+.4f}" for c in m['coeffs']])
        print("\n--- POLYNOMIAL DEGREE 3 PARAMETERS ---")
        print(f"    Formula : {m['type']}")
        print(f"    Coeffs  : [{coeffs_str}]")
        print(f"    R²={m['metrics']['r2']:.4f}  RMSE={m['metrics']['rmse']:.2f} mm  MAE={m['metrics']['mae']:.2f} mm")

    if 'Empirical Logarithmic' in models:
        m = models['Empirical Logarithmic']
        p = m['params']
        print("\n" + "=" * 94)
        print(" EMPIRICAL LOGARITHMIC MODEL PARAMETERS: distance = a / ln(raw + c) - b")
        print("=" * 94)
        print(f"    a = {p['a']:.6f}")
        print(f"    b = {p['b']:.6f}")
        print(f"    c = {p['c']:.6f}")
        print(f"    Formula   : {m['type']}")
        print(f"    R²={m['metrics']['r2']:.4f}  RMSE={m['metrics']['rmse']:.2f} mm  "
              f"MAE={m['metrics']['mae']:.2f} mm  Max Err={m['metrics']['max_err']:.2f} mm")
        print("=" * 94)


def print_base_curve_summary(base_p3, sensor_labels):
    coeffs_str = ", ".join([f"{c:+.4f}" for c in base_p3])
    print("\n" + "=" * 94)
    print(" SHARED BASE POLYNOMIAL DEGREE 3 CURVE")
    print(f" Discovered from: {' + '.join(sensor_labels)}")
    print("=" * 94)
    print(" Formula : d = base_poly3((raw * GAIN) / 1000) + OFFSET_MM")
    print(f" Coeffs  : [{coeffs_str}]")
    print(" Hard-code these 4 coefficients into firmware once. Each sensor then only")
    print(" needs its own GAIN and OFFSET_MM, calibrated from 2 reference points below.")
    print("=" * 94)


def print_base_log_curve_summary(base_a, base_b, base_c, sensor_labels):
    print("\n" + "=" * 94)
    print(" SHARED BASE EMPIRICAL LOGARITHMIC CURVE")
    print(f" Discovered from: {' + '.join(sensor_labels)}")
    print("=" * 94)
    print(" Formula : d = a / ln(raw + BASE_C) - b")
    print(f" BASE_C  = {base_c:.4f}   (shared shape term — hard-code this into firmware)")
    print(f" Nominal a, b (generic starting curve, pre-calibration): a = {base_a:.4f}, b = {base_b:.4f}")
    print(" a and b are cheap to solve from 2 real reference points per sensor (see below);")
    print(" only BASE_C needs to be shared/fixed across sensors.")
    print("=" * 94)


def print_two_point_calibration(sensor, ref_points, gain, offset, ideal_metrics, two_point_metrics):
    (d1, r1), (d2, r2) = ref_points
    label = SENSOR_LABELS.get(sensor, sensor)
    print("\n" + "-" * 94)
    print(f" TWO-POINT FIELD CALIBRATION (Polynomial Degree 3): {label}")
    print("-" * 94)
    print(f"    Reference Point 1 : {d1:.0f} mm  ->  raw ADC {r1:.1f}")
    print(f"    Reference Point 2 : {d2:.0f} mm  ->  raw ADC {r2:.1f}")
    print(f"    {sensor}_GAIN       = {gain:.6f}")
    print(f"    {sensor}_OFFSET_MM  = {offset:+.4f}")
    print(f"    Reconstructed-curve accuracy : R²={two_point_metrics['r2']:.4f}  "
          f"RMSE={two_point_metrics['rmse']:.2f} mm  MAE={two_point_metrics['mae']:.2f} mm  "
          f"Max Err={two_point_metrics['max_err']:.2f} mm")
    print(f"    (Ideal per-sensor fit, for comparison : R²={ideal_metrics['r2']:.4f}  "
          f"RMSE={ideal_metrics['rmse']:.2f} mm  MAE={ideal_metrics['mae']:.2f} mm  "
          f"Max Err={ideal_metrics['max_err']:.2f} mm)")
    print("-" * 94)


def print_two_point_log_calibration(sensor, ref_points, a, b, c, ideal_metrics, two_point_metrics,
                                     heading="TWO-POINT FIELD CALIBRATION (Empirical Logarithmic)",
                                     c_note="(shared)"):
    (d1, r1), (d2, r2) = ref_points
    label = SENSOR_LABELS.get(sensor, sensor)
    print("\n" + "-" * 94)
    print(f" {heading}: {label}")
    print("-" * 94)
    print(f"    Reference Point 1 : {d1:.0f} mm  ->  raw ADC {r1:.1f}")
    print(f"    Reference Point 2 : {d2:.0f} mm  ->  raw ADC {r2:.1f}")
    print(f"    {sensor}_A = {a:.4f}   {sensor}_B = {b:.4f}   {sensor}_C {c_note} = {c:.4f}")
    print(f"    Reconstructed-curve accuracy : R²={two_point_metrics['r2']:.4f}  "
          f"RMSE={two_point_metrics['rmse']:.2f} mm  MAE={two_point_metrics['mae']:.2f} mm  "
          f"Max Err={two_point_metrics['max_err']:.2f} mm")
    print(f"    (Ideal per-sensor fit, for comparison : R²={ideal_metrics['r2']:.4f}  "
          f"RMSE={ideal_metrics['rmse']:.2f} mm  MAE={ideal_metrics['mae']:.2f} mm  "
          f"Max Err={ideal_metrics['max_err']:.2f} mm)")
    print("-" * 94)


# ==============================================================================
# PLOTTING
# ==============================================================================

def plot_fitted_models(dists, adcs, ordered_models, label, title_suffix=""):
    """
    Plots estimate-vs-real-data and residuals for a list of (name, model)
    pairs, one column per model. Models with a 'ref_points' entry get their
    reference points marked on the curve subplot.
    """
    d = np.array(dists)
    adc = np.array(adcs)

    num_cols = len(ordered_models)
    if num_cols == 0:
        print("[!] No models to plot.")
        return

    fig, axs = plt.subplots(2, num_cols, figsize=(5.5 * num_cols, 9), sharey='row')
    fig.suptitle(f'Distance Estimation Models for {label} {title_suffix}', fontsize=16, fontweight='bold')
    if num_cols == 1:
        axs = np.array([[axs[0]], [axs[1]]])

    adc_grid = np.linspace(np.min(adc), np.max(adc), 300)
    all_res = np.concatenate([m['metrics']['residuals'] for _, m in ordered_models])
    res_min, res_max = float(np.min(all_res)) - 1.5, float(np.max(all_res)) + 1.5

    for col, (name, m) in enumerate(ordered_models):
        color = m['color']
        met = m['metrics']

        ax_curve = axs[0, col]
        ax_curve.plot(d, adc, 'ko', markersize=5, zorder=5, label='Calibrated Data')
        try:
            d_curve = np.clip(m['fn'](adc_grid), np.min(d) - 10, np.max(d) + 15)
            ax_curve.plot(d_curve, adc_grid, '-', color=color, linewidth=2.4, label='Fitted Curve')
        except Exception:
            pass

        if 'ref_points' in m:
            ref_d = [p[0] for p in m['ref_points']]
            ref_r = [p[1] for p in m['ref_points']]
            ax_curve.scatter(ref_d, ref_r, marker='D', s=90, color='red', zorder=6,
                              edgecolor='black', linewidth=1, label='Reference Points')

        ax_curve.set_title(name, fontsize=12, fontweight='semibold')
        ax_curve.set_xlabel('Distance (mm)', fontsize=10)
        if col == 0:
            ax_curve.set_ylabel('Raw ADC Reading', fontsize=11)
        ax_curve.grid(True, linestyle='--', alpha=0.6)
        ax_curve.legend(loc='upper right', fontsize=8)

        stats_lines = [f"R²: {met['r2']:.4f}", f"MAE: {met['mae']:.2f} mm",
                       f"RMSE: {met['rmse']:.2f} mm", f"Max Err: {met['max_err']:.1f} mm"]
        if 'gain' in m:
            stats_lines = [f"GAIN: {m['gain']:.4f}", f"OFFSET: {m['offset']:+.2f} mm",
                           "----------------"] + stats_lines
        elif 'a' in m and 'b' in m and 'c' in m:
            stats_lines = [f"a: {m['a']:.4f}", f"b: {m['b']:.4f}", f"c: {m['c']:.4f}",
                           "----------------"] + stats_lines
        stats_text = "\n".join(stats_lines)
        ax_curve.text(0.05, 0.08, stats_text, transform=ax_curve.transAxes, fontsize=8.5,
                      verticalalignment='bottom',
                      bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=color, alpha=0.9))

        ax_res = axs[1, col]
        ax_res.axhline(0, color='black', linestyle='--', linewidth=1.2, alpha=0.8)
        ax_res.plot(d, met['residuals'], 'o-', color=color, linewidth=1.8, markersize=5, label='Error')
        ax_res.fill_between(d, 0, met['residuals'], color=color, alpha=0.15)
        ax_res.set_title(f'Prediction Error (mm)\n{name}', fontsize=12, fontweight='semibold')
        ax_res.set_xlabel('True Distance (mm)', fontsize=10)
        if col == 0:
            ax_res.set_ylabel('Error: (Pred - Real) mm', fontsize=11)
        ax_res.set_ylim(res_min, res_max)
        ax_res.grid(True, linestyle='--', alpha=0.6)
        ax_res.legend(loc='upper left', fontsize=8)

    plt.subplots_adjust(top=0.90, bottom=0.08, left=0.06, right=0.98, hspace=0.32, wspace=0.18)
    plt.show()


# ==============================================================================
# MAIN
# ==============================================================================

def load_sensor_data(sensor, filepath, min_dist_arg, max_dist_arg):
    """
    Parses one sensor's file and computes its mean ADC response per distance.
    Returns a dict with both the full (unfiltered) response curve and the
    range-filtered (d_fit, adc_fit) used for fitting, or None on failure.
    """
    if not os.path.exists(filepath):
        print(f"[!] Error: File '{filepath}' not found. Skipping sensor {sensor}.")
        return None

    print(f"\n[*] Reading calibration data for {sensor} from: {filepath}")
    with open(filepath, "r") as f:
        lines = f.readlines()

    data = parse_sensor_column(lines, sensor)
    if not data:
        print(f"[!] No valid '{sensor}' calibration data found in '{filepath}'. "
              f"Expected lines formatted as: Dist_mm,Sample,L,FL,FR,R")
        return None

    distances, means = compute_mean_response(data)
    total_samples = sum(len(v) for v in data.values())
    print(f"[*] Parsed {total_samples} samples across {len(distances)} distances: {distances} mm")

    # Default fitting range: from peak response distance to max measured distance
    peak_idx = int(np.argmax(means))
    default_min_dist = distances[peak_idx]
    fit_min_dist = min_dist_arg if min_dist_arg is not None else default_min_dist
    fit_max_dist = max_dist_arg if max_dist_arg is not None else min(max(distances), IR_MAX_DISTANCE_MM)

    mask = [(fit_min_dist <= d <= fit_max_dist) for d in distances]
    d_fit = np.array(distances)[mask]
    adc_fit = means[mask]

    if len(d_fit) < 4:
        print(f"[!] Warning: Not enough points in range [{fit_min_dist}, {fit_max_dist}] mm "
              f"to fit {sensor}. Skipping.")
        return None

    return {
        'sensor': sensor,
        'filepath': filepath,
        'distances': distances,  # full, unfiltered (for reference-point interpolation)
        'means': means,          # full, unfiltered
        'd_fit': d_fit,
        'adc_fit': adc_fit,
        'min_dist': fit_min_dist,
        'max_dist': fit_max_dist,
    }


def parse_sensor_file_args(sensor_file_pairs):
    """Validates and de-duplicates the list of (SENSOR, FILE) pairs from the CLI."""
    seen = {}
    for sensor, filepath in sensor_file_pairs:
        sensor = sensor.strip().upper()
        if sensor not in SENSOR_NAMES:
            print(f"[!] Error: Unknown sensor '{sensor}'. Valid sensors: {', '.join(SENSOR_NAMES)}.")
            sys.exit(1)
        if sensor in seen:
            print(f"[!] Warning: Sensor '{sensor}' given more than once; only one file per "
                  f"sensor is supported. Keeping the first ('{seen[sensor]}'), ignoring '{filepath}'.")
            continue
        seen[sensor] = filepath
    return list(seen.items())


def main():
    parser = argparse.ArgumentParser(
        description="Fit IR distance sensor calibration curves, discover a shared base "
                    "Polynomial Degree 3 curve across sensors, and demonstrate reconstructing "
                    "each sensor's curve from that base plus 2 reference points."
    )
    parser.add_argument(
        "-s", "--sensor",
        dest="sensor_files",
        action="append",
        nargs=2,
        metavar=("SENSOR", "FILE"),
        required=True,
        help="A sensor (L, FL, FR, or R) and its calibration log file, one file per sensor. "
             "Repeat -s for each sensor, e.g. -s FR logs/FR.txt -s R logs/R.txt"
    )
    parser.add_argument(
        "--min-dist",
        type=float,
        default=None,
        help="Minimum distance (mm) to include for curve fitting (defaults to each "
             "sensor's peak response distance)."
    )
    parser.add_argument(
        "--max-dist",
        type=float,
        default=IR_MAX_DISTANCE_MM,
        help=f"Maximum distance (mm) to include for curve fitting (default: {IR_MAX_DISTANCE_MM:.0f})."
    )
    parser.add_argument(
        "--ref1",
        type=float,
        default=60.0,
        help="First reference distance (mm) for the 2-point field-calibration demo (default: 60)."
    )
    parser.add_argument(
        "--ref2",
        type=float,
        default=150.0,
        help="Second reference distance (mm) for the 2-point field-calibration demo (default: 150)."
    )
    args = parser.parse_args()

    sensor_files = parse_sensor_file_args(args.sensor_files)

    entries = []
    for sensor, filepath in sensor_files:
        entry = load_sensor_data(sensor, filepath, args.min_dist, args.max_dist)
        if entry is not None:
            entries.append(entry)

    if not entries:
        print("[!] No sensor data could be loaded/fit. Exiting.")
        sys.exit(1)

    # 1. Ideal per-sensor fits: the best possible individual curve for each sensor,
    #    used as the benchmark to judge the 2-point reconstruction against.
    ideal_models = {}
    for e in entries:
        models = fit_all_models(e['d_fit'], e['adc_fit'])
        if not models or 'Polynomial Degree 3' not in models:
            print(f"[!] No usable fit for {e['sensor']}. Skipping sensor.")
            continue
        ideal_models[e['sensor']] = models
        print_fit_results(models, SENSOR_LABELS.get(e['sensor'], e['sensor']), e['min_dist'], e['max_dist'])

    entries = [e for e in entries if e['sensor'] in ideal_models]
    if not entries:
        print("[!] No models could be fitted for any sensor. Exiting.")
        sys.exit(1)

    # 2. Discover a single shared base Polynomial Degree 3 shape across all sensors.
    base_p3 = discover_base_curve(entries)
    print_base_curve_summary(base_p3, [SENSOR_LABELS.get(e['sensor'], e['sensor']) for e in entries])

    # 2b. Discover a shared base_c for the empirical logarithmic model across all sensors.
    base_log_a, base_log_b, base_log_c = discover_base_log_curve(entries)
    print_base_log_curve_summary(base_log_a, base_log_b, base_log_c,
                                  [SENSOR_LABELS.get(e['sensor'], e['sensor']) for e in entries])

    # 3. For each sensor, calibrate from just 2 reference points and compare the
    #    reconstructed curves (poly3 and log) against the sensor's own ideal fit.
    for e in entries:
        sensor = e['sensor']
        r1 = float(np.interp(args.ref1, e['distances'], e['means']))
        r2 = float(np.interp(args.ref2, e['distances'], e['means']))
        ref_points = [(args.ref1, r1), (args.ref2, r2)]

        gain, offset = solve_poly_gain_offset(base_p3, args.ref1, r1, args.ref2, r2)
        two_point_poly = build_two_point_model(base_p3, gain, offset, e['d_fit'], e['adc_fit'], ref_points)

        log_a, log_b = solve_log_ab_two_points(base_log_c, args.ref1, r1, args.ref2, r2)
        two_point_log = build_two_point_log_model(base_log_c, log_a, log_b, e['d_fit'], e['adc_fit'], ref_points)

        ideal_poly3 = ideal_models[sensor]['Polynomial Degree 3']
        print_two_point_calibration(sensor, ref_points, gain, offset,
                                     ideal_poly3['metrics'], two_point_poly['metrics'])

        if 'Empirical Logarithmic' in ideal_models[sensor]:
            ideal_log = ideal_models[sensor]['Empirical Logarithmic']
            print_two_point_log_calibration(sensor, ref_points, log_a, log_b, base_log_c,
                                             ideal_log['metrics'], two_point_log['metrics'])

            # "Own curve" scenario: firmware already has this sensor's own individual
            # log curve (its own c, baked in from its ideal fit) — on competition day,
            # 2 points are used to re-solve only a and b for that same curve.
            own_c = ideal_log['params']['c']
            own_a, own_b = solve_log_ab_two_points(own_c, args.ref1, r1, args.ref2, r2)
            own_two_point_log = build_two_point_log_model(
                own_c, own_a, own_b, e['d_fit'], e['adc_fit'], ref_points,
                name='Own Log (2-Point Tuned)', color='#d62728',
            )
            print_two_point_log_calibration(
                sensor, ref_points, own_a, own_b, own_c,
                ideal_log['metrics'], own_two_point_log['metrics'],
                heading="TWO-POINT FIELD CALIBRATION (Own Individual Log Curve)",
                c_note="(own, fixed)",
            )
        else:
            own_two_point_log = None

        ordered = [
            ('Ideal Polynomial Degree 3', ideal_models[sensor].get('Polynomial Degree 3')),
            ('Ideal Empirical Logarithmic', ideal_models[sensor].get('Empirical Logarithmic')),
            ('Base Poly3 (2-Point Tuned)', two_point_poly),
            ('Base Log (2-Point Tuned)', two_point_log),
            ('Own Log (2-Point Tuned)', own_two_point_log),
        ]
        ordered = [(name, m) for name, m in ordered if m is not None]

        plot_fitted_models(
            e['d_fit'], e['adc_fit'], ordered, SENSOR_LABELS.get(sensor, sensor),
            title_suffix=f"({os.path.basename(e['filepath'])})",
        )


if __name__ == "__main__":
    main()