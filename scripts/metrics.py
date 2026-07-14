#!/usr/bin/env python3
import os
import math
import sys
import argparse

def parse_log_file(file_path):
    """Reads a log file and returns Times, Actual, and Target angular velocities."""
    with open(file_path, 'r') as f:
        lines = f.readlines()

    if len(lines) <= 1:
        return None

    # Index 0: timestamp
    # Index 3: ang_vel_act, Index 4: ang_vel_tgt
    times = []
    act_angular_vel = []
    tgt_angular_vel = []

    for line in lines[1:]:
        line = line.strip()
        if not line: 
            continue
        try:
            fields = [float(x) for x in line.split(';')]
            if len(fields) >= 5:
                times.append(fields[0])
                act_angular_vel.append(fields[3])
                tgt_angular_vel.append(fields[4])
        except ValueError:
            continue

    if not act_angular_vel:
        return None

    return times, act_angular_vel, tgt_angular_vel

def calculate_metrics(times, act, tgt, start_time, end_time):
    """Calculates evaluation statistics and integrates angles over the specified time window."""
    n = len(act)
    if n == 0:
        return None

    errors = [t - a for a, t in zip(act, tgt)]
    
    # Mean Error (Bias)
    mean_error = sum(errors) / n
    
    # Mean Absolute Error (MAE)
    mae = sum(abs(e) for e in errors) / n
    
    # Root Mean Squared Error (RMSE)
    mse = sum(e ** 2 for e in errors) / n
    rmse = math.sqrt(mse)
    
    # Standard Deviation of Error
    variance = sum((e - mean_error) ** 2 for e in errors) / n
    std_dev = math.sqrt(variance)
    
    # Maximum absolute error spike
    max_error = max(abs(e) for e in errors)

    # Calculate Total Turn Angle via Numerical Integration
    total_act_angle = 0.0
    total_tgt_angle = 0.0
    
    for i in range(1, n):
        # Check if the current time falls within the requested window
        if start_time <= times[i] <= end_time:   
            dt = times[i] - times[i-1]

            dt = dt / 1000.0
            total_act_angle += act[i] * dt
            total_tgt_angle += tgt[i] * dt

    return {
        "samples": n,
        "mean_error": mean_error,
        "mae": mae,
        "rmse": rmse,
        "std_dev": std_dev,
        "max_error": max_error,
        "total_act_angle": total_act_angle,
        "total_tgt_angle": total_tgt_angle
    }

def analyze_logs_folder(folder_path, start_time, end_time):
    """Walks through the logs directory and summarizes control performance."""
    if not os.path.exists(folder_path):
        print(f"Error: The target directory '{folder_path}' does not exist.")
        sys.exit(1)

    results = []

    # Recursively look for any files inside the logs folder
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith(".txt") or file.endswith(".log"):
                file_path = os.path.join(root, file)
                try:
                    data = parse_log_file(file_path)
                    if data:
                        times, act, tgt = data
                        metrics = calculate_metrics(times, act, tgt, start_time, end_time)
                        if metrics:
                            metrics["filename"] = os.path.relpath(file_path, folder_path)
                            results.append(metrics)
                except Exception as e:
                    print(f"Warning: Failed to process {file_path}. Error: {e}")

    if not results:
        print(f"No valid log files containing numeric data points found in '{folder_path}'.")
        return

    # Sort results by Mean Absolute Error (MAE) - lower/better first
    results.sort(key=lambda x: x["mae"])

    # Print Summary Table
    print("\n" + "="*125)
    print(f"{'ANGULAR PID CONTROLLER PERFORMANCE REPORT':^125}")
    print(f"{f'Integration Window: {start_time} to {end_time}':^125}")
    print("="*125)
    
    header_format = "{:<30} | {:<8} | {:<10} | {:<10} | {:<10} | {:<10} | {:<12} | {:<12}"
    row_format =    "{:<30} | {:<8} | {:<10.4f} | {:<10.4f} | {:<10.4f} | {:<10.4f} | {:<12.4f} | {:<12.4f}"
    
    print(header_format.format("Log File (Relative to /logs)", "Samples", "MAE ↓", "RMSE", "Std Dev", "Max Error", "Tot Tgt Ang", "Tot Act Ang"))
    print("-" * 125)
    
    for r in results:
        print(row_format.format(
            r["filename"][-30:], # Truncate long paths for view preservation
            r["samples"],
            r["mae"],
            r["rmse"],
            r["std_dev"],
            r["max_error"],
            math.degrees(r["total_tgt_angle"]),
            math.degrees(r["total_act_angle"])
        ))
    print("="*125)
    print("Note: The table is ordered by Mean Absolute Error (MAE) ascending. Top files showed best overall tracking.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze angular PID controller log metrics.")
    parser.add_argument("folder", nargs="?", default="logs", help="Target logs directory (default: 'logs')")
    # Using float so it works with either seconds or milliseconds depending on the log data
    parser.add_argument("--start", type=float, default=0.0, help="Integration start window time (default: 0.0)")
    parser.add_argument("--end", type=float, default=10000.0, help="Integration end window time (default: 10000.0)")
    
    args = parser.parse_args()
    
    analyze_logs_folder(args.folder, args.start, args.end)