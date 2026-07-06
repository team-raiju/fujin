#!/usr/bin/env python3
import os
import math
import sys

def parse_log_file(file_path):
    """Reads a log file and returns the Actual and Target angular velocities."""
    with open(file_path, 'r') as f:
        lines = f.readlines()

    if len(lines) <= 1:
        return None

    # Determine positions based on your standard log index mappings
    # Both control and legacy modes place angular metrics at index 3 and 4
    # Index 3: ang_vel_act, Index 4: ang_vel_tgt
    act_angular_vel = []
    tgt_angular_vel = []

    for line in lines[1:]:
        line = line.strip()
        if not line: 
            continue
        try:
            fields = [float(x) for x in line.split(';')]
            if len(fields) >= 5:
                act_angular_vel.append(fields[3])
                tgt_angular_vel.append(fields[4])
        except ValueError:
            continue

    if not act_angular_vel:
        return None

    return act_angular_vel, tgt_angular_vel

def calculate_metrics(act, tgt):
    """Calculates evaluation statistics for the control loop quality."""
    n = len(act)
    if n == 0:
        return None

    errors = [t - a for a, t in zip(act, tgt)]
    
    # Mean Error (Bias)
    mean_error = sum(errors) / n
    
    # Mean Absolute Error (MAE) - overall magnitude of error
    mae = sum(abs(e) for e in errors) / n
    
    # Root Mean Squared Error (RMSE) - penalizes larger deviations heavily
    mse = sum(e ** 2 for e in errors) / n
    rmse = math.sqrt(mse)
    
    # Standard Deviation of Error
    variance = sum((e - mean_error) ** 2 for e in errors) / n
    std_dev = math.sqrt(variance)
    
    # Maximum absolute error spike
    max_error = max(abs(e) for e in errors)

    return {
        "samples": n,
        "mean_error": mean_error,
        "mae": mae,
        "rmse": rmse,
        "std_dev": std_dev,
        "max_error": max_error
    }

def analyze_logs_folder(folder_path="logs"):
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
                        act, tgt = data
                        metrics = calculate_metrics(act, tgt)
                        if metrics:
                            # Keep track of relative path for cleaner printing
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
    print("\n" + "="*95)
    print(f"{'ANGULAR PID CONTROLLER PERFORMANCE REPORT':^95}")
    print("="*95)
    header_format = "{:<30} | {:<8} | {:<10} | {:<10} | {:<10} | {:<10}"
    row_format =    "{:<30} | {:<8} | {:<10.4f} | {:<10.4f} | {:<10.4f} | {:<10.4f}"
    
    print(header_format.format("Log File (Relative to /logs)", "Samples", "MAE ↓", "RMSE", "Std Dev", "Max Error"))
    print("-" * 95)
    
    for r in results:
        print(row_format.format(
            r["filename"][-30:], # Truncate long paths for view preservation
            r["samples"],
            r["mae"],
            r["rmse"],
            r["std_dev"],
            r["max_error"]
        ))
    print("="*95)
    print("Note: The table is ordered by Mean Absolute Error (MAE) ascending. Top files showed best overall tracking.")

if __name__ == "__main__":
    # Check if user passed a custom logs directory argument
    target_dir = sys.argv[1] if len(sys.argv) > 1 else "logs"
    analyze_logs_folder(target_dir)