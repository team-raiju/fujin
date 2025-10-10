import os
import math
import matplotlib.pyplot as plt
from statistics import mean, stdev

# --- Configuration ---
kp = "0.60"
ki = "0.040_2"
filter = "fujin_fan_25"
num = "1"

# --- Helper Functions for Position Calculation ---
def get_shortest_delta_angle(a, b):
    """Calculates the shortest angle difference between a and b, handling wrap-around."""
    delta = a - b
    if delta > math.pi:
        delta -= 2 * math.pi
    elif delta < -math.pi:
        delta += 2 * math.pi
    return delta

def limit_angle_minus_pi_pi(angle):
    """Limits an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle

# --- Plotting Functions ---

def calculate_average_error(actual, target):
    total_error = sum(abs(a - t) for a, t in zip(actual, target))
    num_samples = len(actual)
    return total_error / num_samples

def plot_velocities(file_path):
    time, linear_velocity1, linear_velocity2, angular_velocity1, angular_velocity2, acceleration = [], [], [], [], [], []
    with open(file_path, 'r') as file:
        for line in file:
            fields = line.strip().split(';')
            time.append(float(fields[0]))
            linear_velocity1.append(float(fields[1]))
            linear_velocity2.append(float(fields[2]))
            angular_velocity1.append(float(fields[3]))
            angular_velocity2.append(float(fields[4]))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

    linear_error = calculate_average_error(linear_velocity1, linear_velocity2)
    angular_error = calculate_average_error(angular_velocity1, angular_velocity2)

    print(f"Total Linear Velocity Error: {linear_error:.5f}")
    print(f"Total Angular Velocity Error: {angular_error:.5f}")

    # Plot linear velocities
    ax1.plot(time, linear_velocity1, label='Linear Velocity')
    ax1.plot(time, linear_velocity2, label='Target Linear Velocity')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('m/s')
    ax1.set_title('Linear Velocities')
    ax1.legend()
    ax1.grid(True)

    # Plot angular velocities
    ax2.plot(time, angular_velocity1, label='Angular Velocity')
    ax2.plot(time, angular_velocity2, label='Target Angular Velocity')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('rad/s')
    ax2.set_title('Angular Velocities')
    ax2.legend()
    ax2.grid(True)

    # Calculate and print acceleration
    print("\n--- Acceleration (m/s²) ---")
    for i in range(0, len(linear_velocity1) - 50, 50):
        accel = (linear_velocity1[i+50] - linear_velocity1[i]) / 0.05
        acceleration.append(accel)
        print(f"t({i}-{i+50}ms): {linear_velocity1[i]:.2f} -> {linear_velocity1[i+50]:.2f} = {accel:.2f} m/s²")
    print("--------------------------\n")

    plt.tight_layout()
    plt.show()

def plot_pwm_and_bat(file_path):
    time, pwm_left, pwm_right, battery = [], [], [], []
    with open(file_path, 'r') as file:
        for line in file:
            fields = line.strip().split(';')
            time.append(float(fields[0]))
            pwm_left.append(float(fields[5]))
            pwm_right.append(float(fields[6]))
            battery.append(float(fields[7]))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

    # Plot PWM signals
    ax1.plot(time, pwm_left, label='PWM Left')
    ax1.plot(time, pwm_right, label='PWM Right')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('0-1000')
    ax1.set_title('PWM Signals')
    ax1.legend()
    ax1.grid(True)

    # Plot battery voltage
    ax2.plot(time, battery, label='Battery Voltage')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('mV')
    ax2.set_title('Battery Voltage')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

def plot_distance(file_path):
    time, distance = [], []
    with open(file_path, 'r') as file:
        for line in file:
            fields = line.strip().split(';')
            time.append(float(fields[0]))
            distance.append(float(fields[11]))

    fig, ax1 = plt.subplots(1, 1, figsize=(12, 8))

    # Plot distance
    ax1.plot(time, distance, label='Distance')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('mm')
    ax1.set_title('Distance Traveled per Movement')
    ax1.legend()
    ax1.grid(True)

    plt.tight_layout()
    plt.show()

def plot_absolute_position(file_path):
    """
    Calculates and plots the robot's absolute path using angle (idx 10)
    and distance (idx 11) from the log file.
    """
    angles_deg, dists = [], []
    with open(file_path, 'r') as file:
        for line in file:
            try:
                fields = line.strip().split(';')
                angles_deg.append(float(fields[10]))
                dists.append(float(fields[11]) )
            except (ValueError, IndexError):
                continue

    if not dists or len(dists) < 2:
        print("Not enough position data found to plot a path.")
        return

    # Initialize path and state variables
    path_x, path_y = [0.0], [0.0]
    current_x, current_y = 0.0, 0.0
    # This will store the final angle of each completed movement segment
    global_angle_offset_rad = 0.0

    for i in range(1, len(dists)):
        # Detect movement reset and update the global angle offset
        if dists[i] < 1.0 and dists[i-1] > 5.0:
            # The previous segment ended. Add its final angle to our offset.
            final_angle_of_segment = math.radians(angles_deg[i-1])
            global_angle_offset_rad += final_angle_of_segment
            # Keep the accumulated angle within the range [-pi, pi]
            global_angle_offset_rad = limit_angle_minus_pi_pi(global_angle_offset_rad)

            delta_x_mm = dists[i] * 10 # On reset, the delta is the new small distance
        else:
            delta_x_mm = (dists[i] - dists[i-1] )* 10
        
        # Angles for the current step relative to the current segment's start
        start_angle_rad_segment = math.radians(angles_deg[i-1])
        end_angle_rad_segment = math.radians(angles_deg[i])

        # Find the shortest angle change for this step within the segment
        delta_angle_rad_segment = get_shortest_delta_angle(end_angle_rad_segment, start_angle_rad_segment)
        
        # Use the intermediate angle for the segment for better accuracy
        intermediate_angle_rad_segment = start_angle_rad_segment + (delta_angle_rad_segment / 2.0)

        # The true orientation is the angle within the segment plus the global offset
        true_intermediate_angle_rad = global_angle_offset_rad + intermediate_angle_rad_segment

        # Calculate the change in position for this step
        rotated_delta_x = delta_x_mm * math.cos(true_intermediate_angle_rad)
        rotated_delta_y = delta_x_mm * math.sin(true_intermediate_angle_rad)

        # Update the absolute position
        current_x += rotated_delta_x
        current_y += rotated_delta_y
        
        path_x.append(current_x)
        path_y.append(current_y)

    # Plot the resulting path
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.plot(path_x, path_y, marker='.', linestyle='-', label='Robot Path')
    ax.set_xlabel('X Position (mm)')
    ax.set_ylabel('Y Position (mm)')
    ax.set_title('Calculated Absolute Robot Position')
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')  # Ensure X and Y axes have the same scale

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_name = f"{filter}/kp_{kp}_ki_{ki}.txt"
    file_path = os.path.join(script_dir, file_name)

    if os.path.exists(file_path):
        # Call all plotting functions
        plot_velocities(file_path)
        plot_distance(file_path)
        plot_pwm_and_bat(file_path)
        plot_absolute_position(file_path)
    else:
        print(f"Error: Data file not found at '{file_path}'")

