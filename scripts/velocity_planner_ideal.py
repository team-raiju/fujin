import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math
from matplotlib.ticker import MultipleLocator

matplotlib.use('TkAgg')

class Position:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


def plot_torque_speed_curve(alpha_0_rad_s2, abs_max_omega_rad_s):
    """Plots the physical motor curve: Angular Velocity vs Max Angular Acceleration."""
    omega_rad_s = np.linspace(0, abs_max_omega_rad_s, 500)
    alpha_rad_s2 = alpha_0_rad_s2 * (1.0 - (omega_rad_s / abs_max_omega_rad_s))

    plt.figure(figsize=(10, 6))
    plt.plot(omega_rad_s, alpha_rad_s2, 'b-', linewidth=2.5, label='Max Available Acceleration')
    
    plt.xlabel('Angular Velocity (rad/s)')
    plt.ylabel('Max Angular Acceleration (rad/s²)')
    plt.title('Motor Physics: Max Angular Acceleration vs Angular Velocity')
    plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_accelerations(real_accels_rad):
    plt.figure(figsize=(10, 6))
    plt.plot(real_accels_rad, 'g-', label='Real Angular Acceleration')
    
    plt.xlabel('Time (ms)')
    plt.ylabel('Angular Acceleration (rad/s²)')
    plt.title('Angular Acceleration Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_angular_speeds(real_speeds_rad):
    plt.figure(figsize=(10, 8))
    plt.plot(real_speeds_rad, 'r-', label='Real Angular Speed')
    
    plt.xlabel('Time (ms)')
    plt.ylabel('Angular Speed (rad/s)')
    plt.title('Angular Speed Over Time (Non-Linear Profile)')
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_robot_positions(real_positions: Position):
    robot = np.array([(pos.x, pos.y) for pos in real_positions])
    
    plt.figure(figsize=(10, 8))
    plt.plot(robot[:, 0], robot[:, 1], 'r-', label='Real Path (Ideal Curve Physics)')
    
    plt.xlim(-90, 540)
    plt.ylim(-90, 540)

    ax = plt.gca()
    ax.xaxis.set_major_locator(MultipleLocator(180))
    ax.yaxis.set_major_locator(MultipleLocator(180))
    ax.set_aspect('equal')
    plt.grid(color='gray', linestyle='--', linewidth=0.5)

    # Add micromouse walls
    plt.plot([0, 0], [0, 180], 'g-', linewidth=3)
    plt.plot([0, 0], [0, -180], 'g-', linewidth=3)
    plt.plot([0, 180], [180, 180], 'g-', linewidth=3)
    plt.plot([180, 180], [0, -180], 'g-', linewidth=3)
    
    plt.plot([180, 360], [180, 180], 'g-', linewidth=3)
    plt.plot([360, 360], [180, 0], 'g-', linewidth=3)
    plt.plot([360, 540], [0, 0], 'g-', linewidth=3)

    for i in range(180, 3600, 180):
        plt.plot([i, i], [180 - i, -i], 'g-', linewidth=3)
        plt.plot([i, i + 180], [-i, -i], 'g-', linewidth=3)
        plt.plot([i + 360, i + 360], [180 - i, -i], 'g-', linewidth=3)
        plt.plot([i + 360, i + 360 + 180], [-i, -i], 'g-', linewidth=3)

    plt.xlabel('X Position (mm)')
    plt.ylabel('Y Position (mm)')
    plt.title('Robot Positions Over Time (Ideal Curve Physics)')
    plt.legend()
    plt.grid(True)
    plt.show()


def main():
    #### Inputs (Strictly in Radians and standard SI/Metric prefixes) ####
    linear_speed_m_s = 0.5
    turn_angle_degree = 90
    turn_angle_rad = np.radians(turn_angle_degree)
    initial_theta_rad = 0.0
    mm_before_turn = 0
    mm_after_turn = 0

    # Curve Physics Parameters (Radians)
    abs_max_omega_rad_s = 34.90658  # With which angular velocity, the max aceleration becomes zero
    ref_angular_speed_rad_s = 10.0  # In this angular velocity, which maximum aceleration (ref_angular_speed_rad_s) do we want             
    ref_acceleration_rad_s2 = 104.71975 # In ref_angular_speed_rad_s what is going to be the maximum aceleration
    max_target_omega_rad_s = 10.0 # Max angular velocity for this profile

    # Calculate stall acceleration (alpha_0)
    alpha_0_rad_s2 = ref_acceleration_rad_s2 / (1.0 - (ref_angular_speed_rad_s / abs_max_omega_rad_s))

    print(f"--- Physics Profile Configuration ---")
    print(f"Stall Acceleration (at 0 speed): {alpha_0_rad_s2:.3f} rad/s²")
    print(f"Absolute Max Angular Speed: {abs_max_omega_rad_s:.3f} rad/s")
    print(f"Target Max Angular Speed Cap: {max_target_omega_rad_s:.3f} rad/s\n")

    # Show the Motor Curve first
    plot_torque_speed_curve(alpha_0_rad_s2, abs_max_omega_rad_s)

    # ---------------------------------------------------------
    # STEP 1: Pre-calculate the continuous acceleration profile
    # ---------------------------------------------------------
    dt = 0.001 # 1 ms steps
    temp_omega = 0.0
    accel_array = []
    omega_array = []
    theta_accel = 0.0

    while temp_omega < max_target_omega_rad_s:
        current_alpha = alpha_0_rad_s2 * (1.0 - (temp_omega / abs_max_omega_rad_s))
        accel_array.append(current_alpha)
        omega_array.append(temp_omega)
        
        theta_accel += temp_omega * dt
        temp_omega += current_alpha * dt
        
        if current_alpha <= 0:
            break

    # ---------------------------------------------------------
    # STEP 2: Stitch the profile based on required turning angle
    # ---------------------------------------------------------
    full_omega = []
    full_alpha = []

    # Check if the turning angle allows reaching the target velocity (Trapezoidal vs Triangular)
    if 2.0 * theta_accel <= turn_angle_rad:
        cruise_angle = turn_angle_rad - (2.0 * theta_accel)
        cruise_steps = int(round(cruise_angle / (max_target_omega_rad_s * dt)))
        
        full_omega.extend(omega_array)
        full_alpha.extend(accel_array)
        
        full_omega.extend([max_target_omega_rad_s] * cruise_steps)
        full_alpha.extend([0.0] * cruise_steps)
        
        full_omega.extend(reversed(omega_array))
        full_alpha.extend([-a for a in reversed(accel_array)])

        t1_ms = len(omega_array)
        t2_ms = cruise_steps
        t3_ms = len(omega_array)

        angle_1_rad = theta_accel
        angle_2_rad = theta_accel + cruise_angle
        angle_3_rad = turn_angle_rad
        
    else:
        target_half_angle = turn_angle_rad / 2.0
        current_angle = 0.0
        idx = 0
        for i, w in enumerate(omega_array):
            current_angle += w * dt
            if current_angle >= target_half_angle:
                idx = i
                break
                
        full_omega.extend(omega_array[:idx])
        full_alpha.extend(accel_array[:idx])
        
        full_omega.extend(reversed(omega_array[:idx]))
        full_alpha.extend([-a for a in reversed(accel_array[:idx])])

        t1_ms = idx
        t2_ms = 0
        t3_ms = idx

        angle_1_rad = current_angle
        angle_2_rad = current_angle
        angle_3_rad = current_angle * 2.0

    T_ms = t1_ms + t2_ms + t3_ms

    # ---------------------------------------------------------
    # STEP 3: Calculate Positions over Time
    # ---------------------------------------------------------
    real_ang_speeds = [0.0]
    real_accelerations = [0.0]
    real_positions = [
        Position(90 + mm_before_turn * np.sin(initial_theta_rad), 
                 mm_before_turn * np.cos(initial_theta_rad), 
                 initial_theta_rad)
    ]

    for i in range(len(full_omega)):
        angular_speed_rad_s = full_omega[i]
        current_alpha = full_alpha[i]
        
        # Update orientation
        theta = real_positions[-1].theta + (angular_speed_rad_s * dt)
        linear_speed_mm_ms = linear_speed_m_s * 1000.0 * dt

        # Update spatial position
        x = real_positions[-1].x + linear_speed_mm_ms * np.sin(theta) 
        y = real_positions[-1].y + linear_speed_mm_ms * np.cos(theta)

        real_positions.append(Position(x, y, theta))
        real_ang_speeds.append(angular_speed_rad_s)
        real_accelerations.append(current_alpha)


    # ---------------------------------------------------------
    # Output Prints (Converted back to degrees strictly for the report)
    # ---------------------------------------------------------
    print(f"t1 (ms):  {t1_ms}")
    print(f"t2 (ms):  {t2_ms}")
    print(f"t3 (ms):  {t3_ms}")
    print(f"T (ms) :  {T_ms}\n")

    print(f"Angle until max speed (deg) : 0.000 to {np.rad2deg(angle_1_rad):.3f}")
    print(f"Angle on max speed (deg)    : {np.rad2deg(angle_1_rad):.3f} to {np.rad2deg(angle_2_rad):.3f}")
    print(f"Final angle (deg)           : {np.rad2deg(angle_2_rad):.3f} to {np.rad2deg(angle_3_rad):.3f}\n")

    print(f"Angle until max speed (rad) : 0.000 to {angle_1_rad:.3f}")
    print(f"Angle on max speed (rad)    : {angle_1_rad:.3f} to {angle_2_rad:.3f}")
    print(f"Final angle (rad)           : {angle_2_rad:.3f} to {angle_3_rad:.3f}\n")

    if t1_ms > 0 and t1_ms < len(real_positions):
        print(f"Reached max angular speed at t = {t1_ms} ms and angle = {np.rad2deg(real_positions[t1_ms].theta):.3f} deg -> {real_positions[t1_ms].theta:.3f} rad")
    if (t1_ms + t2_ms) > 0 and (t1_ms + t2_ms) < len(real_positions):
        print(f"Started deceleration at t = {t1_ms + t2_ms} ms and angle = {np.rad2deg(real_positions[t1_ms + t2_ms].theta):.3f} deg -> {real_positions[t1_ms + t2_ms].theta:.3f} rad")

    print(f"Final Real Position: {real_positions[-1].x:.3f}, {real_positions[-1].y:.3f}, {np.rad2deg(real_positions[-1].theta):.3f}")


    # Manually add the mm_after_turn
    last_x = real_positions[-1].x + mm_after_turn * np.sin(real_positions[-1].theta) 
    last_y = real_positions[-1].y + mm_after_turn * np.cos(real_positions[-1].theta)
    real_positions.append(Position(last_x, last_y, real_positions[-1].theta))

    # Output sequentially
    plot_accelerations(real_accelerations)
    plot_angular_speeds(real_ang_speeds)
    plot_robot_positions(real_positions)

if __name__ == "__main__":
    main()