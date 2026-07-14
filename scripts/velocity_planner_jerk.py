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


def plot_results(times_ms, accel_hist, omega_hist, positions):
    fig, axes = plt.subplots(1, 3, figsize=(21, 7))

    # --- Angular acceleration over time ---
    axes[0].plot(times_ms, accel_hist, 'g-', linewidth=1.5, label='Angular Acceleration')
    axes[0].axhline(0, color='black', linewidth=0.6, linestyle='--')
    axes[0].set_xlabel('Time (ms)')
    axes[0].set_ylabel('Angular Acceleration (rad/s²)')
    axes[0].set_title('Angular Acceleration Over Time')
    axes[0].legend()
    axes[0].grid(True)

    # --- Angular speed over time ---
    axes[1].plot(times_ms, omega_hist, 'b-', linewidth=1.5, label='Angular Speed')
    axes[1].set_xlabel('Time (ms)')
    axes[1].set_ylabel('Angular Speed (rad/s)')
    axes[1].set_title('Angular Speed Over Time')
    axes[1].set_ylim(0, max(omega_hist) * 1.3 if max(omega_hist) > 0 else 1)
    axes[1].legend()
    axes[1].grid(True)

    # --- Trajectory ---
    robot = np.array([(pos.x, pos.y) for pos in positions])
    axes[2].plot(robot[:, 0], robot[:, 1], 'r-', linewidth=1.5, label='Real Path')
    axes[2].set_xlim(-90, 540)
    axes[2].set_ylim(-90, 540)
    axes[2].xaxis.set_major_locator(MultipleLocator(180))
    axes[2].yaxis.set_major_locator(MultipleLocator(180))
    axes[2].set_aspect('equal')
    axes[2].grid(color='gray', linestyle='--', linewidth=0.5)

    # Micromouse walls
    axes[2].plot([0, 0], [0, 180], 'g-', linewidth=3)
    axes[2].plot([0, 0], [0, -180], 'g-', linewidth=3)
    axes[2].plot([0, 180], [180, 180], 'g-', linewidth=3)
    axes[2].plot([180, 180], [0, -180], 'g-', linewidth=3)
    axes[2].plot([180, 360], [180, 180], 'g-', linewidth=3)
    axes[2].plot([360, 360], [180, 0], 'g-', linewidth=3)
    axes[2].plot([360, 540], [0, 0], 'g-', linewidth=3)
    for i in range(180, 3600, 180):
        axes[2].plot([i, i], [180 - i, -i], 'g-', linewidth=3)
        axes[2].plot([i, i + 180], [-i, -i], 'g-', linewidth=3)
        axes[2].plot([i + 360, i + 360], [180 - i, -i], 'g-', linewidth=3)
        axes[2].plot([i + 360, i + 360 + 180], [-i, -i], 'g-', linewidth=3)

    axes[2].set_xlabel('X Position (mm)')
    axes[2].set_ylabel('Y Position (mm)')
    axes[2].set_title('Robot Trajectory')
    axes[2].legend()

    plt.tight_layout()
    plt.show()


def simulate_ramp(theta0, omega0, t_start_ms, duration_s, alpha_fn, dt,
                   linear_speed_m_s, max_omega_rad_s, positions,
                   accel_hist, omega_hist, times_ms, last_pos):
    """
    Steps the simulation forward for `duration_s` seconds using alpha_fn(local_time_s) -> alpha.
    Appends to the shared history lists. Returns (theta, omega, steps_taken).
    """
    theta = theta0
    omega = omega0
    n_steps = int(round(duration_s / dt))

    for i in range(n_steps):
        local_t = i * dt
        alpha = alpha_fn(local_t)

        omega += alpha * dt
        if omega > max_omega_rad_s:
            omega = max_omega_rad_s
        elif omega < 0.0:
            omega = 0.0

        theta += omega * dt

        x = last_pos[0] + (linear_speed_m_s * 1000.0 * dt) * np.sin(theta)
        y = last_pos[1] + (linear_speed_m_s * 1000.0 * dt) * np.cos(theta)
        last_pos[0], last_pos[1] = x, y

        positions.append(Position(x, y, theta))
        accel_hist.append(alpha)
        omega_hist.append(omega)
        times_ms.append(t_start_ms + (i + 1))

    return theta, omega, n_steps


def main():
    #### Inputs (Strictly in Radians and standard SI/Metric prefixes) ####
    linear_speed_m_s = 1
    turn_angle_degree = 90.0
    turn_angle_rad = np.radians(turn_angle_degree)

    max_angular_acceleration_rad_s2 = 1000
    max_angular_speed_rad_s = 25        
    max_jerk_1_rad_s3 = 100000.0 # Jerk for 0 -> max accel, and -max_accel -> 0
    max_jerk_2_rad_s3 = 40000.0  # Jerk for max accel -> 0, and 0 -> -max_accel

    dt = 0.001  # 1 ms steps

    print("--- Inputs ---")
    print(f"Linear speed               : {linear_speed_m_s} m/s")
    print(f"Turn angle                 : {turn_angle_degree:.3f} deg -> {turn_angle_rad:.3f} rad")
    print(f"Max angular acceleration   : {max_angular_acceleration_rad_s2:.3f} rad/s²")
    print(f"Max angular speed (cap)    : {max_angular_speed_rad_s:.3f} rad/s")
    print(f"Max jerk 1 limit           : {max_jerk_1_rad_s3:.3f} rad/s³")
    print(f"Max jerk 2 limit           : {max_jerk_2_rad_s3:.3f} rad/s³")
    print("")

    #### 1. Calculate Analytical Kinematics ####
    # We solve for the maximum achievable speed given the half-distance constraint
    theta_half = turn_angle_rad / 2.0
    j1 = max_jerk_1_rad_s3
    j2 = max_jerk_2_rad_s3
    
    # Calculate what speed we would reach if we didn't have enough room to hit max acceleration
    K = 1.0 / (6.0 * j1**2) + 1.0 / (2.0 * j1 * j2) + 1.0 / (3.0 * j2**2)
    A_peak_dist = (theta_half / K) ** (1.0 / 3.0)
    
    if A_peak_dist <= max_angular_acceleration_rad_s2:
        # Turn is very short, we don't even reach max_alpha
        achievable_omega = 0.5 * (A_peak_dist**2) * (1.0 / j1 + 1.0 / j2)
    else:
        # We reach max_alpha
        achieved_alpha = max_angular_acceleration_rad_s2
        tau1 = achieved_alpha / j1
        tau3 = achieved_alpha / j2
        omega_1 = 0.5 * j1 * (tau1**2)
        
        C_quad = (1.0/6.0) * (achieved_alpha**3) / (j1**2) + omega_1 * tau3 + (1.0/3.0) * (achieved_alpha**3) / (j2**2) - theta_half
        B_quad = omega_1 + achieved_alpha * tau3
        A_quad = 0.5 * achieved_alpha
        
        discriminant = B_quad**2 - 4 * A_quad * C_quad
        tau2 = (-B_quad + math.sqrt(discriminant)) / (2 * A_quad) if discriminant >= 0 else 0.0
        achievable_omega = omega_1 + achieved_alpha * tau2 + 0.5 * j2 * (tau3**2)
        
    # The actual peak omega is restricted by our physical speed cap
    peak_omega = min(max_angular_speed_rad_s, achievable_omega)
    
    if peak_omega < max_angular_speed_rad_s * 0.999:
        print(f"WARNING: Turn angle is too short to reach max angular speed.")
        print(f"Peak speed capped at {peak_omega:.3f} rad/s instead of {max_angular_speed_rad_s:.3f} rad/s.\n")

    # Now recalculate phase times based on the locked-in peak_omega
    A_req = math.sqrt(2.0 * peak_omega / (1.0 / j1 + 1.0 / j2))
    
    if A_req <= max_angular_acceleration_rad_s2:
        # Triangular acceleration profile
        achieved_alpha = A_req
        tau1 = achieved_alpha / j1
        tau2 = 0.0
        tau3 = achieved_alpha / j2
    else:
        # Trapezoidal acceleration profile
        achieved_alpha = max_angular_acceleration_rad_s2
        tau1 = achieved_alpha / j1
        tau3 = achieved_alpha / j2
        omega_ramp = 0.5 * (achieved_alpha**2) * (1.0 / j1 + 1.0 / j2)
        tau2 = (peak_omega - omega_ramp) / achieved_alpha
        
    #### 2. Verify auto-calculated transition angles ####
    # Angle swept in phase 1 (jerk ramp)
    transition_angle_1_rad = (1.0 / 6.0) * j1 * (tau1 ** 3)
    
    # Angle swept in phase 2 (const accel)
    omega_1 = 0.5 * achieved_alpha * tau1
    dtheta2 = (omega_1 * tau2) + (0.5 * achieved_alpha * (tau2 ** 2))
    transition_angle_2_rad = transition_angle_1_rad + dtheta2

    print("--- Auto-Calculated Constraints ---")
    print(f"Achieved max acceleration  : {achieved_alpha:.3f} rad/s²")
    print(f"Achieved peak speed        : {peak_omega:.3f} rad/s")
    print(f"Calculated trans angle 1   : {np.rad2deg(transition_angle_1_rad):.3f} deg")
    print(f"Calculated trans angle 2   : {np.rad2deg(transition_angle_2_rad):.3f} deg")
    print("")

    #### Define alpha functions for simulation ####
    def alpha_phase1(t): return achieved_alpha * (t / tau1) if tau1 > 0 else 0.0
    def alpha_phase2(t): return achieved_alpha
    def alpha_phase3(t): return achieved_alpha * (1.0 - t / tau3) if tau3 > 0 else 0.0

    #### Compute Euler-simulated theta to match the exact discretised simulation steps ####
    def compute_simulated_theta_1():
        theta, omega = 0.0, 0.0
        for alpha_fn, dur in ((alpha_phase1, tau1), (alpha_phase2, tau2), (alpha_phase3, tau3)):
            n = int(round(dur / dt))
            for i in range(n):
                alpha = alpha_fn(i * dt)
                omega += alpha * dt
                omega = min(max(omega, 0.0), max_angular_speed_rad_s)
                theta += omega * dt
        return theta, omega

    sim_theta_1, sim_omega_peak = compute_simulated_theta_1()

    # Calculate cruise phase based on the simulated acceleration angle to avoid rounding errors
    cruise_angle_rad = turn_angle_rad - (2.0 * sim_theta_1)
    if cruise_angle_rad < 1e-9:
        cruise_angle_rad = 0.0
        cruise_time_s = 0.0
    else:
        cruise_time_s = cruise_angle_rad / sim_omega_peak

    t1_ms = int(round((tau1 + tau2 + tau3) * 1000))
    t2_ms = int(round(cruise_time_s * 1000))
    T_ms = t1_ms + t2_ms + t1_ms

    print("--- Timing ---")
    print(f"tau1 (jerk 1 ramp) time    : {tau1 * 1000:.3f} ms")
    print(f"tau2 (const max accel) time: {tau2 * 1000:.3f} ms")
    print(f"tau3 (jerk 2 ramp) time    : {tau3 * 1000:.3f} ms")
    print(f"t1 (ms) [acceleration]     : {t1_ms}")
    print(f"t2 (ms) [cruise]           : {t2_ms}")
    print(f"t3 (ms) [deceleration]     : {t1_ms}")
    print(f"T  (ms) [total]            : {T_ms}")
    print("")

    print(f"Angle until max speed (deg) : 0.000 to {np.rad2deg(sim_theta_1):.3f}")
    print(f"Angle on max speed (deg)    : {np.rad2deg(sim_theta_1):.3f} to {np.rad2deg(sim_theta_1 + cruise_angle_rad):.3f}")
    print(f"Final angle (deg)           : {np.rad2deg(sim_theta_1 + cruise_angle_rad):.3f} to {turn_angle_degree:.3f}")
    print("")

    #### Full simulation ####
    positions = [Position(90.0, 0.0, 0.0)]
    accel_hist = [0.0]
    omega_hist = [0.0]
    times_ms = [0]
    last_pos = [positions[0].x, positions[0].y]

    theta, omega = 0.0, 0.0
    t_ms = 0

    # --- Phase 1: jerk ramp-up ---
    theta, omega, n = simulate_ramp(theta, omega, t_ms, tau1, alpha_phase1, dt,
                                     linear_speed_m_s, max_angular_speed_rad_s,
                                     positions, accel_hist, omega_hist, times_ms, last_pos)
    t_ms += n
    print(f"Reached transition angle 1 at t = {t_ms} ms, angle = {np.rad2deg(theta):.3f} deg -> {theta:.3f} rad")

    # --- Phase 2: constant max acceleration ---
    theta, omega, n = simulate_ramp(theta, omega, t_ms, tau2, alpha_phase2, dt,
                                     linear_speed_m_s, max_angular_speed_rad_s,
                                     positions, accel_hist, omega_hist, times_ms, last_pos)
    t_ms += n
    print(f"Reached transition angle 2 at t = {t_ms} ms, angle = {np.rad2deg(theta):.3f} deg -> {theta:.3f} rad")

    # --- Phase 3: jerk ramp-down ---
    theta, omega, n = simulate_ramp(theta, omega, t_ms, tau3, alpha_phase3, dt,
                                     linear_speed_m_s, max_angular_speed_rad_s,
                                     positions, accel_hist, omega_hist, times_ms, last_pos)
    t_ms += n
    print(f"Reached max angular speed at t = {t_ms} ms, angle = {np.rad2deg(theta):.3f} deg -> {theta:.3f} rad")

    # --- Cruise phase ---
    if t2_ms > 0:
        theta, omega, n = simulate_ramp(theta, omega, t_ms, t2_ms / 1000.0, lambda t: 0.0, dt,
                                         linear_speed_m_s, max_angular_speed_rad_s,
                                         positions, accel_hist, omega_hist, times_ms, last_pos)
        t_ms += n
    print(f"Started deceleration at t = {t_ms} ms, angle = {np.rad2deg(theta):.3f} deg -> {theta:.3f} rad")

    # --- Decel A: jerk ramp-up (deceleration grows 0 -> -max_alpha) ---
    def alpha_decelA(t): return -achieved_alpha * (t / tau3) if tau3 > 0 else 0.0
    def alpha_decelB(t): return -achieved_alpha
    def alpha_decelC(t): return -achieved_alpha * (1.0 - t / tau1) if tau1 > 0 else 0.0

    theta, omega, n = simulate_ramp(theta, omega, t_ms, tau3, alpha_decelA, dt,
                                     linear_speed_m_s, max_angular_speed_rad_s,
                                     positions, accel_hist, omega_hist, times_ms, last_pos)
    t_ms += n

    # --- Decel B: constant max deceleration ---
    theta, omega, n = simulate_ramp(theta, omega, t_ms, tau2, alpha_decelB, dt,
                                     linear_speed_m_s, max_angular_speed_rad_s,
                                     positions, accel_hist, omega_hist, times_ms, last_pos)
    t_ms += n

    # --- Decel C: jerk ramp-down (deceleration eases back to 0) ---
    theta, omega, n = simulate_ramp(theta, omega, t_ms, tau1, alpha_decelC, dt,
                                     linear_speed_m_s, max_angular_speed_rad_s,
                                     positions, accel_hist, omega_hist, times_ms, last_pos)
    t_ms += n

    print("")
    print(f"Final Real Position: {positions[-1].x:.3f}, {positions[-1].y:.3f}, {np.rad2deg(positions[-1].theta):.3f} deg")
    print(f"Final angular speed: {omega:.5f} rad/s (should be ~0)")
    print(f"Total simulated time: {t_ms} ms")

    plot_results(times_ms, accel_hist, omega_hist, positions)


if __name__ == "__main__":
    main()