#!/usr/bin/env python3
"""
S-Curve Linear Acceleration Simulator & Plotter
Based on implementation in firmware/src/services/navigation.cpp

Simulates the target linear speed profile for the sequence:
START -> FORWARD -> TURN_RIGHT_90 -> FORWARD -> STOP
"""

import math
from enum import Enum, auto
import matplotlib.pyplot as plt
import numpy as np


# ==============================================================================
# CONFIGURATION & PARAMETERS (EDITABLE USER INPUTS)
# ==============================================================================

# Control loop frequency
CONTROL_FREQUENCY_HZ = 2000.0  # [Hz] (matches Config::CONTROL_FREQUENCY_HZ)
CONTROL_PERIOD_S = 1.0 / CONTROL_FREQUENCY_HZ

# Jerk parameters for S-curve acceleration and braking
ACC_JERK = 625.0    # [m/s^3] (acc_jerk in navigation.cpp)
BRAKE_JERK = 625.0  # [m/s^3] (brake_jerk in navigation.cpp)

# Margins
BREAK_MARGIN = 20.0  # [mm] Final velocity reached on target_travel_mm - break_margin
ACCEL_MARGIN = 20.0  # [mm] Only accelerates after accel_margin

# Minimum movement speed
MIN_MOVE_SPEED = 0.2  # [m/s] (Config::min_move_speed)

# Micromouse Maze Constants
CELL_SIZE_MM = 180.0
HALF_CELL_SIZE_MM = 90.0
ROBOT_DIST_FROM_CENTER_START_MM_FAST = 21.0


class Movement(Enum):
    START = auto()
    FORWARD = auto()
    TURN_RIGHT_90 = auto()
    STOP = auto()
    DIAGONAL = auto()


class MiniFSMStates(Enum):
    FORWARD_1 = auto()
    TURN = auto()
    FORWARD_2 = auto()
    STABILIZE_1 = auto()
    STABILIZE_2 = auto()


class ForwardParams:
    """Matches C++ ForwardParams struct"""
    def __init__(self, max_speed: float, acceleration: float, deceleration: float, target_travel_mm: float):
        self.max_speed = max_speed          # [m/s]
        self.acceleration = acceleration    # [m/s^2]
        self.deceleration = deceleration    # [m/s^2]
        self.target_travel_mm = target_travel_mm  # [mm]


class TurnParams:
    """Matches C++ TurnParams struct"""
    def __init__(self, start_mm: float, end_mm: float, turn_linear_speed: float,
                 angular_accel: float, max_angular_speed: float,
                 t_start_deccel_ms: float, t_stop_ms: float, sign: int = -1):
        self.start = start_mm                     # [mm]
        self.end = end_mm                         # [mm]
        self.turn_linear_speed = turn_linear_speed  # [m/s]
        self.angular_accel = angular_accel        # [rad/s^2]
        self.max_angular_speed = max_angular_speed# [rad/s]
        self.t_start_deccel_ms = t_start_deccel_ms# [ms]
        self.t_stop_ms = t_stop_ms                # [ms]
        self.sign = sign


# Forward parameters for each movement (User editable inputs)
FORWARD_PARAMS = {
    Movement.START: ForwardParams(
        max_speed=0.5,
        acceleration=12.0,
        deceleration=12.0,
        target_travel_mm=HALF_CELL_SIZE_MM + ROBOT_DIST_FROM_CENTER_START_MM_FAST,
    ),
    Movement.FORWARD: ForwardParams(
        max_speed=7.5,
        acceleration=35.0,
        deceleration=35.0,
        target_travel_mm=CELL_SIZE_MM,
    ),
    Movement.TURN_RIGHT_90: ForwardParams(
        max_speed=0.5,
        acceleration=10.0,
        deceleration=10.0,
        target_travel_mm=5.0,
    ),
    Movement.STOP: ForwardParams(
        max_speed=0.5,
        acceleration=20.0,
        deceleration=20.0,
        target_travel_mm=(HALF_CELL_SIZE_MM - 10.0),
    ),
}

# Turn parameters for each movement (User editable inputs)
TURN_PARAMS = {
    Movement.START: TurnParams(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0),
    Movement.FORWARD: TurnParams(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0),
    Movement.TURN_RIGHT_90: TurnParams(
        start_mm=0.0,
        end_mm=-11.0,
        turn_linear_speed=0.5,
        angular_accel=785.40,
        max_angular_speed=26.18,
        t_start_deccel_ms=60.0,
        t_stop_ms=108.0,
        sign=-1,
    ),
    Movement.STOP: TurnParams(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0),
}


# ==============================================================================
# S-CURVE & KINEMATICS HELPER FUNCTIONS
# ==============================================================================

def get_torricelli_distance(final_speed: float, initial_speed: float, acceleration: float) -> float:
    if acceleration == 0.0:
        return 0.0
    return (final_speed * final_speed - initial_speed * initial_speed) / (2.0 * acceleration)


def get_s_curve_brake_distance(initial_speed: float, final_speed: float, deceleration: float, jerk: float) -> float:
    """Calculates S-curve brake distance in meters assuming initial acceleration = 0."""
    if initial_speed <= final_speed or deceleration <= 0.0 or jerk <= 0.0:
        return 0.0

    delta_v = initial_speed - final_speed
    v_threshold = (deceleration * deceleration) / jerk

    if delta_v >= v_threshold:
        return ((initial_speed * initial_speed - final_speed * final_speed) / (2.0 * deceleration)) + \
               (((initial_speed + final_speed) * deceleration) / (2.0 * jerk))
    else:
        return (initial_speed + final_speed) * math.sqrt(delta_v / jerk)


def start_accel_ramp_down(current_speed: float, current_accel: float, max_speed: float, jerk: float) -> bool:
    if current_accel <= 0.0 or jerk <= 0.0:
        return False
    pred_speed = current_speed + (current_accel * current_accel) / (2.0 * jerk)
    return pred_speed >= max_speed


def start_brake_ramp_up(current_speed: float, current_accel: float, final_speed: float, jerk: float) -> bool:
    if current_accel > 0.0 or jerk <= 0.0:
        return False
    pred_speed = current_speed - (current_accel * current_accel) / (2.0 * jerk)
    return pred_speed <= final_speed


def get_effective_max_acceleration(current_speed: float, base_max_accel: float) -> float:
    """Motor torque derating: reduces maximum achievable acceleration at higher speeds due to Back-EMF."""
    if current_speed > 7.0:
        return 0.50 * base_max_accel
    elif current_speed > 6.0:
        return 0.70 * base_max_accel
    elif current_speed > 5.0:
        return 0.85 * base_max_accel
    else:
        return base_max_accel


# ==============================================================================
# SIMULATION ENGINE
# ==============================================================================

def run_simulation(movement_sequence):
    time_log = []
    total_dist_log = []
    movement_dist_log = []
    speed_log = []
    accel_log = []
    movement_name_log = []
    braking_log = []

    control_linear_speed = 0.0        # m/s
    total_traveled_dist_mm = 0.0      # mm
    total_time_s = 0.0                # s

    transitions = []  # list of tuples: (time_s, total_dist_mm, label, speed_m_s)

    parsed_sequence = []
    for item in movement_sequence:
        move, cnt = item if isinstance(item, tuple) else (item, 1)
        if parsed_sequence and parsed_sequence[-1][0] == move and move in [Movement.FORWARD, Movement.DIAGONAL]:
            parsed_sequence[-1] = (move, parsed_sequence[-1][1] + cnt)
        else:
            parsed_sequence.append((move, cnt))
    for i, movement in enumerate(movement_sequence):
        prev_movement = movement_sequence[i - 1] if i > 0 else Movement.START
        next_movement = movement_sequence[i + 1] if i < len(movement_sequence) - 1 else Movement.STOP
        count = 1

    for i, (movement, count) in enumerate(parsed_sequence):
        prev_movement = parsed_sequence[i - 1][0] if i > 0 else Movement.START
        next_movement = parsed_sequence[i + 1][0] if i < len(parsed_sequence) - 1 else Movement.STOP

        # Setup movement params (matching Navigation::set_movement)
        complete_prev_move_travel = -1.0 * TURN_PARAMS[prev_movement].end

        if movement in [Movement.FORWARD, Movement.DIAGONAL]:
            target_travel_mm = (complete_prev_move_travel + 
                                (FORWARD_PARAMS[movement].target_travel_mm * count) + 
                                TURN_PARAMS[next_movement].start)
        elif movement == Movement.START:
            target_travel_mm = FORWARD_PARAMS[movement].target_travel_mm + TURN_PARAMS[next_movement].start
        else:
            target_travel_mm = complete_prev_move_travel + FORWARD_PARAMS[movement].target_travel_mm

        # Determine forward_end_speed (matching Navigation::set_movement lines 785-796)
        if movement == Movement.STOP:
            forward_end_speed = 0.0
        elif movement == Movement.START:
            forward_end_speed = FORWARD_PARAMS[Movement.START].max_speed
        elif next_movement in [Movement.FORWARD, Movement.DIAGONAL]:
            forward_end_speed = FORWARD_PARAMS[next_movement].max_speed
        elif next_movement == Movement.STOP:
            forward_end_speed = FORWARD_PARAMS[next_movement].max_speed
        else:
            forward_end_speed = TURN_PARAMS[next_movement].turn_linear_speed

        traveled_dist_mm = 0.0
        current_linear_acceleration = 0.0
        is_braking = False
        is_finished = False
        mini_fsm_state = MiniFSMStates.FORWARD_1
        turn_tick_counter = 0

        label = f"{movement.name} x{count}" if count > 1 else movement.name
        transitions.append((total_time_s, total_traveled_dist_mm, label, control_linear_speed))
        transitions.append((total_time_s, total_traveled_dist_mm, movement.name, control_linear_speed))

        max_ticks_safety = int(30 * CONTROL_FREQUENCY_HZ)
        ticks = 0

        while not is_finished and ticks < max_ticks_safety:
            ticks += 1
            dt = CONTROL_PERIOD_S

            # Log data
            time_log.append(total_time_s)
            total_dist_log.append(total_traveled_dist_mm)
            movement_dist_log.append(traveled_dist_mm)
            speed_log.append(control_linear_speed)
            accel_log.append(current_linear_acceleration)
            movement_name_log.append(movement.name)
            braking_log.append(is_braking)

            # Step logic
            if movement in [Movement.START, Movement.FORWARD, Movement.DIAGONAL, Movement.STOP]:
                if movement == Movement.STOP:
                    forward_end_speed = 0.0

                max_speed = FORWARD_PARAMS[movement].max_speed
                max_acceleration = FORWARD_PARAMS[movement].acceleration
                deceleration = FORWARD_PARAMS[movement].deceleration

                acc_jerk = ACC_JERK
                brake_jerk = BRAKE_JERK
                break_margin = BREAK_MARGIN
                accel_margin = ACCEL_MARGIN

                # Calculate brake distance including ramp-down of positive acceleration to 0
                if current_linear_acceleration > 0.0 and brake_jerk > 0.0:
                    t_ramp = current_linear_acceleration / brake_jerk
                    delta_v = (current_linear_acceleration ** 2) / (2.0 * brake_jerk)
                    v_peak = control_linear_speed + delta_v
                    d_ramp_m = (control_linear_speed * t_ramp) + (current_linear_acceleration ** 3) / (3.0 * (brake_jerk ** 2))
                else:
                    v_peak = control_linear_speed
                    d_ramp_m = 0.0

                required_brake_distance = (1000.0 * (d_ramp_m + get_s_curve_brake_distance(
                    v_peak, forward_end_speed, deceleration, brake_jerk
                ))) + break_margin

                if not is_braking and (abs(traveled_dist_mm) < (target_travel_mm - required_brake_distance)):
                    if control_linear_speed < 1.0 or abs(traveled_dist_mm) > accel_margin:
                        if control_linear_speed >= max_speed:
                            current_linear_acceleration = 0.0
                            control_linear_speed = max_speed
                        elif start_accel_ramp_down(control_linear_speed, current_linear_acceleration, max_speed, acc_jerk):
                            current_linear_acceleration -= (acc_jerk / CONTROL_FREQUENCY_HZ)
                            current_linear_acceleration = max(current_linear_acceleration, 0.0)
                            control_linear_speed += current_linear_acceleration / CONTROL_FREQUENCY_HZ
                            control_linear_speed = min(control_linear_speed, max_speed)
                        else:
                            eff_max_accel = get_effective_max_acceleration(control_linear_speed, max_acceleration)
                            if current_linear_acceleration < eff_max_accel:
                                current_linear_acceleration += (acc_jerk / CONTROL_FREQUENCY_HZ)
                                current_linear_acceleration = min(current_linear_acceleration, eff_max_accel)
                            elif current_linear_acceleration > eff_max_accel:
                                current_linear_acceleration -= (acc_jerk / CONTROL_FREQUENCY_HZ)
                                current_linear_acceleration = max(current_linear_acceleration, eff_max_accel)

                            control_linear_speed += current_linear_acceleration / CONTROL_FREQUENCY_HZ
                            control_linear_speed = min(control_linear_speed, max_speed)
                elif abs(traveled_dist_mm) > accel_margin:
                    is_braking = True
                    if control_linear_speed > forward_end_speed:
                        if start_brake_ramp_up(control_linear_speed, current_linear_acceleration, forward_end_speed, brake_jerk):
                            current_linear_acceleration += (brake_jerk / CONTROL_FREQUENCY_HZ)
                            current_linear_acceleration = min(current_linear_acceleration, 0.0)
                        else:
                            current_linear_acceleration -= (brake_jerk / CONTROL_FREQUENCY_HZ)
                            current_linear_acceleration = max(current_linear_acceleration, -deceleration)

                        control_linear_speed += current_linear_acceleration / CONTROL_FREQUENCY_HZ
                        control_linear_speed = max(control_linear_speed, forward_end_speed)
                        if forward_end_speed > 0.0:
                            control_linear_speed = max(control_linear_speed, MIN_MOVE_SPEED)
                    else:
                        current_linear_acceleration = 0.0
                        control_linear_speed = min(control_linear_speed, forward_end_speed)
                        if forward_end_speed > 0.0:
                            control_linear_speed = max(control_linear_speed, MIN_MOVE_SPEED)

                if (abs(traveled_dist_mm) >= target_travel_mm) or (movement == Movement.STOP and is_braking and control_linear_speed <= 0.0):
                    is_finished = True

            elif movement in [Movement.TURN_RIGHT_90]:
                if mini_fsm_state in [MiniFSMStates.FORWARD_1, MiniFSMStates.FORWARD_2]:
                    max_speed = FORWARD_PARAMS[movement].max_speed
                    acceleration = FORWARD_PARAMS[movement].acceleration
                    deceleration = FORWARD_PARAMS[movement].deceleration
                    final_speed = FORWARD_PARAMS[movement].max_speed

                    brake_dist = 1000.0 * get_torricelli_distance(final_speed, control_linear_speed, -deceleration)

                    if abs(traveled_dist_mm) < (target_travel_mm - brake_dist):
                        if control_linear_speed < max_speed:
                            control_linear_speed += acceleration / CONTROL_FREQUENCY_HZ
                            control_linear_speed = min(control_linear_speed, max_speed)
                    else:
                        if control_linear_speed > final_speed:
                            control_linear_speed -= deceleration / CONTROL_FREQUENCY_HZ
                            control_linear_speed = max(control_linear_speed, MIN_MOVE_SPEED)

                    if abs(traveled_dist_mm) >= target_travel_mm:
                        traveled_dist_mm = 0.0
                        turn_tick_counter = 0
                        mini_fsm_state = MiniFSMStates.TURN
                        current_linear_acceleration = 0.0

                elif mini_fsm_state == MiniFSMStates.TURN:
                    curr_turn_params = TURN_PARAMS[movement]
                    t_stop_ticks = int(curr_turn_params.t_stop_ms * (CONTROL_FREQUENCY_HZ / 1000.0))

                    elapsed_time_ticks = turn_tick_counter
                    turn_tick_counter += 1

                    if elapsed_time_ticks > t_stop_ticks:
                        is_finished = True
                        mini_fsm_state = MiniFSMStates.FORWARD_1

            # Update simulated distance traveled by robot with control_linear_speed
            delta_x_mm = (control_linear_speed * 1000.0) / CONTROL_FREQUENCY_HZ
            traveled_dist_mm += delta_x_mm
            total_traveled_dist_mm += delta_x_mm
            total_time_s += dt

    return {
        "time": time_log,
        "total_dist": total_dist_log,
        "movement_dist": movement_dist_log,
        "speed": speed_log,
        "accel": accel_log,
        "movement_name": movement_name_log,
        "braking": braking_log,
        "transitions": transitions,
    }


# ==============================================================================
# PLOTTING FUNCTION
# ==============================================================================

def plot_results(data, show_plot=True, save_path="s_curve_linear_velocity_profile.png"):
    time_s = np.array(data["time"])
    speed_m_s = np.array(data["speed"])
    accel_m_s2 = np.array(data["accel"])
    dist_mm = np.array(data["total_dist"])
    transitions = data["transitions"]

    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(13, 11), sharex=False)

    color_speed = '#2ecc71'  # Green
    color_accel = '#ff7f0e'  # Orange
    color_dist  = '#3498db'  # Blue

    y_max = max(speed_m_s) * 1.15 if len(speed_m_s) > 0 else 1.0

    # 1. Target Linear Speed vs Time
    ax1.plot(time_s, speed_m_s, color=color_speed, linewidth=2.5, label="Target Linear Speed [m/s]")
    ax1.axhline(y=0.5, color='gray', linestyle=':', label="Target Turn Speed (0.5 m/s)")
    ax1.set_ylabel("Speed [m/s]", fontsize=11, fontweight='bold')
    ax1.set_title("S-Curve Simulation: Target Linear Velocity vs Time", fontsize=14, fontweight='bold', pad=10)
    ax1.grid(True, linestyle="--", alpha=0.7)
    ax1.set_ylim(-0.05, y_max)

    for t_s, d_mm, label, v_m_s in transitions:
        ax1.axvline(x=t_s, color='red', linestyle='--', alpha=0.6)
        ax1.scatter([t_s], [v_m_s], color='red', s=45, zorder=5)
        ax1.text(t_s + 0.005, y_max * 0.85, f"{label} ({t_s:.2f}s)", color='red', rotation=90, fontsize=8, fontweight='bold')

    ax1.legend(loc="upper right")

    # 2. Target Linear Speed vs Cumulative Traveled Distance
    ax2.plot(dist_mm, speed_m_s, color=color_dist, linewidth=2.5, label="Target Linear Speed [m/s]")
    ax2.axhline(y=0.5, color='gray', linestyle=':', label="Target Turn Speed (0.5 m/s)")
    ax2.set_xlabel("Cumulative Traveled Distance [mm]", fontsize=11, fontweight='bold')
    ax2.set_ylabel("Speed [m/s]", fontsize=11, fontweight='bold')
    ax2.set_title("Target Linear Velocity vs Cumulative Traveled Distance", fontsize=14, fontweight='bold', pad=10)
    ax2.grid(True, linestyle="--", alpha=0.7)
    ax2.set_ylim(-0.05, y_max)

    for t_s, d_mm, label, v_m_s in transitions:
        ax2.axvline(x=d_mm, color='red', linestyle='--', alpha=0.6)
        ax2.scatter([d_mm], [v_m_s], color='red', s=45, zorder=5)
        ax2.text(d_mm + 3, y_max * 0.85, f"{label} ({d_mm:.0f}mm)", color='red', rotation=90, fontsize=8, fontweight='bold')

    ax2.legend(loc="upper right")

    # 3. Linear Acceleration vs Time
    ax3.plot(time_s, accel_m_s2, color=color_accel, linewidth=1.5, label="Linear Acceleration [m/s²]")
    ax3.set_xlabel("Time [s]", fontsize=11, fontweight='bold')
    ax3.set_ylabel("Acceleration [m/s²]", fontsize=11, fontweight='bold')
    ax3.set_title("Linear Acceleration vs Time", fontsize=14, fontweight='bold', pad=10)
    ax3.grid(True, linestyle="--", alpha=0.7)

    for t_s, d_mm, label, v_m_s in transitions:
        ax3.axvline(x=t_s, color='red', linestyle='--', alpha=0.6)

    ax3.legend(loc="upper right")

    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"Plot saved to: {save_path}")
    if show_plot:
        plt.show()


if __name__ == "__main__":
    sequence = [
        Movement.START,
        *([Movement.FORWARD] * 20),
        Movement.FORWARD,
        Movement.STOP,
    ]

    print("Running S-Curve simulation for sequence:")
    print(" -> ".join([m.name if not isinstance(m, tuple) else f"{m[0].name} x{m[1]}" for m in sequence]))
    print(" -> ".join([m.name for m in sequence]))

    data = run_simulation(sequence)

    print("\nSimulation Breakdown:")
    for t_s, d_mm, label, v_m_s in data["transitions"]:
        print(f"  {label:15s} | Time: {t_s:6.3f} s | Dist: {d_mm:7.2f} mm | Speed at transition: {v_m_s:.3f} m/s")

    print(f"\nTotal Time: {data['time'][-1]:.3f} s")
    print(f"Total Traveled Distance: {data['total_dist'][-1]:.2f} mm")

    plot_results(data, show_plot=True)
