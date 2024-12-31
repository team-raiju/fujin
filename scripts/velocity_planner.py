import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math
import matplotlib.patches as patches

matplotlib.use('TkAgg')

class Position:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


def plot_robot_positions(ideal_positions: Position, real_positions: Position):

    # Convert positions to numpy arrays for easier handling
    robot1 = np.array([(pos.x, pos.y) for pos in ideal_positions])
    robot2 = np.array([(pos.x, pos.y) for pos in real_positions])
    
    # Create figure and axis
    plt.figure(figsize=(10, 8))
    
    # Plot trajectories
    plt.plot(robot1[:, 0], robot1[:, 1], 'b-', label='Ideal Path')
    plt.plot(robot2[:, 0], robot2[:, 1], 'r-', label='Real Path')
    
    # Set axis limits
    plt.xlim(-92, 630)
    plt.ylim(-92, 540)


     # Add micromouse grid
    ax = plt.gca()
    ax.set_xticks(range(-90, 720, 180))
    ax.set_yticks(range(-180, 540, 180))
    ax.set_aspect('equal')
    plt.grid(color='gray', linestyle='--', linewidth=0.5)

     # Add micromouse walls
    plt.plot([-90, -90], [180, 0], 'g-', linewidth=3)
    plt.plot([-90, -90], [0, -180], 'g-', linewidth=3)
    plt.plot([-90, 90], [180, 180], 'g-', linewidth=3)
    plt.plot([90, 90], [0, -180], 'g-', linewidth=3)


    # Add labels and legend
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Positions Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_angular_speeds(ideal_speeds, real_speeds):

    # Create figure and axis
    plt.figure(figsize=(10, 8))
    
    # Plot angular speeds
    plt.plot(ideal_speeds, 'b-', label='Ideal Angular Speed')
    plt.plot(real_speeds, 'r-', label='Real Angular Speed')
    
    # Add labels and legend
    plt.xlabel('Time (ms)')
    plt.ylabel('Angular Speed (deg/s)')
    plt.title('Angular Speeds Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():

    #### Inputs ####
    linear_speed_m_s = 0.5
    turn_angle_deg = 90
    turn_radius_mm = 55

    angular_acceleration_deg_s2 = 35000
    angular_desacceleration_deg_s2 = 35000
    maximum_angular_speed_deg_s = 560

    mm_before_turn = 0
    mm_after_turn = 100


    #### Units conversion ####
    turn_angle_rad = np.deg2rad(turn_angle_deg)
    turn_radius_m = turn_radius_mm / 1000
    angular_acceleration_rad_s2 = np.deg2rad(angular_acceleration_deg_s2)
    angular_desacceleration_rad_s2 = np.deg2rad(angular_desacceleration_deg_s2)
    maximum_angular_speed_rad_s = np.deg2rad(maximum_angular_speed_deg_s)


    #### Ideal outputs ####
    total_ideal_time_ms = ((turn_angle_rad * turn_radius_m) / linear_speed_m_s) * 1000
    ideal_angular_speed_rad_s = (turn_angle_rad / total_ideal_time_ms) * 1000
    ideal_angular_speed_deg_s = np.rad2deg(ideal_angular_speed_rad_s)
    angle_area_rad = ideal_angular_speed_rad_s * (total_ideal_time_ms / 1000) # = turn_angle_rad

    print("Total ideal time (ms): {:.2f}".format(total_ideal_time_ms))
    print("Ideal angular speed (deg/s): {:.2f}".format(ideal_angular_speed_deg_s))
    print("")


    #### Real outputs ####
    t1_ms = (maximum_angular_speed_rad_s / angular_acceleration_rad_s2) * 1000
    t3_ms = (maximum_angular_speed_rad_s / angular_desacceleration_rad_s2) * 1000
    t2_ms = (angle_area_rad / maximum_angular_speed_rad_s) * 1000 - (t1_ms / 2) - (t3_ms / 2)

    if (t2_ms < 0):
        t2_ms = 0
        t1_ms = math.sqrt((2*angle_area_rad) / ((1 + angular_acceleration_rad_s2 / angular_desacceleration_rad_s2) * angular_acceleration_rad_s2)) * 1000
        t3_ms = t1_ms * (angular_acceleration_rad_s2 / angular_desacceleration_rad_s2)


    t1_ms = round(t1_ms)
    t2_ms = round(t2_ms)
    t3_ms = round(t3_ms)

    T_ms = t1_ms + t2_ms + t3_ms

    print("Max angular speed (deg/s): ", maximum_angular_speed_deg_s, " (=", maximum_angular_speed_rad_s, "rad/s)")
    print("Acc (deg/s^2): ", angular_acceleration_deg_s2, " (=", angular_acceleration_rad_s2, "rad/s^2)")
    print("Desacc (deg/s^2): ", angular_desacceleration_deg_s2, " (=", angular_desacceleration_rad_s2, "rad/s^2)")
    print("t1 (ms): ", t1_ms)
    print("t2 (ms): ", t2_ms)
    print("t3 (ms): ", t3_ms)
    print("T (ms): ", T_ms)


    #### Calculate Ideal Positions ####
    ideal_positions = [
        Position(0, mm_before_turn, 0)
    ]
    ideal_ang_speeds = [0]

    for t in range(1, int(total_ideal_time_ms)):
        theta = ideal_positions[t - 1].theta

        if t < total_ideal_time_ms:
            angular_speed_rad_s = ideal_angular_speed_rad_s
            theta = ideal_positions[t - 1].theta + (angular_speed_rad_s / 1000)
        else:
            angular_speed_rad_s = 0
            theta = turn_angle_rad

        linear_speed_mm_ms = linear_speed_m_s * 1000 / 1000

        x = ideal_positions[t - 1].x + linear_speed_mm_ms * np.sin(theta) 
        y = ideal_positions[t - 1].y + linear_speed_mm_ms * np.cos(theta)

        ideal_positions.append(Position(x, y, theta))
        ideal_ang_speeds.append(angular_speed_rad_s * 180 / np.pi)
        # print("t: ", t, "x: ", x, " y: ", y, " theta: ", theta * 180 / np.pi)
    

    #### Calculate Real Positions ####
    angular_speed_rad_s = 0
    real_ang_speeds = [0]
    real_positions = [
        Position(0, mm_before_turn, 0),    # t=0
    ]

    for t in range(1, T_ms):
        if t <= t1_ms:
            angular_speed_rad_s += angular_acceleration_rad_s2 / 1000
        elif t <= t1_ms + t2_ms:
            angular_speed_rad_s = angular_speed_rad_s # Do nothing
        elif t <= T_ms:
            angular_speed_rad_s -= angular_desacceleration_rad_s2 / 1000
        else:
            angular_speed_rad_s -= angular_desacceleration_rad_s2 / 1000

        if (angular_speed_rad_s > maximum_angular_speed_rad_s):
            angular_speed_rad_s = maximum_angular_speed_rad_s
        elif (angular_speed_rad_s < 0):
            angular_speed_rad_s = 0


        theta = real_positions[t - 1].theta + (angular_speed_rad_s / 1000)
        linear_speed_mm_ms = linear_speed_m_s * 1000 / 1000

        x = real_positions[t - 1].x + linear_speed_mm_ms * np.sin(theta) 
        y = real_positions[t - 1].y + linear_speed_mm_ms * np.cos(theta)

        real_positions.append(Position(x, y, theta))
        real_ang_speeds.append(angular_speed_rad_s * 180 / np.pi)
        # print("t: ", t, "x: ", x, "y: ", y, " angular_speed_rad_s: ", angular_speed_rad_s, " theta: ", theta * 180 / np.pi)
    
    print("Final Real Position: ", real_positions[-1].x, real_positions[-1].y, real_positions[-1].theta * 180 / np.pi)


    #### Manually add the mm_after_turn ####
    last_x = ideal_positions[-1].x + mm_after_turn * np.sin(ideal_positions[-1].theta) 
    last_y = ideal_positions[-1].y + mm_after_turn * np.cos(ideal_positions[-1].theta)
    ideal_positions.append(Position(last_x, last_y, ideal_positions[-1].theta))

    last_x = real_positions[-1].x + mm_after_turn * np.sin(real_positions[-1].theta) 
    last_y = real_positions[-1].y + mm_after_turn * np.cos(real_positions[-1].theta)
    real_positions.append(Position(last_x, last_y, real_positions[-1].theta))

    # Plot the positions
    plot_robot_positions(ideal_positions, real_positions)
    plot_angular_speeds(ideal_ang_speeds, real_ang_speeds)

if __name__ == "__main__":
    main()