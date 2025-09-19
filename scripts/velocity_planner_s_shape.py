import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math
import matplotlib.patches as patches
from matplotlib.ticker import MultipleLocator

matplotlib.use('TkAgg')

class Position:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


def plot_robot_positions(real_positions: Position):

    # Convert positions to numpy arrays for easier handling
    robot = np.array([(pos.x, pos.y) for pos in real_positions])
    
    # Create figure and axis
    plt.figure(figsize=(10, 8))
    
    # Plot trajectories
    plt.plot(robot[:, 0], robot[:, 1], 'r-', label='Real Path')
    
    # Set axis limits
    # plt.xlim(-90, 2700+360)
    # plt.ylim(-2790, 360)

    plt.xlim(-90, 540)
    plt.ylim(-90, 540)


     # Add micromouse grid
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

    ## Discomment to add 45deg ladder pattern
    # for i in range (0, 7200, 180):
    #     plt.plot([i, i], [i, i + 180], 'g-', linewidth=3)
    #     plt.plot([i, i + 180], [i + 180, i + 180], 'g-', linewidth=3)
        
    #     plt.plot([i + 180, i + 360], [i, i], 'g-', linewidth=3)
    #     plt.plot([i + 360, i + 360], [i, i + 180], 'g-', linewidth=3)
    
    ## Discomment to add 135deg ladder pattern
    plt.plot([180, 360], [180, 180], 'g-', linewidth=3)
    plt.plot([360, 360], [180, 0], 'g-', linewidth=3)
    plt.plot([360, 540], [0, 0], 'g-', linewidth=3)

    for i in range (180, 3600, 180):
        plt.plot([i, i], [180 - i, -i], 'g-', linewidth=3)
        plt.plot([i, i + 180], [-i, -i], 'g-', linewidth=3)

        plt.plot([i + 360, i + 360], [180 - i, -i], 'g-', linewidth=3)
        plt.plot([i + 360, i + 360 + 180], [-i, -i], 'g-', linewidth=3)


    # Add labels and legend
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Positions Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_angular_speeds(real_speeds):

    # Create figure and axis
    plt.figure(figsize=(10, 8))
    
    # Plot angular speeds
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
    linear_speed_m_s = 0.7
    turn_angle_deg = 90
    angular_transition_deg = 25
    maximum_angular_speed_deg_s = 900
    sharpness_factor = 1.0

    mm_before_turn = 0
    mm_after_turn = 0
    initial_theta_deg = 0


    #### Units conversion ####
    turn_angle_rad = np.deg2rad(turn_angle_deg)
    angular_transition_rad =  np.deg2rad(angular_transition_deg)
    initial_theta_rad = np.deg2rad(initial_theta_deg)
    maximum_angular_speed_rad_s = np.deg2rad(maximum_angular_speed_deg_s)

    #### Print inputs ####
    print(f"linear_speed_m_s: {linear_speed_m_s}")
    print(f"turn_angle_deg: {turn_angle_deg} -> {turn_angle_rad:.3f} rad")
    print(f"angular_transition_deg: {angular_transition_deg} -> {angular_transition_rad:.3f} rad")
    print(f"maximum_angular_speed_deg_s: {maximum_angular_speed_deg_s} -> {maximum_angular_speed_rad_s:.3f} rad/s")
    print(f"sharpness_factor: {sharpness_factor}")
    print(f"mm_before_turn: {mm_before_turn}")
    print(f"mm_after_turn: {mm_after_turn}")


    #### Calculate Real Positions ####
    angular_speed_rad_s = 0
    real_ang_speeds = [0]
    real_positions = [
        Position(90 + mm_before_turn * np.sin(initial_theta_rad), 
                  mm_before_turn * np.cos(initial_theta_rad), 
                  initial_theta_rad)
    ]

    angle_turned_rad = 0
    deceleration_start_angle = turn_angle_rad - angular_transition_rad
    t = 1
    traveled_dist = 0
    turn_arc_dist = 0
    last_angular_speed_rad_s = 0
    max_measured_acceleration_rad_s2 = 0

    while (angle_turned_rad < (turn_angle_rad)):
        speed_factor = 1
        if (angle_turned_rad < angular_transition_rad):
            factor = (angle_turned_rad / angular_transition_rad)
            speed_factor = math.pow(math.sin(factor * math.pi / 2.0), sharpness_factor)
        elif (angle_turned_rad >= deceleration_start_angle) :
            remaining_angle = turn_angle_rad - angle_turned_rad
            factor = (remaining_angle / angular_transition_rad)
            speed_factor = math.pow(math.sin(factor * math.pi / 2.0), sharpness_factor)


        # print(f"speed_factor: {speed_factor}; maximum_angular_speed_rad_s: {maximum_angular_speed_rad_s}")
        angular_speed_rad_s = maximum_angular_speed_rad_s * speed_factor
        
        measured_acceleration_rad_s2 = math.fabs(1000 * (angular_speed_rad_s - last_angular_speed_rad_s))
        if (measured_acceleration_rad_s2 > max_measured_acceleration_rad_s2):
            max_measured_acceleration_rad_s2 = measured_acceleration_rad_s2
        last_angular_speed_rad_s = angular_speed_rad_s

        if (angular_speed_rad_s < 0.05):
            angular_speed_rad_s = 0.05


        theta = real_positions[t - 1].theta + (angular_speed_rad_s / 1000)
        linear_speed_mm_ms = linear_speed_m_s * 1000 / 1000

        x = real_positions[t - 1].x + linear_speed_mm_ms * np.sin(theta) 
        y = real_positions[t - 1].y + linear_speed_mm_ms * np.cos(theta)
        traveled_dist += linear_speed_mm_ms

        real_positions.append(Position(x, y, theta))
        real_ang_speeds.append(angular_speed_rad_s * 180 / np.pi)


        if (real_positions[t - 1].theta < angular_transition_rad and theta >= angular_transition_rad): 
            print(f"t: {t}, x: {x:.3f}, y: {y:.3f}, angular_speed_rad_s: {angular_speed_rad_s:.3f}, theta: {theta * 180 / np.pi:.3f}, traveled_dist: {traveled_dist:.3f}")
            turn_arc_dist = traveled_dist

        elif (real_positions[t - 1].theta < deceleration_start_angle and theta >= deceleration_start_angle):
            print(f"t: {t}, x: {x:.3f}, y: {y:.3f}, angular_speed_rad_s: {angular_speed_rad_s:.3f}, theta: {theta * 180 / np.pi:.3f}, traveled_dist: {traveled_dist:.3f}")
            turn_arc_dist = traveled_dist - turn_arc_dist
            print(f"Turn arc distance: {turn_arc_dist:.3f} mm")

        angle_turned_rad = theta
        t = t + 1

    print(f"Final Real Position: {real_positions[-1].x:.3f}, {real_positions[-1].y:.3f}, {real_positions[-1].theta * 180 / np.pi:.3f}")
    print(f"Max measured acceleration: {max_measured_acceleration_rad_s2:.3f} rad/s² -> {max_measured_acceleration_rad_s2 * 180 / np.pi:.3f} deg/s²")
    print(f"Total time: {t} ms")

    #### Manually add the mm_after_turn ####
    last_x = real_positions[-1].x + mm_after_turn * np.sin(real_positions[-1].theta) 
    last_y = real_positions[-1].y + mm_after_turn * np.cos(real_positions[-1].theta)
    real_positions.append(Position(last_x, last_y, real_positions[-1].theta))

    # Plot the positions
    plot_robot_positions(real_positions)
    plot_angular_speeds(real_ang_speeds)

if __name__ == "__main__":
    main()