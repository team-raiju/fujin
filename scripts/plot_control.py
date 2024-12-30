import os
import matplotlib.pyplot as plt
from statistics import mean, stdev

kp = "11"
ki = "0.180"
filter = "fujin_butterworth_25hz"
num = "1"

def plot_velocities(file_path):
    time = []
    linear_velocity1 = []
    linear_velocity2 = []
    angular_velocity1 = []
    angular_velocity2 = []

    with open(file_path, 'r') as file:
        for line in file:
            fields = line.strip().split(';')
            time.append(float(fields[0]))
            linear_velocity1.append(float(fields[1]))
            linear_velocity2.append(float(fields[2]))
            angular_velocity1.append(float(fields[3]))
            angular_velocity2.append(float(fields[4]))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

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

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_name = filter + '/kp_' + kp + '_ki_' + ki + '.txt'
    file_path = os.path.join(script_dir, file_name)
    plot_velocities(file_path)