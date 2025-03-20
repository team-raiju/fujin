import os
import matplotlib.pyplot as plt
from statistics import mean, stdev

kp = "0.60"
ki = "0.040_2"
filter = "fujin_fan_25"
num = "1"

def calculate_average_error(actual, target):
    total_error = sum(abs(a - t) for a, t in zip(actual, target))
    num_samples = len(actual)
    return total_error / num_samples

def plot_velocities(file_path):
    time = []
    linear_velocity1 = []
    linear_velocity2 = []
    angular_velocity1 = []
    angular_velocity2 = []
    acceleration = []
    pwm_left = []
    pwm_right = []
    with open(file_path, 'r') as file:
        for line in file:
            fields = line.strip().split(';')
            time.append(float(fields[0]))
            linear_velocity1.append(float(fields[1]))
            linear_velocity2.append(float(fields[2]))
            angular_velocity1.append(float(fields[3]))
            angular_velocity2.append(float(fields[4]))
            pwm_left.append(float(fields[5]))
            pwm_right.append(float(fields[6]))

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12))

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

    # Create a list of acceleration values each 10ms
    for i in range(0, len(linear_velocity1) - 50, 50):
        acceleration.append((linear_velocity1[i+50] - linear_velocity1[i]) / 0.05)

        print(f"t({i} - {i+50}): {linear_velocity1[i]:.2f} -> {linear_velocity1[i+50]:.2f} = {acceleration[-1]:.2f}m/s²")


    # Plot pwms
    ax3.plot(time, pwm_left, label='PWM_Left')
    ax3.plot(time, pwm_right, label='PWM_Right')
    ax3.set_xlabel('Time (ms)')
    ax3.set_ylabel('0-1000')
    ax3.set_title('PWMS')
    ax3.legend()
    ax3.grid(True)


    plt.tight_layout()
    plt.show()

def plot_pwm_and_bat(file_path):
    time = []
    pwm_left = []
    pwm_right = []
    battery = []
    
    with open(file_path, 'r') as file:
        for line in file:
            fields = line.strip().split(';')
            time.append(float(fields[0]))
            pwm_left.append(float(fields[5]))
            pwm_right.append(float(fields[6]))
            battery.append(float(fields[7]))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

    # Plot linear velocities
    ax1.plot(time, pwm_left, label='PWM_Left')
    ax1.plot(time, pwm_right, label='PWM_Right')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('0-1000')
    ax1.set_title('PWMS')
    ax1.legend()
    ax1.grid(True)

    # Plot battery voltage
    ax2.plot(time, battery, label='Angular Velocity')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('mV')
    ax2.set_title('Battery Voltage')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

def plot_integral(file_path):
    time = []
    integral_vel = []
    integral_angular = []
    
    with open(file_path, 'r') as file:
        for line in file:
            fields = line.strip().split(';')
            time.append(float(fields[0]))
            integral_vel.append(float(fields[8]))
            integral_angular.append(float(fields[9]))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))

    # Plot integral velocities
    ax1.plot(time, integral_vel, label='integral_vel')
    ax1.set_xlabel('Time (ms)')
    ax1.set_ylabel('-0-100')
    ax1.set_title('integral_vel')
    ax1.legend()
    ax1.grid(True)

    # Plot integral angle
    ax2.plot(time, integral_angular, label='integral_angular')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('-1000-1000')
    ax2.set_title('integral_angular')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_name = filter + '/kp_' + kp + '_ki_' + ki + '.txt'
    file_path = os.path.join(script_dir, file_name)
    plot_velocities(file_path)
    plot_integral(file_path)
    plot_pwm_and_bat(file_path)