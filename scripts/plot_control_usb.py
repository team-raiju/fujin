import serial
import matplotlib.pyplot as plt
import sys
import signal
import os
from datetime import datetime

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
READ_TIMEOUT = 1.0
CONTROL_LOG_MODE = True  # Set to True for control parameters, False for original metrics
PLOT_IMU_DIFF = True     # True: Plots IMU_Encoder_Diff | False: Plots Velocity P/I Terms

def signal_handler(sig, frame):
    """Handles the Ctrl+C signal to ensure a clean exit."""
    print("\nCtrl+C detected! Closing program.")
    sys.exit(0)

def save_log_to_disk(header, data_lines):
    """Saves the structured log strings into a date-organized folder."""
    date_folder = os.path.join("logs", datetime.now().strftime("%Y-%m-%d"))
    if not os.path.exists(date_folder):
        os.makedirs(date_folder)
    
    log_filename = f"log_{datetime.now().strftime('%H-%M-%S')}.txt"
    log_path = os.path.join(date_folder, log_filename)
    
    try:
        with open(log_path, "w") as f:
            f.write(header + "\n")
            f.write("\n".join(data_lines))
        print(f"\n[Success] Log safely stored to: {log_path}")
    except Exception as e:
        print(f"Error saving log file: {e}")

def parse_log_file(file_path):
    """Reads a log file and returns a structured dictionary of data arrays."""
    if not os.path.exists(file_path):
        print(f"Error: Log file '{file_path}' not found.")
        return None

    with open(file_path, 'r') as f:
        lines = f.readlines()

    if len(lines) <= 1:
        print(f"Log file '{file_path}' is empty or missing data rows.")
        return None

    if CONTROL_LOG_MODE:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i', 'rotation_ff', 'linear_ff'
        ]
    else:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'imu_diff', 'battery', 'pos_x', 'pos_y', 'angle', 'dist'
        ]

    data_dict = {k: [] for k in keys}

    for line in lines[1:]:
        line = line.strip()
        if not line: 
            continue
        try:
            fields = [float(x) for x in line.split(';')]
            # Handle legacy files dynamically: loop only up to the available column count
            for idx, field_val in enumerate(fields):
                if idx < len(keys):
                    data_dict[keys[idx]].append(field_val)
        except ValueError:
            continue

    if not data_dict['time']:
        print(f"No valid numerical data could be parsed from '{file_path}'.")
        return None

    return data_dict

def plot_single(data_dict, title_suffix=""):
    """Generates the system analysis visualizations for a single data set."""
    t = data_dict['time']
    
    fig, axs = plt.subplots(3, 2, figsize=(15, 12))
    
    if CONTROL_LOG_MODE:
        fig.suptitle(f'Motion Control Performance {title_suffix}', fontsize=16)

        # Row 1, Col 1: Velocity
        axs[0, 0].plot(t, data_dict['lin_vel_act'], label='Actual Linear Velocity')
        axs[0, 0].plot(t, data_dict['lin_vel_tgt'], label='Target', linestyle='--')
        axs[0, 0].set_title('Linear Velocity')
        axs[0, 0].set_ylabel('m/s')

        # Row 1, Col 2: Angular Velocity
        axs[0, 1].plot(t, data_dict['ang_vel_act'], label='Actual Angular Velocity')
        axs[0, 1].plot(t, data_dict['ang_vel_tgt'], label='Target', linestyle='--')
        axs[0, 1].set_title('Angular Velocity')
        axs[0, 1].set_ylabel('rad/s')

        # Row 2, Col 1: Dynamic Toggle between IMU Diff and Velocity PID
        if PLOT_IMU_DIFF:
            axs[1, 0].plot(t, data_dict['imu_diff'], label='IMU Encoder Diff', color='tab:purple')
            axs[1, 0].set_title('IMU & Encoder Variance')
            axs[1, 0].set_ylabel('Difference')
        else:
            axs[1, 0].plot(t, data_dict['vel_p'], label='Velocity P Term')
            axs[1, 0].plot(t, data_dict['vel_i'], label='Velocity I Term')
            axs[1, 0].set_title('Velocity PID Terms')
            axs[1, 0].set_ylabel('Value')

        # Row 2, Col 2: Angular Velocity PID
        axs[1, 1].plot(t, data_dict['ang_p'], label='Angular P Term')
        axs[1, 1].plot(t, data_dict['ang_i'], label='Angular I Term')
        axs[1, 1].set_title('Angular Velocity PID Terms')
        axs[1, 1].set_ylabel('Value')

        # Row 3, Col 1: PWMs
        axs[2, 0].plot(t, data_dict['pwm_left'], label='PWM Left')
        axs[2, 0].plot(t, data_dict['pwm_right'], label='PWM Right')
        axs[2, 0].set_title('PWM Signals')
        axs[2, 0].set_ylabel('Duty Cycle (0-1000)')

        # Row 3, Col 2: Feed Forward (Plots both if linear_ff data exists)
        axs[2, 1].plot(t, data_dict['rotation_ff'], label='Rotation Feedforward')
        if 'linear_ff' in data_dict and len(data_dict['linear_ff']) == len(t):
            axs[2, 1].plot(t, data_dict['linear_ff'], label='Linear Feedforward')
            axs[2, 1].set_title('Feed Forward Terms')
        else:
            axs[2, 1].set_title('Feed Forward (rotation_ff)')
        axs[2, 1].set_ylabel('Value')

    else:
        fig.suptitle(f'System and Sensor Data {title_suffix}', fontsize=16)

        # Row 1, Col 1: Velocity
        axs[0, 0].plot(t, data_dict['lin_vel_act'], label='Actual Linear Velocity')
        axs[0, 0].plot(t, data_dict['lin_vel_tgt'], label='Target', linestyle='--')
        axs[0, 0].set_title('Linear Velocity')
        axs[0, 0].set_ylabel('m/s')

        # Row 1, Col 2: Angular Velocity
        axs[0, 1].plot(t, data_dict['ang_vel_act'], label='Actual Angular Velocity')
        axs[0, 1].plot(t, data_dict['ang_vel_tgt'], label='Target', linestyle='--')
        axs[0, 1].set_title('Angular Velocity')
        axs[0, 1].set_ylabel('rad/s')

        # Row 2, Col 1: Dynamic Toggle for Distance vs IMU Diff
        if PLOT_IMU_DIFF:
            axs[1, 0].plot(t, data_dict['imu_diff'], label='IMU Encoder Diff', color='tab:purple')
            axs[1, 0].set_title('IMU & Encoder Variance')
            axs[1, 0].set_ylabel('Difference')
        else:
            axs[1, 0].plot(t, data_dict['dist'], label='Distance')
            axs[1, 0].set_title('Distance over Time')
            axs[1, 0].set_ylabel('Distance')

        # Row 2, Col 2: Angle
        axs[1, 1].plot(t, data_dict['angle'], label='Angle')
        axs[1, 1].set_title('Angle over Time')
        axs[1, 1].set_ylabel('Angle (rad)')

        # Row 3, Col 1: PWMs
        axs[2, 0].plot(t, data_dict['pwm_left'], label='PWM Left')
        axs[2, 0].plot(t, data_dict['pwm_right'], label='PWM Right')
        axs[2, 0].set_title('PWM Signals')
        axs[2, 0].set_ylabel('Duty Cycle (0-1000)')

        # Row 3, Col 2: Battery Voltage
        axs[2, 1].plot(t, data_dict['battery'], label='Battery Voltage')
        axs[2, 1].set_title('Battery Voltage over Time')
        axs[2, 1].set_ylabel('Voltage (mV)')

    for ax in axs.flat:
        ax.set_xlabel('Time (ms)')
        ax.legend()
        ax.grid(True)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def plot_comparison(file1, file2, offset=0.0):
    """Parses two log files, applies an optional time offset to file2, and plots both."""
    d1 = parse_log_file(file1)
    d2 = parse_log_file(file2)

    if not d1 or not d2:
        return

    name1 = os.path.basename(file1)
    name2 = os.path.basename(file2)

    if offset != 0.0:
        d2['time'] = [t + offset for t in d2['time']]
        offset_msg = f" (Shifted by {offset:+.1f} ms)"
    else:
        offset_msg = ""

    c1_act, c1_tgt = 'tab:blue', 'tab:cyan'
    c2_act, c2_tgt = 'tab:green', 'tab:olive'

    fig, axs = plt.subplots(3, 2, figsize=(16, 12))
    fig.suptitle(f'Comparison: {name1} vs {name2}{offset_msg}', fontsize=16)

    # Row 1, Col 1: Velocity
    axs[0, 0].plot(d1['time'], d1['lin_vel_act'], color=c1_act, label=f'Actual ({name1})')
    axs[0, 0].plot(d1['time'], d1['lin_vel_tgt'], '--', color=c1_tgt, label=f'Target ({name1})')
    axs[0, 0].plot(d2['time'], d2['lin_vel_act'], color=c2_act, label=f'Actual ({name2})')
    axs[0, 0].plot(d2['time'], d2['lin_vel_tgt'], '--', color=c2_tgt, label=f'Target ({name2})')
    axs[0, 0].set_title('Linear Velocity Comparison')
    axs[0, 0].set_ylabel('m/s')

    # Row 1, Col 2: Angular Velocity
    axs[0, 1].plot(d1['time'], d1['ang_vel_act'], color=c1_act, label=f'Actual ({name1})')
    axs[0, 1].plot(d1['time'], d1['ang_vel_tgt'], '--', color=c1_tgt, label=f'Target ({name1})')
    axs[0, 1].plot(d2['time'], d2['ang_vel_act'], color=c2_act, label=f'Actual ({name2})')
    axs[0, 1].plot(d2['time'], d2['ang_vel_tgt'], '--', color=c2_tgt, label=f'Target ({name2})')
    axs[0, 1].set_title('Angular Velocity Comparison')
    axs[0, 1].set_ylabel('rad/s')

    if CONTROL_LOG_MODE:
        # Row 2, Col 1: Dynamic comparison toggle
        if PLOT_IMU_DIFF:
            axs[1, 0].plot(d1['time'], d1['imu_diff'], color=c1_act, label=f'IMU Diff ({name1})')
            axs[1, 0].plot(d2['time'], d2['imu_diff'], color=c2_act, label=f'IMU Diff ({name2})')
            axs[1, 0].set_title('IMU Variance Comparison')
        else:
            axs[1, 0].plot(d1['time'], d1['vel_p'], color=c1_act, label=f'Vel P ({name1})')
            axs[1, 0].plot(d1['time'], d1['vel_i'], ':', color=c1_act, label=f'Vel I ({name1})')
            axs[1, 0].plot(d2['time'], d2['vel_p'], color=c2_act, label=f'Vel P ({name2})')
            axs[1, 0].plot(d2['time'], d2['vel_i'], ':', color=c2_act, label=f'Vel I ({name2})')
            axs[1, 0].set_title('Velocity PID Terms Comparison')
        axs[1, 0].set_ylabel('Value')

        # Row 2, Col 2: Angular Velocity PID
        axs[1, 1].plot(d1['time'], d1['ang_p'], color=c1_act, label=f'Ang P ({name1})')
        axs[1, 1].plot(d1['time'], d1['ang_i'], ':', color=c1_act, label=f'Ang I ({name1})')
        axs[1, 1].plot(d2['time'], d2['ang_p'], color=c2_act, label=f'Ang P ({name2})')
        axs[1, 1].plot(d2['time'], d2['ang_i'], ':', color=c2_act, label=f'Ang I ({name2})')
        axs[1, 1].set_title('Angular PID Terms Comparison')
        axs[1, 1].set_ylabel('Value')

        # Row 3, Col 2: Feed Forward Comparison
        axs[2, 1].plot(d1['time'], d1['rotation_ff'], color=c1_act, label=f'Rot FF ({name1})')
        axs[2, 1].plot(d2['time'], d2['rotation_ff'], color=c2_act, label=f'Rot FF ({name2})')
        
        # Dynamically append linear feedforward to comparison if available
        if 'linear_ff' in d1 and len(d1['linear_ff']) == len(d1['time']):
            axs[2, 1].plot(d1['time'], d1['linear_ff'], color=c1_act, label=f'Lin FF ({name1})')
        if 'linear_ff' in d2 and len(d2['linear_ff']) == len(d2['time']):
            axs[2, 1].plot(d1['time'], d1['linear_ff'], color=c2_act, label=f'Lin FF ({name1})')

            
        axs[2, 1].set_title('Feedforward Terms Comparison')
        axs[2, 1].set_ylabel('Value')
    else:
        # Row 2, Col 1: Non-control context dynamic toggle
        if PLOT_IMU_DIFF:
            axs[1, 0].plot(d1['time'], d1['imu_diff'], color=c1_act, label=name1)
            axs[1, 0].plot(d2['time'], d2['imu_diff'], color=c2_act, label=name2)
            axs[1, 0].set_title('IMU Variance Comparison')
            axs[1, 0].set_ylabel('Difference')
        else:
            axs[1, 0].plot(d1['time'], d1['dist'], color=c1_act, label=name1)
            axs[1, 0].plot(d2['time'], d2['dist'], color=c2_act, label=name2)
            axs[1, 0].set_title('Distance Tracking Comparison')
            axs[1, 0].set_ylabel('Distance')

        # Row 2, Col 2: Spatial Odometry Tracking (pos_x vs pos_y)
        axs[1, 1].plot(d1['pos_x'], d1['pos_y'], color=c1_act, label=f'Path ({name1})', marker='o', markersize=2, alpha=0.7)
        axs[1, 1].plot(d2['pos_x'], d2['pos_y'], color=c2_act, label=f'Path ({name2})', marker='x', markersize=2, alpha=0.7)
        axs[1, 1].set_title('Spatial Odometry Tracking')
        axs[1, 1].set_xlabel('X Position (m)')
        axs[1, 1].set_ylabel('Y Position (m)')
        axs[1, 1].axis('equal')

        # Row 3, Col 2: Battery Voltage
        axs[2, 1].plot(d1['time'], d1['battery'], color=c1_act, label=name1)
        axs[2, 1].plot(d2['time'], d2['battery'], color=c2_act, label=name2)
        axs[2, 1].set_title('Battery Voltage Comparison')
        axs[2, 1].set_ylabel('mV')

    # Row 3, Col 1: PWM Signals (shared layout property across both modes)
    axs[2, 0].plot(d1['time'], d1['pwm_left'], color=c1_act, label=f'PWM Left ({name1})')
    axs[2, 0].plot(d1['time'], d1['pwm_right'], ':', color=c1_act, label=f'PWM Right ({name1})')
    axs[2, 0].plot(d2['time'], d2['pwm_left'], color=c2_act, label=f'PWM Left ({name2})')
    axs[2, 0].plot(d2['time'], d2['pwm_right'], ':', color=c2_act, label=f'PWM Right ({name2})')
    axs[2, 0].set_title('PWM Signals Comparison')
    axs[2, 0].set_ylabel('Duty Cycle (0-1000)')

    for ax in axs.flat:
        if not (ax == axs[1, 1] and not CONTROL_LOG_MODE):  # skip setting time label on spatial odometry plot
            ax.set_xlabel('Time (ms)')
        ax.legend(fontsize='small')
        ax.grid(True)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def collect_print_and_plot_data():
    """Connects to serial port, pipes text stream, logs to disk, and updates graphs."""
    if CONTROL_LOG_MODE:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'imu_diff', 'vel_p', 'vel_i', 'ang_p', 'ang_i', 'rotation_ff', 'linear_ff'
        ]
        header = (
            "Time(ms);ActualLinearVel;TargetLinearVel;ActualAngularVel;TargetAngularVel;"
            "PWML;PWMR;ImuDiff;VelP;VelI;AngP;AngI;RotationFF;LinearFF"
        )
    else:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'imu_diff', 'battery', 'pos_x', 'pos_y', 'angle', 'dist'
        ]
        header = (
            "Time(ms);ActualLinearVel;TargetLinearVel;ActualAngularVel;TargetAngularVel;"
            "PWML;PWMR;ImuDiff;Battery(mV);PosX(m);PosY(m);Angle(rad);Dist"
        )

    data_dict = {k: [] for k in keys}
    data_lines = []

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT) as ser:
            ser.flush()
            print(f"Successfully connected to {SERIAL_PORT}")
            print("Waiting for data burst... Press physical button or use Ctrl+C to exit.")
            
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print("Data reception started...")
                    break
            
            while line:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    break 
                
                try:
                    fields = line.split(';')
                    if len(fields) >= len(fields): # parse whatever columns show up
                        data_lines.append(";".join(fields))
                        for idx, field_val in enumerate(fields):
                            if idx < len(keys):
                                data_dict[keys[idx]].append(float(field_val))
                except (ValueError, IndexError) as e:
                    print(f"Skipping malformed line '{line}': {e}")
            
            print(f"Data collection complete. Received {len(data_dict['time'])} data points.")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
        return

    if not data_dict['time']:
        print("No data was collected. Exiting.")
        return

    save_log_to_disk(header, data_lines)
    plot_single(data_dict, title_suffix="(Live Session)")

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    args_count = len(sys.argv)
    
    if args_count == 2:
        parsed_data = parse_log_file(sys.argv[1])
        if parsed_data:
            plot_single(parsed_data, title_suffix=f"({os.path.basename(sys.argv[1])})")
            
    elif args_count >= 3:
        file_one = sys.argv[1]
        file_two = sys.argv[2]
        
        time_offset = 0.0
        if args_count >= 4:
            try:
                time_offset = float(sys.argv[3])
            except ValueError:
                print(f"Warning: Invalid offset target context '{sys.argv[3]}'. Defaulting to 0.0.")
        
        plot_comparison(file_one, file_two, offset=time_offset)
        
    else:
        collect_print_and_plot_data()