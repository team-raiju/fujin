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
CONTROL_LOG_MODE = True  # Set to True for new control parameters, False for original metrics

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

    # Detect mode based on header if possible
    header_line = lines[0].strip().lower()
    is_control = "velp" in header_line or "rotff" in header_line

    if is_control:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'vel_p', 'vel_i', 'ang_p', 'ang_i', 'rotation_ff'
        ]
    else:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'battery', 'pos_x', 'pos_y', 'angle', 'dist'
        ]

    data_dict = {k: [] for k in keys}
    data_dict['is_control_mode'] = is_control

    for line in lines[1:]:
        line = line.strip()
        if not line: 
            continue
        try:
            fields = [float(x) for x in line.split(';')]
            if len(fields) >= 12:
                for idx, key in enumerate(keys):
                    data_dict[key].append(fields[idx])
        except ValueError:
            continue

    if not data_dict['time']:
        print(f"No valid numerical data could be parsed from '{file_path}'.")
        return None

    return data_dict

def plot_single(data_dict, title_suffix=""):
    """Generates the system analysis visualizations for a single data set."""
    t = data_dict['time']
    is_control = data_dict.get('is_control_mode', CONTROL_LOG_MODE)
    
    if is_control:
        fig, axs = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(f'Motion Control Performance {title_suffix}', fontsize=16)

        # Row 1, Col 1: Velocity
        ax1 = axs[0, 0]
        ax1.plot(t, data_dict['lin_vel_act'], label='Actual Linear Velocity')
        ax1.plot(t, data_dict['lin_vel_tgt'], label='Target', linestyle='--')
        ax1.set_title('Linear Velocity')
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('m/s')
        ax1.legend()
        ax1.grid(True)

        # Row 1, Col 2: Angular Velocity
        ax2 = axs[0, 1]
        ax2.plot(t, data_dict['ang_vel_act'], label='Actual Angular Velocity')
        ax2.plot(t, data_dict['ang_vel_tgt'], label='Target', linestyle='--')
        ax2.set_title('Angular Velocity')
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('rad/s')
        ax2.legend()
        ax2.grid(True)

        # Row 2, Col 1: Velocity PID
        ax3 = axs[1, 0]
        ax3.plot(t, data_dict['vel_p'], label='Velocity P Term')
        ax3.plot(t, data_dict['vel_i'], label='Velocity I Term')
        ax3.set_title('Velocity PID Terms')
        ax3.set_xlabel('Time (ms)')
        ax3.set_ylabel('Value')
        ax3.legend()
        ax3.grid(True)

        # Row 2, Col 2: Angular Velocity PID
        ax4 = axs[1, 1]
        ax4.plot(t, data_dict['ang_p'], label='Angular P Term')
        ax4.plot(t, data_dict['ang_i'], label='Angular I Term')
        ax4.set_title('Angular Velocity PID Terms')
        ax4.set_xlabel('Time (ms)')
        ax4.set_ylabel('Value')
        ax4.legend()
        ax4.grid(True)

        # Row 3, Col 1: PWMs
        ax5 = axs[2, 0]
        ax5.plot(t, data_dict['pwm_left'], label='PWM Left')
        ax5.plot(t, data_dict['pwm_right'], label='PWM Right')
        ax5.set_title('PWM Signals')
        ax5.set_xlabel('Time (ms)')
        ax5.set_ylabel('Duty Cycle (0-1000)')
        ax5.legend()
        ax5.grid(True)

        # Row 3, Col 2: Feed Forward
        ax6 = axs[2, 1]
        ax6.plot(t, data_dict['rotation_ff'], label='Rotation Feedforward')
        ax6.set_title('Feed Forward (rotation_ff)')
        ax6.set_xlabel('Time (ms)')
        ax6.set_ylabel('Value')
        ax6.legend()
        ax6.grid(True)

        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    else:
        fig, axs = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(f'System and Sensor Data {title_suffix}', fontsize=16)

        # Row 1, Col 1: Velocity
        ax1 = axs[0, 0]
        ax1.plot(t, data_dict['lin_vel_act'], label='Actual Linear Velocity')
        ax1.plot(t, data_dict['lin_vel_tgt'], label='Target', linestyle='--')
        ax1.set_title('Linear Velocity')
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('m/s')
        ax1.legend()
        ax1.grid(True)

        # Row 1, Col 2: Angular Velocity
        ax2 = axs[0, 1]
        ax2.plot(t, data_dict['ang_vel_act'], label='Actual Angular Velocity')
        ax2.plot(t, data_dict['ang_vel_tgt'], label='Target', linestyle='--')
        ax2.set_title('Angular Velocity')
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('rad/s')
        ax2.legend()
        ax2.grid(True)

        # Row 2, Col 1: Distance
        ax3 = axs[1, 0]
        ax3.plot(t, data_dict['dist'], label='Distance')
        ax3.set_title('Distance over Time')
        ax3.set_xlabel('Time (ms)')
        ax3.set_ylabel('Distance')
        ax3.legend()
        ax3.grid(True)

        # Row 2, Col 2: Angle
        ax4 = axs[1, 1]
        ax4.plot(t, data_dict['angle'], label='Angle')
        ax4.set_title('Angle over Time')
        ax4.set_xlabel('Time (ms)')
        ax4.set_ylabel('Angle (rad)')
        ax4.legend()
        ax4.grid(True)

        # Row 3, Col 1: PWMs
        ax5 = axs[2, 0]
        ax5.plot(t, data_dict['pwm_left'], label='PWM Left')
        ax5.plot(t, data_dict['pwm_right'], label='PWM Right')
        ax5.set_title('PWM Signals')
        ax5.set_xlabel('Time (ms)')
        ax5.set_ylabel('Duty Cycle (0-1000)')
        ax5.legend()
        ax5.grid(True)

        # Row 3, Col 2: Battery Voltage
        ax6 = axs[2, 1]
        ax6.plot(t, data_dict['battery'], label='Battery Voltage')
        ax6.set_title('Battery Voltage over Time')
        ax6.set_xlabel('Time (ms)')
        ax6.set_ylabel('Voltage (mV)')
        ax6.legend()
        ax6.grid(True)

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

    # Apply timeline shift to the second data set if requested
    if offset != 0.0:
        d2['time'] = [t + offset for t in d2['time']]
        offset_msg = f" (Shifted by {offset:+.1f} ms)"
    else:
        offset_msg = ""

    # Distinct color palette pairing[cite: 3]
    c1_act, c1_tgt = 'tab:blue', 'tab:cyan'
    c2_act, c2_tgt = 'tab:green', 'tab:olive'

    is_control = d1.get('is_control_mode', CONTROL_LOG_MODE)

    if is_control:
        fig, axs = plt.subplots(3, 2, figsize=(16, 12))
        fig.suptitle(f'Comparison: {name1} vs {name2}{offset_msg}', fontsize=16)

        # Row 1, Col 1: Velocity
        ax1 = axs[0, 0]
        ax1.plot(d1['time'], d1['lin_vel_act'], color=c1_act, label=f'Actual ({name1})')
        ax1.plot(d1['time'], d1['lin_vel_tgt'], '--', color=c1_tgt, label=f'Target ({name1})')
        ax1.plot(d2['time'], d2['lin_vel_act'], color=c2_act, label=f'Actual ({name2})')
        ax1.plot(d2['time'], d2['lin_vel_tgt'], '--', color=c2_tgt, label=f'Target ({name2})')
        ax1.set_title('Linear Velocity Comparison')
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('m/s')
        ax1.legend(fontsize='small')
        ax1.grid(True)

        # Row 1, Col 2: Angular Velocity
        ax2 = axs[0, 1]
        ax2.plot(d1['time'], d1['ang_vel_act'], color=c1_act, label=f'Actual ({name1})')
        ax2.plot(d1['time'], d1['ang_vel_tgt'], '--', color=c1_tgt, label=f'Target ({name1})')
        ax2.plot(d2['time'], d2['ang_vel_act'], color=c2_act, label=f'Actual ({name2})')
        ax2.plot(d2['time'], d2['ang_vel_tgt'], '--', color=c2_tgt, label=f'Target ({name2})')
        ax2.set_title('Angular Velocity Comparison')
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('rad/s')
        ax2.legend(fontsize='small')
        ax2.grid(True)

        # Row 2, Col 1: Velocity PID
        ax3 = axs[1, 0]
        ax3.plot(d1['time'], d1['vel_p'], color=c1_act, label=f'Vel P ({name1})')
        ax3.plot(d1['time'], d1['vel_i'], ':', color=c1_act, label=f'Vel I ({name1})')
        ax3.plot(d2['time'], d2['vel_p'], color=c2_act, label=f'Vel P ({name2})')
        ax3.plot(d2['time'], d2['vel_i'], ':', color=c2_act, label=f'Vel I ({name2})')
        ax3.set_title('Velocity PID Terms Comparison')
        ax3.set_xlabel('Time (ms)')
        ax3.set_ylabel('Value')
        ax3.legend(fontsize='small')
        ax3.grid(True)

        # Row 2, Col 2: Angular Velocity PID
        ax4 = axs[1, 1]
        ax4.plot(d1['time'], d1['ang_p'], color=c1_act, label=f'Ang P ({name1})')
        ax4.plot(d1['time'], d1['ang_i'], ':', color=c1_act, label=f'Ang I ({name1})')
        ax4.plot(d2['time'], d2['ang_p'], color=c2_act, label=f'Ang P ({name2})')
        ax4.plot(d2['time'], d2['ang_i'], ':', color=c2_act, label=f'Ang I ({name2})')
        ax4.set_title('Angular PID Terms Comparison')
        ax4.set_xlabel('Time (ms)')
        ax4.set_ylabel('Value')
        ax4.legend(fontsize='small')
        ax4.grid(True)

        # Row 3, Col 1: PWMs
        ax5 = axs[2, 0]
        ax5.plot(d1['time'], d1['pwm_left'], color=c1_act, label=f'PWM Left ({name1})')
        ax5.plot(d1['time'], d1['pwm_right'], ':', color=c1_act, label=f'PWM Right ({name1})')
        ax5.plot(d2['time'], d2['pwm_left'], color=c2_act, label=f'PWM Left ({name2})')
        ax5.plot(d2['time'], d2['pwm_right'], ':', color=c2_act, label=f'PWM Right ({name2})')
        ax5.set_title('PWM Signals Comparison')
        ax5.set_xlabel('Time (ms)')
        ax5.set_ylabel('Duty Cycle (0-1000)')
        ax5.legend(fontsize='small')
        ax5.grid(True)

        # Row 3, Col 2: Feed Forward
        ax6 = axs[2, 1]
        ax6.plot(d1['time'], d1['rotation_ff'], color=c1_act, label=name1)
        ax6.plot(d2['time'], d2['rotation_ff'], color=c2_act, label=name2)
        ax6.set_title('Rotation Feedforward Comparison')
        ax6.set_xlabel('Time (ms)')
        ax6.set_ylabel('Value')
        ax6.legend(fontsize='small')
        ax6.grid(True)

        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    else:
        fig, axs = plt.subplots(3, 2, figsize=(16, 12))
        fig.suptitle(f'Comparison: {name1} vs {name2}{offset_msg}', fontsize=16)

        # Row 1, Col 1: Velocity
        ax1 = axs[0, 0]
        ax1.plot(d1['time'], d1['lin_vel_act'], color=c1_act, label=f'Actual ({name1})')
        ax1.plot(d1['time'], d1['lin_vel_tgt'], '--', color=c1_tgt, label=f'Target ({name1})')
        ax1.plot(d2['time'], d2['lin_vel_act'], color=c2_act, label=f'Actual ({name2})')
        ax1.plot(d2['time'], d2['lin_vel_tgt'], '--', color=c2_tgt, label=f'Target ({name2})')
        ax1.set_title('Linear Velocity Comparison')
        ax1.set_xlabel('Time (ms)')
        ax1.set_ylabel('m/s')
        ax1.legend(fontsize='small')
        ax1.grid(True)

        # Row 1, Col 2: Angular Velocity
        ax2 = axs[0, 1]
        ax2.plot(d1['time'], d1['ang_vel_act'], color=c1_act, label=f'Actual ({name1})')
        ax2.plot(d1['time'], d1['ang_vel_tgt'], '--', color=c1_tgt, label=f'Target ({name1})')
        ax2.plot(d2['time'], d2['ang_vel_act'], color=c2_act, label=f'Actual ({name2})')
        ax2.plot(d2['time'], d2['ang_vel_tgt'], '--', color=c2_tgt, label=f'Target ({name2})')
        ax2.set_title('Angular Velocity Comparison')
        ax2.set_xlabel('Time (ms)')
        ax2.set_ylabel('rad/s')
        ax2.legend(fontsize='small')
        ax2.grid(True)

        # Row 2, Col 1: Distance Comparison
        ax3 = axs[1, 0]
        ax3.plot(d1['time'], d1['dist'], color=c1_act, label=name1)
        ax3.plot(d2['time'], d2['dist'], color=c2_act, label=name2)
        ax3.set_title('Distance Tracking Comparison')
        ax3.set_xlabel('Time (ms)')
        ax3.set_ylabel('Distance')
        ax3.legend(fontsize='small')
        ax3.grid(True)

        # Row 2, Col 2: Spatial Odometry Tracking Comparison (pos_x vs pos_y)
        ax4 = axs[1, 1]
        ax4.plot(d1['pos_x'], d1['pos_y'], color=c1_act, label=f'Path ({name1})', marker='o', markersize=2, alpha=0.7)
        ax4.plot(d2['pos_x'], d2['pos_y'], color=c2_act, label=f'Path ({name2})', marker='x', markersize=2, alpha=0.7)
        ax4.set_title('Spatial Odometry Tracking')
        ax4.set_xlabel('X Position (m)')
        ax4.set_ylabel('Y Position (m)')
        ax4.axis('equal')
        ax4.legend(fontsize='small')
        ax4.grid(True)

        # Row 3, Col 1: PWMs
        ax5 = axs[2, 0]
        ax5.plot(d1['time'], d1['pwm_left'], color=c1_act, label=f'PWM Left ({name1})')
        ax5.plot(d1['time'], d1['pwm_right'], ':', color=c1_act, label=f'PWM Right ({name1})')
        ax5.plot(d2['time'], d2['pwm_left'], color=c2_act, label=f'PWM Left ({name2})')
        ax5.plot(d2['time'], d2['pwm_right'], ':', color=c2_act, label=f'PWM Right ({name2})')
        ax5.set_title('PWM Signals Comparison')
        ax5.set_xlabel('Time (ms)')
        ax5.set_ylabel('Duty Cycle (0-1000)')
        ax5.legend(fontsize='small')
        ax5.grid(True)

        # Row 3, Col 2: Battery Voltage
        ax6 = axs[2, 1]
        ax6.plot(d1['time'], d1['battery'], color=c1_act, label=name1)
        ax6.plot(d2['time'], d2['battery'], color=c2_act, label=name2)
        ax6.set_title('Battery Voltage Comparison')
        ax6.set_xlabel('Time (ms)')
        ax6.set_ylabel('mV')
        ax6.legend(fontsize='small')
        ax6.grid(True)

        fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def collect_print_and_plot_data():
    """Connects to serial port, pipes text stream, logs to disk, and updates graphs."""
    if CONTROL_LOG_MODE:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'vel_p', 'vel_i', 'ang_p', 'ang_i', 'rotation_ff'
        ]
        header = (
            "Time(ms);ActualLinearVel;TargetLinearVel;ActualAngularVel;TargetAngularVel;"
            "PWML;PWMR;VelP;VelI;AngP;AngI;RotationFF"
        )
    else:
        keys = [
            'time', 'lin_vel_act', 'lin_vel_tgt', 'ang_vel_act', 'ang_vel_tgt',
            'pwm_left', 'pwm_right', 'battery', 'pos_x', 'pos_y', 'angle', 'dist'
        ]
        header = (
            "Time(ms);ActualLinearVel;TargetLinearVel;ActualAngularVel;TargetAngularVel;"
            "PWML;PWMR;Battery(mV);PosX(m);PosY(m);Angle(rad);Dist"
        )

    data_dict = {k: [] for k in keys}
    data_dict['is_control_mode'] = CONTROL_LOG_MODE
    data_lines = []

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT) as ser:
            ser.flush()
            print(f"Successfully connected to {SERIAL_PORT}")
            print("Waiting for data burst... Press the physical button or use Ctrl+C to exit.")
            
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
                    if len(fields) >= 12:
                        data_lines.append(";".join(fields[:12]))
                        for idx, key in enumerate(keys):
                            data_dict[key].append(float(fields[idx]))
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
        # Standard Single Log Viewer
        parsed_data = parse_log_file(sys.argv[1])
        if parsed_data:
            plot_single(parsed_data, title_suffix=f"({os.path.basename(sys.argv[1])})")
            
    elif args_count >= 3:
        # Comparison Log Viewer Mode[cite: 3]
        file_one = sys.argv[1]
        file_two = sys.argv[2]
        
        # Check for optional timeline offset alignment parameter
        time_offset = 0.0
        if args_count >= 4:
            try:
                time_offset = float(sys.argv[3])
            except ValueError:
                print(f"Warning: Invalid offset target context '{sys.argv[3]}'. Defaulting to 0.0.")
        
        plot_comparison(file_one, file_two, offset=time_offset)
        
    else:
        collect_print_and_plot_data()