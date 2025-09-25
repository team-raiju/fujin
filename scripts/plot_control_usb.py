import serial
import matplotlib.pyplot as plt
import sys
import signal

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
READ_TIMEOUT = 1.0

def signal_handler(sig, frame):
    """Handles the Ctrl+C signal to ensure a clean exit."""
    print("\nCtrl+C detected! Closing program.")
    sys.exit(0)

def collect_print_and_plot_data():
    """
    Connects to the serial port, collects data, prints it, and generates plots.
    """
    # --- Lists to store all data fields ---
    time_data, linear_velocity_actual, linear_velocity_target = [], [], []
    angular_velocity_actual, angular_velocity_target = [], []
    pwm_left, pwm_right = [], []
    battery, pos_x, pos_y, angle, dist = [], [], [], [], []

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT) as ser:
            ser.flush()
            print(f"Successfully connected to {SERIAL_PORT}")

            print("Waiting for data burst... Press the physical button or use Ctrl+C to exit.")
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print("Data reception started...")
                    break
            
            while line:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    break # Exit if timeout occurs
                
                try:
                    fields = line.split(';')
                    if len(fields) >= 12:
                        time_data.append(float(fields[0]))
                        linear_velocity_actual.append(float(fields[1]))
                        linear_velocity_target.append(float(fields[2]))
                        angular_velocity_actual.append(float(fields[3]))
                        angular_velocity_target.append(float(fields[4]))
                        pwm_left.append(float(fields[5]))
                        pwm_right.append(float(fields[6]))
                        battery.append(float(fields[7]))
                        pos_x.append(float(fields[8]))
                        pos_y.append(float(fields[9]))
                        angle.append(float(fields[10]))
                        dist.append(float(fields[11]))
                except (ValueError, IndexError) as e:
                    print(f"Skipping malformed line '{line}': {e}")
            
            print(f"Data collection complete. Received {len(time_data)} data points.")
        print(f"Serial port {SERIAL_PORT} closed.")

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
        return

    if not time_data:
        print("No data was collected. Exiting.")
        return

    # --- Print all collected data to the terminal ---
    print("\n--- Collected Data ---")
    header = (
        "Time(ms);ActualLinearVel;TargetLinearVel;ActualAngularVel;TargetAngularVel;"
        "PWML;PWMR;Battery(mV);PosX(m);PosY(m);Angle(rad);Dist"
    )
    print(header)
    for i in range(len(time_data)):
        print(
            f"{time_data[i]:.2f};"
            f"{linear_velocity_actual[i]:.4f};{linear_velocity_target[i]:.4f};"
            f"{angular_velocity_actual[i]:.4f};{angular_velocity_target[i]:.4f};"
            f"{pwm_left[i]:.0f};{pwm_right[i]:.0f};"
            f"{battery[i]:.0f};{pos_x[i]:.4f};{pos_y[i]:.4f};"
            f"{angle[i]:.4f};{dist[i]:.4f}"
        )
    print("----------------------\n")

    # --- Create Figure 1: Velocities and PWMs ---
    fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
    fig1.suptitle('Motion Control Performance', fontsize=16)
    ax1.plot(time_data, linear_velocity_actual, label='Actual Linear Velocity')
    ax1.plot(time_data, linear_velocity_target, label='Target', linestyle='--')
    ax1.set_title('Linear Velocities'); ax1.set_xlabel('Time (ms)'); ax1.set_ylabel('m/s'); ax1.legend(); ax1.grid(True)
    ax2.plot(time_data, angular_velocity_actual, label='Actual Angular Velocity')
    ax2.plot(time_data, angular_velocity_target, label='Target', linestyle='--')
    ax2.set_title('Angular Velocities'); ax2.set_xlabel('Time (ms)'); ax2.set_ylabel('rad/s'); ax2.legend(); ax2.grid(True)
    ax3.plot(time_data, pwm_left, label='PWM Left')
    ax3.plot(time_data, pwm_right, label='PWM Right')
    ax3.set_title('PWM Signals'); ax3.set_xlabel('Time (ms)'); ax3.set_ylabel('Duty Cycle (0-1000)'); ax3.legend(); ax3.grid(True)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])

    # --- Create Figure 2: Battery, Distance, and Angle ---
    fig2, (ax4, ax5, ax6) = plt.subplots(3, 1, figsize=(12, 18))
    fig2.suptitle('System and Sensor Data', fontsize=16)
    ax4.plot(time_data, battery, label='Battery Voltage')
    ax4.set_title('Battery Voltage over Time'); ax4.set_xlabel('Time (ms)'); ax4.set_ylabel('Voltage (mV)'); ax4.legend(); ax4.grid(True)
    ax5.plot(time_data, dist, label='Distance')
    ax5.set_title('Distance over Time'); ax5.set_xlabel('Time (ms)'); ax5.set_ylabel('Distance'); ax5.legend(); ax5.grid(True)
    ax6.plot(time_data, angle, label='Angle')
    ax6.set_title('Angle over Time'); ax6.set_xlabel('Time (ms)'); ax6.set_ylabel('Angle (rad)'); ax6.legend(); ax6.grid(True)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Show all plot windows
    plt.show()

if __name__ == "__main__":
    # Register the signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    collect_print_and_plot_data()