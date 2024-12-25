from gpiozero import OutputDevice
import time
import matplotlib.pyplot as plt

# Define pins
DIR_PIN = OutputDevice(27)
STEP_PIN = OutputDevice(17)
ENABLE_PIN = OutputDevice(22)

def disable_motor():
    ENABLE_PIN.value = True  # Assuming HIGH disables the motor

def enable_motor():
    ENABLE_PIN.value = False  # Assuming LOW enables the motor

# Function to control direction
def set_direction(clockwise):
    DIR_PIN.value = clockwise  # True for CW, False for CCW

# Function to calculate delay from RPM
def rpm_to_delay(rpm, steps_per_revolution):
    if rpm > 0:
        return 60 / (rpm * steps_per_revolution) #/ micro_stepping
    else:
        return float('inf')  # Very high delay when RPM is zero

# Function to move the stepper motor
def move_steps_with_linear_rpm(steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min=1):
    accel_steps = int(steps * accel_fraction)
    decel_steps = int(steps * decel_fraction)
    hold_steps = steps - accel_steps - decel_steps

    time_data = []
    rpm_data = []
    delay_data = []

    total_steps_completed = 0
    start_time = time.time()

    # Acceleration phase
    for step in range(accel_steps):
        elapsed_time = time.time() - start_time
        rpm = rpm_start + (rpm_max - rpm_start) * (step / accel_steps)
        rpm = max(rpm, rpm_min)
        delay = rpm_to_delay(rpm, steps_per_revolution)

        STEP_PIN.on()
        time.sleep(delay)
        STEP_PIN.off()
        time.sleep(delay)

        total_steps_completed += 1
        time_data.append(elapsed_time)
        rpm_data.append(rpm)
        delay_data.append(delay)

    # Hold phase
    for step in range(hold_steps):
        elapsed_time = time.time() - start_time
        rpm = rpm_max
        delay = rpm_to_delay(rpm, steps_per_revolution)/4

        STEP_PIN.on()
        time.sleep(delay)
        STEP_PIN.off()
        time.sleep(delay)

        total_steps_completed += 1
        time_data.append(elapsed_time)
        rpm_data.append(rpm)
        delay_data.append(delay)

    # Deceleration phase
    for step in range(decel_steps):
        elapsed_time = time.time() - start_time
        rpm = rpm_max - (rpm_max - rpm_min) * (step / decel_steps)
        rpm = max(rpm, rpm_min)
        delay = rpm_to_delay(rpm, steps_per_revolution)

        STEP_PIN.on()
        time.sleep(delay)
        STEP_PIN.off()
        time.sleep(delay)

        total_steps_completed += 1
        time_data.append(elapsed_time)
        rpm_data.append(rpm)
        delay_data.append(delay)

    print(f"Final RPM: {rpm_data[-1]:.2f}, execution time {time.time() - start_time:.2f} seconds")
    return time_data, rpm_data, delay_data

# Function to compute real RPM during the hold region
def compute_real_rpm(time_data, steps_per_revolution, hold_start_index, hold_end_index):
    hold_steps = hold_end_index - hold_start_index
    revolutions = hold_steps / steps_per_revolution
    hold_time = time_data[hold_end_index] - time_data[hold_start_index]
    hold_time_minutes = hold_time / 60
    real_rpm = revolutions / hold_time_minutes
    return real_rpm

# Function to plot data
def plots_with_labels(time_data_all, rpm_data_all, delay_data_all):
    delay_data_all = [[d * 1000*1000 for d in delay_data] for delay_data in delay_data_all]

    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    for i, (time_data, rpm_data) in enumerate(zip(time_data_all, rpm_data_all)):
        plt.plot(time_data, rpm_data, label=f"RPM Iteration {i+1}")
    plt.xlabel("Time (s)")
    plt.ylabel("RPM")
    plt.title("Motor Speed (RPM) vs Time")
    plt.legend()
    plt.ylim(bottom=0)

    plt.subplot(2, 1, 2)
    for i, (time_data, delay_data) in enumerate(zip(time_data_all, delay_data_all)):
        plt.plot(time_data, delay_data, label=f"Step Delay Iteration {i+1}")
    plt.xlabel("Time (s)")
    plt.ylabel("Delay (micro second)")
    plt.title("Step Delay vs Time")
    plt.legend()
    plt.ylim(bottom=0)

    plt.tight_layout()
    plt.show()

# Main execution
try:
    # Parameters
    numb_full_rota = 0.3
    reduction_factor = 9.5
    micro_stepping = 4
    steps_per_revolution = micro_stepping * 200

    steps = int(steps_per_revolution * numb_full_rota * reduction_factor)
    rpm_start = 50
    rpm_max = 350
    acceleration_fraction = 0.3
    deceleration_fraction = 0.2
    hold_fraction = 1 - (acceleration_fraction + deceleration_fraction)

    enable_motor()

    all_time_data = []
    all_rpm_data = []
    all_delay_data = []
    real_rpms = []

    for i in range(4):
        print(f"Starting clockwise motion (iteration {i+1})...")
        set_direction(True)
        time_data, rpm_data, delay_data = move_steps_with_linear_rpm(
            steps, steps_per_revolution, rpm_start, rpm_max, acceleration_fraction, deceleration_fraction
        )
        all_time_data.append(time_data)
        all_rpm_data.append(rpm_data)
        all_delay_data.append(delay_data)

        hold_start_index = int(len(time_data) * acceleration_fraction)
        hold_end_index = hold_start_index + int(len(time_data) * hold_fraction)

        real_rpm = compute_real_rpm(time_data, steps_per_revolution, hold_start_index, hold_end_index)
        real_rpms.append(real_rpm)

        print(f"Real RPM during hold phase (CW, iteration {i+1}): {real_rpm:.2f}")

        print(f"Starting counterclockwise motion (iteration {i+1})...")
        
        time.sleep(0.1)

        set_direction(False)
        time_data, rpm_data, delay_data = move_steps_with_linear_rpm(
            steps, steps_per_revolution, rpm_start, rpm_max, acceleration_fraction, deceleration_fraction
        )
        all_time_data.append(time_data)
        all_rpm_data.append(rpm_data)
        all_delay_data.append(delay_data)

        hold_start_index = int(len(time_data) * acceleration_fraction)
        hold_end_index = hold_start_index + int(len(time_data) * hold_fraction)

        real_rpm = compute_real_rpm(time_data, steps_per_revolution, hold_start_index, hold_end_index)
        real_rpms.append(real_rpm)

        print(f"Real RPM during hold phase (CCW, iteration {i+1}): {real_rpm:.2f}")

    plots_with_labels(all_time_data, all_rpm_data, all_delay_data)

    average_real_rpm = sum(real_rpms) / len(real_rpms)
    print(f"Average Real RPM across all iterations: {average_real_rpm:.2f}")

finally:
    disable_motor()
    time.sleep(0.1)
    print("Motor disabled!")
