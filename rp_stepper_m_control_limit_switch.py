from gpiozero import OutputDevice, Button
import time
import threading

# Define pins for stepper motor 1
DIR_PIN_1 = OutputDevice(27)
STEP_PIN_1 = OutputDevice(17)
ENABLE_PIN_1 = OutputDevice(22)

# Define pins for stepper motor 2
DIR_PIN_2 = OutputDevice(20)
STEP_PIN_2 = OutputDevice(16)
ENABLE_PIN_2 = OutputDevice(21)

# Limit switch pin (use Button instead of InputDevice)
LIMIT_SWITCH = Button(12, pull_up=True)

# Stop flag
stop_flag = threading.Event()

def disable_motors():
    ENABLE_PIN_1.value = True  # Assuming HIGH disables the motor
    ENABLE_PIN_2.value = True  # Assuming HIGH disables the motor

def enable_motors():
    ENABLE_PIN_1.value = False  # Assuming LOW enables the motor
    ENABLE_PIN_2.value = False  # Assuming LOW enables the motor

def set_directions(clockwise_motor1, clockwise_motor2):
    DIR_PIN_1.value = clockwise_motor1  # True for CW, False for CCW
    DIR_PIN_2.value = clockwise_motor2  # True for CW, False for CCW
    print(f"Motor 1 Direction: {'CW' if clockwise_motor1 else 'CCW'}, Motor 2 Direction: {'CW' if clockwise_motor2 else 'CCW'}")

def rpm_to_delay(rpm, steps_per_revolution):
    if rpm > 0:
        return 60 / (rpm * steps_per_revolution)
    else:
        return float('inf')  # Very high delay when RPM is zero

def move_steps_motor(steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, 
                     decel_fraction, rpm_min, stop_event, step_pin, motor_id):
    accel_steps = int(steps * accel_fraction)
    decel_steps = int(steps * decel_fraction)
    hold_steps = steps - accel_steps - decel_steps

    hold_start_time = None
    hold_end_time = None

    for step in range(accel_steps):
        rpm = rpm_start + (rpm_max - rpm_start) * (step / accel_steps)
        rpm = max(rpm, rpm_min)  
        delay = rpm_to_delay(rpm, steps_per_revolution)
        if stop_event.is_set():
            print(f"Limit switch activated! Stopping Motor {motor_id}...")
            return "stop"

        step_pin.on()
        time.sleep(delay)
        step_pin.off()
        time.sleep(delay)

    hold_start_time = time.time()
    for step in range(hold_steps):
        delay = rpm_to_delay(rpm_max, steps_per_revolution)  
        if stop_event.is_set():
            print(f"Limit switch activated! Stopping Motor {motor_id}...")
            return "stop"

        step_pin.on()
        time.sleep(delay)
        step_pin.off()
        time.sleep(delay)
    hold_end_time = time.time()

    for step in range(decel_steps):
        rpm = rpm_max - (rpm_max - rpm_min) * (step / decel_steps)  
        rpm = max(rpm, rpm_min)
        delay = rpm_to_delay(rpm, steps_per_revolution)
        if stop_event.is_set():
            print(f"Limit switch activated! Stopping Motor {motor_id}...")
            return "stop"
        step_pin.on()
        time.sleep(delay)
        step_pin.off()
        time.sleep(delay)

    hold_duration = hold_end_time - hold_start_time
    revolutions = hold_steps / steps_per_revolution
    real_rpm = (revolutions / hold_duration) * 60 if hold_duration > 0 else 0
    print(f"Motor {motor_id} Real RPM in Hold Phase: {real_rpm:.2f}")
    return real_rpm

def move_both_motors(steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min):
    thread_motor1 = threading.Thread(target=move_steps_motor,
                                     args=(1*steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min, stop_flag, STEP_PIN_1, 1))
    thread_motor2 = threading.Thread(target=move_steps_motor,
                                      args=(4*steps, steps_per_revolution, rpm_start, 1*rpm_max, accel_fraction/1, decel_fraction/1, rpm_min, stop_flag, STEP_PIN_2, 2))

    thread_motor1.start()
    thread_motor2.start()

    thread_motor1.join()
    thread_motor2.join()

def limit_switch_pressed():
    stop_flag.set()  # Stop the motors when the limit switch is pressed
    #print("Limit switch pressed. Stopping motors...")

def limit_switch_released():
    stop_flag.clear()  # Clear the stop flag when the limit switch is released
    #print("Limit switch released. Ready to continue...")

# Setup gpiozero events for the limit switch
LIMIT_SWITCH.when_pressed = limit_switch_pressed
LIMIT_SWITCH.when_released = limit_switch_released


# Main execution
try:
    angle = 40
    numb_full_rota = angle / 360
    reduction_factor = 9.5
    micro_stepping = 32
    steps_per_revolution = micro_stepping * 200

    steps_motor = int(steps_per_revolution * numb_full_rota * reduction_factor)

    rpm_start = 0.5
    rpm_max = 37
    accel_fraction = 0.005

    decel_fraction = 0.005
    rpm_min = 0.5

    enable_motors()
    for i in range(4):
        print(f"iteration number {i}")

        set_directions(clockwise_motor1=True, clockwise_motor2=True)

        move_both_motors(steps_motor, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min)

        time.sleep(0.05)

        set_directions(clockwise_motor1=False, clockwise_motor2=False)                
        move_both_motors(steps_motor, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min)

finally:
    #disable_motors()
    #print("Motors disabled!")
    print("done")
