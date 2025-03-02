from gpiozero import OutputDevice
import time

# Define pins
DIR_PIN = OutputDevice(27)
STEP_PIN = OutputDevice(17)
ENABLE_PIN = OutputDevice(22)

# Function to enable/disable the motor
def enable_motor(enable):
    ENABLE_PIN.value = not enable  # Set to False to enable

# Function to control direction
def set_direction(clockwise):
    DIR_PIN.value = clockwise  # True for CW, False for CCW

# Function to send steps
def move_steps_with_rpm(steps, delay):
    for step in range(steps):
        STEP_PIN.on()
        time.sleep(delay)  # High pulse
        STEP_PIN.off()
        time.sleep(delay/2)  # Low pulse
try:
    for i in range(4):
        numb_full_rota = 1  # Number of full revolutions
        micro_stepping = 1  # Microstepping setting on driver
        steps_per_revolution = micro_stepping * 200         
        steps = int(steps_per_revolution * numb_full_rota)

        step_delay = 0.001

        enable_motor(True)  # Enable motor
        set_direction(True)  # Set direction clockwise

        print(f"Cycle {i + 1}: Clockwise")
        move_steps_with_rpm(steps, step_delay)  # Move steps with RPM calculation
        time.sleep(0.5)

        set_direction(False)  # Change direction to counterclockwise
        print(f"Cycle {i + 1}: Counterclockwise")
        move_steps_with_rpm(steps, step_delay, steps_per_revolution)  # Move steps with RPM calculation

        time.sleep(0.5)

finally:
    enable_motor(False)  # Disable motor
    print("Motor disabled!")
