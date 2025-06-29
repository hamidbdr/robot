from gpiozero import OutputDevice
import time

# Define pins
# Define pins for stepper motor 3
DIR_PIN = OutputDevice(24)
STEP_PIN = OutputDevice(23)
ENABLE_PIN = OutputDevice(16)

DIR_PIN = OutputDevice(19)
STEP_PIN = OutputDevice(27)
#ENABLE_PIN = OutputDevice(6)

# Function to enable/disable the motor
def enable_motor(enable):
    ENABLE_PIN.value = not enable  # Set to False to enable

# Function to control direction
def set_direction(clockwise):
    DIR_PIN.value = clockwise  # True for CW, False for CCW

# Function to send steps
def move_steps(steps, delay):
    for step in range(steps):
        STEP_PIN.on()
        time.sleep(delay)  # High pulse
        STEP_PIN.off()
        time.sleep(delay)  # Low pulse
try:
    for i in range(40):

        angle = 90
        numb_full_rota = angle / 360
        reduction_factor = 6.5
        micro_stepping = 16
        steps_per_revolution = micro_stepping * 200

        steps = int(steps_per_revolution * numb_full_rota * reduction_factor)

        step_delay = 0.001

        enable_motor(True)  # Enable motor
        DIR_PIN.value = False
        print(f"Cycle {i + 1}: Clockwise")
        move_steps(steps, step_delay)  # Move steps
        time.sleep(2)

        DIR_PIN.value = True
        print(f"Cycle {i + 1}: Counterclockwise")
        move_steps(steps, step_delay)  # Move steps 

        time.sleep(2)

finally:
    enable_motor(False)  # Disable motor
    print("Motor disabled!")
