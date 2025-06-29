import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

# Configuration
MUX_ADDR = 0x70
PWM_ADDR = 0x40
MUX_CHANNEL = 1
SERVO_RANGE = 270

# Calibration parameters (adjust these for each servo)
SERVO_CALIBRATION = [
    {'channel': 0, 'min_pulse': 500, 'max_pulse': 2500, 'offset': 45},  # Servo 1
    {'channel': 1, 'min_pulse': 500, 'max_pulse': 2500, 'offset': 35}   # Servo 2
]

def map_angle(angle, offset):
    """Apply calibration offset to angle"""
    calibrated = angle + offset
    return max(0, min(SERVO_RANGE, calibrated))  # Constrain to valid range

# Initialize hardware
i2c = busio.I2C(board.SCL, board.SDA)
i2c.writeto(MUX_ADDR, bytes([1 << MUX_CHANNEL]))
time.sleep(0.1)

pca = PCA9685(i2c, address=PWM_ADDR)
pca.frequency = 50
kit = ServoKit(channels=16, i2c=i2c, address=PWM_ADDR)

# Setup servos with calibration
for cal in SERVO_CALIBRATION:
    kit.servo[cal['channel']].actuation_range = SERVO_RANGE
    kit.servo[cal['channel']].set_pulse_width_range(cal['min_pulse'], cal['max_pulse'])

print(f"Servo control ready (calibrated for {SERVO_RANGE}° range)")

try:
    while True:
        try:
            angle = int(input(f"Enter target angle (0-{SERVO_RANGE}): "))
            if 0 <= angle <= SERVO_RANGE:
                for cal in SERVO_CALIBRATION:
                    calibrated_angle = map_angle(angle, cal['offset'])
                    kit.servo[cal['channel']].angle = calibrated_angle
                print(f"Set to {angle}° (calibrated)")
            else:
                print(f"Angle must be 0-{SERVO_RANGE}")
        except ValueError:
            print("Numbers only please")

except KeyboardInterrupt:
    print("\nCentering servos...")
    for cal in SERVO_CALIBRATION:
        kit.servo[cal['channel']].angle = map_angle(90, cal['offset'])
    time.sleep(1)
    pca.deinit()
    print("Done")