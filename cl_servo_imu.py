import time
import math
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import smbus2
from collections import deque

# Hardware Configuration
MUX_ADDR = 0x70       # Multiplexer address
PWM_ADDR = 0x40       # PCA9685 address
MPU_ADDR = 0x68       # MPU-6050 address
MPU_CHANNEL = 7       # Using sensor on channel 7
SERVO_CHANNEL = 1     # Channel where PCA9685 is connected

# Servo Configuration
SERVO_RANGE = 270     # Maximum servo rotation
SERVO_CALIBRATION = [
    {'channel': 0, 'min_pulse': 500, 'max_pulse': 2500, 'offset': 45},
    {'channel': 1, 'min_pulse': 500, 'max_pulse': 2500, 'offset': 35}
]

# Smoothing Configuration
SMOOTHING_WINDOW = 5  # Number of samples for moving average
MIN_CHANGE = 2.0      # Minimum degrees change to update servos

# Initialize I2C buses
i2c = busio.I2C(board.SCL, board.SDA)
bus = smbus2.SMBus(1)

# Initialize smoothing buffer
pitch_buffer = deque(maxlen=SMOOTHING_WINDOW)
last_servo_angle = 0

def select_channel(channel):
    """Select I2C channel on multiplexer"""
    bus.write_byte(MUX_ADDR, 1 << channel)
    time.sleep(0.01)

def mpu6050_init():
    """Initialize MPU-6050 sensor"""
    select_channel(MPU_CHANNEL)
    bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)  # Wake up MPU6050
    time.sleep(0.1)

def read_word_2c(addr):
    """Read 16-bit signed value from MPU6050"""
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def get_smoothed_pitch():
    """Get smoothed pitch angle from MPU6050"""
    select_channel(MPU_CHANNEL)
    accel_x = read_word_2c(0x3B) / 16384.0
    accel_y = read_word_2c(0x3D) / 16384.0
    accel_z = read_word_2c(0x3F) / 16384.0
    pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * (180/math.pi)
    pitch_buffer.append(pitch)
    return sum(pitch_buffer) / len(pitch_buffer) if pitch_buffer else pitch

def map_angle(pitch):
    """Convert pitch angle to servo angle (REVERSED)"""
    # Changed calculation to reverse the response
    return int(270 - (pitch + 90) * 1.5)  # Now -90°→270° and 90°→0°

# Initialize PCA9685
select_channel(SERVO_CHANNEL)
time.sleep(0.1)
pca = PCA9685(i2c, address=PWM_ADDR)
pca.frequency = 50
kit = ServoKit(channels=16, i2c=i2c, address=PWM_ADDR)

# Setup servos
for cal in SERVO_CALIBRATION:
    kit.servo[cal['channel']].actuation_range = SERVO_RANGE
    kit.servo[cal['channel']].set_pulse_width_range(cal['min_pulse'], cal['max_pulse'])

# Initialize MPU6050
mpu6050_init()

print("System ready - MPU6050 controlling both servos (REVERSED RESPONSE)")
print("Press Ctrl+C to exit")

try:
    while True:
        pitch = get_smoothed_pitch()
        servo_angle = map_angle(pitch)
        
        # Only update if change is significant
        if abs(servo_angle - last_servo_angle) >= MIN_CHANGE:
            select_channel(SERVO_CHANNEL)
            time.sleep(0.01)
            
            # Move both servos
            for cal in SERVO_CALIBRATION:
                calibrated_angle = max(0, min(SERVO_RANGE, servo_angle + cal['offset']))
                kit.servo[cal['channel']].angle = calibrated_angle
            
            last_servo_angle = servo_angle
            print(f"Pitch: {pitch:6.1f}° → Servos: {servo_angle:3d}°")
        
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nCentering servos...")
    select_channel(SERVO_CHANNEL)
    for cal in SERVO_CALIBRATION:
        kit.servo[cal['channel']].angle = map_angle(30) + cal['offset']
    time.sleep(1)
    pca.deinit()
    print("System off")