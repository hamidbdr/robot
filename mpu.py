import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import adafruit_tca9548a

# ========================
# Hardware Configuration
# ========================
MUX_ADDR = 0x70
PWM_CHANNEL = 1  # PCA9685 on multiplexer channel 1
IMU_CHANNELS = [7, 0]  # IMUs on channels 7 and 0
SERVO_RANGE = 270
MPU6050_ADDR = 0x68  # Default address (AD0 = GND)

# Calibration (adjust per servo)
SERVO_CALIBRATION = [
    {'channel': 0, 'min_pulse': 500, 'max_pulse': 2500, 'offset': 45},
    {'channel': 1, 'min_pulse': 500, 'max_pulse': 2500, 'offset': 35}
]

# ========================
# I2C Initialization
# ========================
i2c = busio.I2C(board.SCL, board.SDA)
mux = adafruit_tca9548a.TCA9548A(i2c)

# Initialize PCA9685
with mux[PWM_CHANNEL]:
    pca = PCA9685(mux[PWM_CHANNEL])
    pca.frequency = 50
    kit = ServoKit(channels=16, i2c=mux[PWM_CHANNEL])
    
    for cal in SERVO_CALIBRATION:
        kit.servo[cal['channel']].actuation_range = SERVO_RANGE
        kit.servo[cal['channel']].set_pulse_width_range(
            cal['min_pulse'], cal['max_pulse']
        )

# ========================
# IMU Initialization
# ========================
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C

def verify_imu(channel):
    """Check if IMU is present on specified channel"""
    try:
        with mux[channel]:
            if not i2c.try_lock():
                return False
            addresses = i2c.scan()
            i2c.unlock()
            return MPU6050_ADDR in addresses
    except Exception as e:
        print(f"Channel {channel} scan error: {str(e)}")
        return False

def imu_init(channel):
    """Initialize IMU with retries"""
    for attempt in range(3):
        try:
            with mux[channel]:
                # Wake up device
                i2c.writeto(MPU6050_ADDR, bytes([PWR_MGMT_1, 0]))
                # Set accelerometer range to ±2g
                i2c.writeto(MPU6050_ADDR, bytes([ACCEL_CONFIG, 0x00]))
                time.sleep(0.1)
                return True
        except Exception as e:
            print(f"IMU {channel} init attempt {attempt+1} failed: {str(e)}")
            time.sleep(0.1)
    return False

# Verify and initialize IMUs
valid_imu_channels = []
for channel in IMU_CHANNELS:
    if verify_imu(channel):
        if imu_init(channel):
            valid_imu_channels.append(channel)
            print(f"IMU on channel {channel} initialized successfully")
        else:
            print(f"Failed to initialize IMU on channel {channel}")
    else:
        print(f"No IMU found on channel {channel}")

if not valid_imu_channels:
    raise RuntimeError("No valid IMUs found!")

# ========================
# Control Logic
# ========================
def read_imu(channel):
    """Read accelerometer data with retries"""
    for attempt in range(3):
        try:
            with mux[channel]:
                data = bytearray(6)
                i2c.writeto_then_readfrom(
                    MPU6050_ADDR,
                    bytes([0x3B]),
                    data
                )
                return {
                    'x': (data[0] << 8 | data[1]) / 16384.0,
                    'y': (data[2] << 8 | data[3]) / 16384.0,
                    'z': (data[4] << 8 | data[5]) / 16384.0
                }
        except Exception as e:
            print(f"IMU {channel} read attempt {attempt+1} failed: {str(e)}")
            time.sleep(0.01)
    return None

def update_servo(servo_channel, angle):
    """Update servo position"""
    angle = max(0, min(SERVO_RANGE, angle))
    try:
        with mux[PWM_CHANNEL]:
            kit.servo[servo_channel].angle = angle
    except Exception as e:
        print(f"Servo {servo_channel} error: {str(e)}")

# ========================
# Main Control Loop
# ========================
try:
    print("IMU-Servo Control Active")
    while True:
        for servo_idx, cal in enumerate(SERVO_CALIBRATION):
            if servo_idx >= len(valid_imu_channels):
                continue
                
            imu_channel = valid_imu_channels[servo_idx]
            accel = read_imu(imu_channel)
            
            if accel:
                target_angle = 135 + (accel['x'] * 45) + cal['offset']
                update_servo(cal['channel'], target_angle)
        
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nResetting servos...")
    for cal in SERVO_CALIBRATION:
        update_servo(cal['channel'], 80 + cal['offset'])
    time.sleep(1)
    pca.deinit()
    print("Safe shutdown complete")