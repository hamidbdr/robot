import smbus2
import time
import math

# I2C Configuration
PCA9548A_ADDR = 0x70  # Multiplexer address
MPU6050_ADDR = 0x68    # MPU-6050 address
bus = smbus2.SMBus(1)  # I2C bus 1

# MPU-6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F

def select_channel(channel):
    """Select I2C channel on multiplexer"""
    bus.write_byte(PCA9548A_ADDR, 1 << channel)

def mpu6050_init():
    """Initialize MPU-6050"""
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)  # Wake up device

def read_accel(channel):
    """Read accelerometer data"""
    select_channel(channel)
    
    # Read raw accelerometer values
    accel_x = read_word_2c(ACCEL_XOUT)
    accel_y = read_word_2c(ACCEL_YOUT)
    accel_z = read_word_2c(ACCEL_ZOUT)
    
    # Convert to g-forces (±2g range)
    accel_x_g = accel_x / 16384.0
    accel_y_g = accel_y / 16384.0
    accel_z_g = accel_z / 16384.0
    
    return accel_x_g, accel_y_g, accel_z_g

def read_word_2c(addr):
    """Read 16-bit signed value"""
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def calculate_angles(accel_x, accel_y, accel_z):
    """Calculate pitch and roll in degrees (-180 to 180)"""
    pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * (180/math.pi)
    roll = math.atan2(accel_y, accel_z) * (180/math.pi)
    return pitch, roll

def convert_to_360(pitch, roll):
    """Convert -180 to 180 range to 0-360 degrees"""
    pitch_360 = pitch + 360 if pitch < 0 else pitch
    roll_360 = roll + 360 if roll < 0 else roll
    return pitch_360, roll_360

# Initialize system
channels = [7, 2, 0]  # Your multiplexer channels
for channel in channels:
    select_channel(channel)
    mpu6050_init()
    time.sleep(0.1)

try:
    print("Reading MPU-6050 sensors (Ctrl+C to stop)")
    while True:
        for channel in channels:
            channel = 7
            # Read accelerometer
            accel_x, accel_y, accel_z = read_accel(channel)
            
            # Calculate angles
            pitch, roll = calculate_angles(accel_x, accel_y, accel_z)
            pitch_360, roll_360 = convert_to_360(pitch, roll)
            
            # Print results
            print(f"\nChannel {channel}:")
            print(f"Pitch: {pitch_360:.1f}° (Raw: {pitch:.1f}°)")
            print(f"Roll: {roll_360:.1f}° (Raw: {roll:.1f}°)")
            print(f"Accel: X={accel_x:.2f}g, Y={accel_y:.2f}g, Z={accel_z:.2f}g")
            print("-" * 40)
        
        time.sleep(0.5)  # Update rate

except KeyboardInterrupt:
    print("\nStopped")