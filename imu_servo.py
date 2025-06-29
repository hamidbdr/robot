#!/usr/bin/env python3
import time
import math
import smbus

# I2C Addresses
MUX_ADDR = 0x70    # TCA9548A multiplexer
MPU_ADDR = 0x68    # MPU6050 sensors
PCA_ADDR = 0x40    # PCA9685 servo controller

# MPU6050 Registers
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
PWR_MGMT_1 = 0x6B

# PCA9685 Registers
PCA_MODE1 = 0x00
PCA_PRESCALE = 0xFE
LED0_ON_L = 0x06

# Servo Pulse Range (adjust for your specific servos if needed)
SERVO_MIN = 150  # Min pulse length out of 4096
SERVO_MAX = 600  # Max pulse length out of 4096

def select_mux_channel(bus, mux_addr, channel):
    """Select channel on TCA9548A multiplexer"""
    bus.write_byte(mux_addr, 1 << channel)
    time.sleep(0.01)  # Brief delay after switching channels

def read_word_2c(bus, addr, reg):
    """Read a word from a register and handle two's complement"""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def get_rotation(accel_x, accel_y, accel_z, axis='x'):
    """Calculate rotation angle from accelerometer data"""
    if axis.lower() == 'x':
        # Roll angle (X rotation)
        dist = math.sqrt((accel_y * accel_y) + (accel_z * accel_z))
        if dist == 0:
            return 0
        angle = math.atan2(accel_x, dist)
    else:
        # Pitch angle (Y rotation)
        dist = math.sqrt((accel_x * accel_x) + (accel_z * accel_z))
        if dist == 0:
            return 0
        angle = math.atan2(accel_y, dist)
        
    return math.degrees(angle)

def init_pca9685(bus, addr):
    """Initialize the PCA9685 servo controller"""
    # Reset PCA9685
    bus.write_byte_data(addr, PCA_MODE1, 0x00)
    time.sleep(0.05)
    
    # Set PWM frequency to 50Hz (for servos)
    prescale = int(25000000.0 / (4096.0 * 50.0) - 0.5)
    old_mode = bus.read_byte_data(addr, PCA_MODE1)
    new_mode = (old_mode & 0x7F) | 0x10  # Sleep
    bus.write_byte_data(addr, PCA_MODE1, new_mode)  # Go to sleep
    bus.write_byte_data(addr, PCA_PRESCALE, prescale)
    bus.write_byte_data(addr, PCA_MODE1, old_mode)
    time.sleep(0.005)
    bus.write_byte_data(addr, PCA_MODE1, old_mode | 0xA0)  # Auto increment

def set_servo_angle(bus, addr, channel, angle):
    """Set a servo to a specific angle (0-180)"""
    # Ensure angle is within bounds
    angle = max(0, min(180, angle))
    
    # Map angle to pulse length
    pulse = int(SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180.0)
    
    # Set PWM values
    bus.write_byte_data(addr, LED0_ON_L + 4 * channel, 0 & 0xFF)
    bus.write_byte_data(addr, LED0_ON_L + 4 * channel + 1, 0 >> 8)
    bus.write_byte_data(addr, LED0_ON_L + 4 * channel + 2, pulse & 0xFF)
    bus.write_byte_data(addr, LED0_ON_L + 4 * channel + 3, pulse >> 8)

def read_mpu_angles(bus, mux_addr, mpu_addr, mux_channel):
    """Read angles from MPU6050 sensor"""
    # Select multiplexer channel
    select_mux_channel(bus, mux_addr, mux_channel)
    
    # Read accelerometer data
    accel_x = read_word_2c(bus, mpu_addr, ACCEL_XOUT_H)
    accel_y = read_word_2c(bus, mpu_addr, ACCEL_YOUT_H)
    accel_z = read_word_2c(bus, mpu_addr, ACCEL_ZOUT_H)
    
    # Calculate roll and pitch
    x_angle = get_rotation(accel_x, accel_y, accel_z, 'x')
    y_angle = get_rotation(accel_x, accel_y, accel_z, 'y')
    
    # Map angles from -90:90 to 0:180 for servos
    servo_x = min(max(x_angle + 90, 0), 180)
    servo_y = min(max(y_angle + 90, 0), 180)
    
    return servo_x, servo_y

def init_mpus(bus, mux_addr, mpu_addr, channels):
    """Initialize all MPU6050 sensors"""
    for channel in channels:
        select_mux_channel(bus, mux_addr, channel)
        # Wake up the MPU6050 (clear sleep bit)
        bus.write_byte_data(mpu_addr, PWR_MGMT_1, 0)
        time.sleep(0.1)  # Wait for wake-up
        
        # Verify wake-up was successful
        pwr_state = bus.read_byte_data(mpu_addr, PWR_MGMT_1)
        print(f"MPU on channel {channel} - power state: {pwr_state} (should be 0)")
        
def main():
    print("Starting MPU6050 Servo Control System")
    
    # Initialize I2C bus (use '1' for Raspberry Pi)
    bus = smbus.SMBus(1)
    
    # Initialize MPU6050 sensors on channels 0 and 7
    print("Initializing MPU6050 sensors...")
    init_mpus(bus, MUX_ADDR, MPU_ADDR, [0, 7])
    
    # Initialize PCA9685 on channel 1
    print("Initializing PCA9685 servo controller...")
    select_mux_channel(bus, MUX_ADDR, 1)
    init_pca9685(bus, PCA_ADDR)
    
    # Set servos to neutral position initially
    print("Setting servos to neutral position...")
    select_mux_channel(bus, MUX_ADDR, 1)
    set_servo_angle(bus, PCA_ADDR, 0, 90)
    set_servo_angle(bus, PCA_ADDR, 1, 90)
    time.sleep(1)
    
    # Create moving average arrays for smoothing
    readings_ch0 = [90] * 5
    readings_ch7 = [90] * 5
    
    print("Starting control loop. Press Ctrl+C to exit.")
    
    try:
        while True:
            # Read angles from MPU on channel 0
            x_angle_ch0, _ = read_mpu_angles(bus, MUX_ADDR, MPU_ADDR, 0)
            
            # Read angles from MPU on channel 7
            _, y_angle_ch7 = read_mpu_angles(bus, MUX_ADDR, MPU_ADDR, 7)
            
            # Update moving averages
            readings_ch0.pop(0)
            readings_ch0.append(x_angle_ch0)
            readings_ch7.pop(0)
            readings_ch7.append(y_angle_ch7)
            
            # Calculate smooth values
            smooth_ch0 = sum(readings_ch0) / len(readings_ch0)
            smooth_ch7 = sum(readings_ch7) / len(readings_ch7)
            
            # Apply deadzone (reduce jitter when near neutral)
            if 85 <= smooth_ch0 <= 95:
                smooth_ch0 = 90
            if 85 <= smooth_ch7 <= 95:
                smooth_ch7 = 90

            # Update servos
            select_mux_channel(bus, MUX_ADDR, 1)
            set_servo_angle(bus, PCA_ADDR, 0, smooth_ch0)
            set_servo_angle(bus, PCA_ADDR, 1, smooth_ch7)
            
            print(f"MPU Ch0: {x_angle_ch0:.1f}° → {smooth_ch0:.1f}° | MPU Ch7: {y_angle_ch7:.1f}° → {smooth_ch7:.1f}°", end="\r")
            
            time.sleep(0.05)  # 20Hz update rate
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
        # Return servos to neutral position
        select_mux_channel(bus, MUX_ADDR, 1)
        set_servo_angle(bus, PCA_ADDR, 0, 90)
        set_servo_angle(bus, PCA_ADDR, 1, 90)

if __name__ == "__main__":
    main()