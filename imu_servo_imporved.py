#!/usr/bin/env python3
import time
import math
import smbus
import sys

# Default I2C addresses
MUX_ADDR = 0x70       # TCA9548A multiplexer
MPU_ADDR = 0x68       # MPU6050
PCA_ADDR = 0x40       # PCA9685

# MPU6050 Registers
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
PWR_MGMT_1 = 0x6B

# PCA9685 Registers
PCA_MODE1 = 0x00
PCA_PRESCALE = 0xFE
PCA_LED0_ON_L = 0x06   # First LED register address

# Servo configurations
SERVO_MIN_PULSE = 150   # Min pulse length out of 4096
SERVO_MAX_PULSE = 600   # Max pulse length out of 4096

def check_device(bus, addr):
    """Check if a device is connected at the specified address"""
    try:
        bus.read_byte(addr)
        return True
    except:
        return False

def scan_i2c_devices(bus_num=1):
    """Scan for I2C devices on the specified bus"""
    print(f"Scanning I2C bus {bus_num}...")
    bus = smbus.SMBus(bus_num)
    
    devices_found = []
    
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            devices_found.append(addr)
            print(f"I2C device found at address: 0x{addr:02X}")
        except Exception:
            continue
    
    if not devices_found:
        print("No I2C devices found")
    
    return devices_found

class MPU6050:
    def __init__(self, bus, mux_addr, mpu_addr, mux_channel):
        self.bus = bus
        self.mux_addr = mux_addr
        self.mpu_addr = mpu_addr
        self.mux_channel = mux_channel
        
        # Verify connections
        if not self.initialize():
            print(f"Failed to initialize MPU6050 on multiplexer channel {mux_channel}")
        else:
            print(f"Successfully initialized MPU6050 on multiplexer channel {mux_channel}")
            
    def select_channel(self):
        """Select the appropriate channel on the multiplexer"""
        try:
            self.bus.write_byte(self.mux_addr, 1 << self.mux_channel)
            time.sleep(0.05)  # Small delay to let the multiplexer settle
            return True
        except Exception as e:
            print(f"Error selecting multiplexer channel {self.mux_channel}: {e}")
            return False
        
    def initialize(self):
        """Initialize the MPU6050 sensor"""
        try:
            if not self.select_channel():
                return False
                
            # Check if MPU is connected
            try:
                self.bus.read_byte_data(self.mpu_addr, PWR_MGMT_1)
            except:
                print(f"No MPU6050 found at address 0x{self.mpu_addr:02X} on mux channel {self.mux_channel}")
                return False
                
            # Wake up the MPU6050
            self.bus.write_byte_data(self.mpu_addr, PWR_MGMT_1, 0)
            time.sleep(0.1)
            
            # Verify wake up
            power_state = self.bus.read_byte_data(self.mpu_addr, PWR_MGMT_1)
            if power_state != 0:
                print(f"MPU6050 on channel {self.mux_channel} did not wake up properly")
                return False
                
            print(f"MPU6050 initialized on multiplexer channel {self.mux_channel}")
            return True
        except Exception as e:
            print(f"Error initializing MPU6050 on channel {self.mux_channel}: {e}")
            return False
        
    def read_word_2c(self, reg):
        """Read a word from the MPU, handling two's complement"""
        if not self.select_channel():
            return 0
            
        try:
            high = self.bus.read_byte_data(self.mpu_addr, reg)
            low = self.bus.read_byte_data(self.mpu_addr, reg + 1)
            val = (high << 8) + low
            if val >= 0x8000:
                return -((65535 - val) + 1)
            else:
                return val
        except Exception as e:
            print(f"Error reading from MPU6050 on channel {self.mux_channel}: {e}")
            return 0
            
    def get_x_rotation(self, accel_x, accel_y, accel_z):
        """Calculate X-axis rotation from accelerometer data"""
        dist = math.sqrt((accel_y * accel_y) + (accel_z * accel_z))
        if dist == 0:
            return 0
        angle = math.atan2(accel_x, dist)
        return math.degrees(angle)
        
    def get_y_rotation(self, accel_x, accel_y, accel_z):
        """Calculate Y-axis rotation from accelerometer data"""
        dist = math.sqrt((accel_x * accel_x) + (accel_z * accel_z))
        if dist == 0:
            return 0
        angle = math.atan2(accel_y, dist)
        return math.degrees(angle)
        
    def get_servo_angle(self, axis='x'):
        """Get angle values usable for servo positioning"""
        if not self.select_channel():
            return 90  # Return neutral position on error
            
        try:
            accel_x = self.read_word_2c(ACCEL_XOUT_H)
            accel_y = self.read_word_2c(ACCEL_YOUT_H)
            accel_z = self.read_word_2c(ACCEL_ZOUT_H)
            
            if axis.lower() == 'x':
                angle = self.get_x_rotation(accel_x, accel_y, accel_z)
            else:  # y-axis
                angle = self.get_y_rotation(accel_x, accel_y, accel_z)
                
            # Map MPU angle (-90 to 90 degrees) to servo angle (0 to 180 degrees)
            # Apply smoothing to reduce jitter (optional)
            servo_angle = min(max(angle + 90, 0), 180)
            
            # Implement a deadzone around the center to reduce unnecessary servo movements
            if 85 <= servo_angle <= 95:
                servo_angle = 90
                
            return servo_angle
        except Exception as e:
            print(f"Error getting servo angle from MPU on channel {self.mux_channel}: {e}")
            return 90  # Return neutral position on error

class PCA9685:
    def __init__(self, bus, address=PCA_ADDR):
        self.bus = bus
        self.address = address
        
        if not self.initialize():
            print(f"Failed to initialize PCA9685 at address 0x{address:02X}")
            sys.exit(1)
        
    def initialize(self):
        """Initialize the PCA9685 servo controller"""
        try:
            # Check if PCA9685 is connected
            try:
                self.bus.read_byte_data(self.address, PCA_MODE1)
            except:
                print(f"No PCA9685 found at address 0x{self.address:02X}")
                return False
                
            # Reset the controller
            self.bus.write_byte_data(self.address, PCA_MODE1, 0x00)
            time.sleep(0.05)
            
            # Set frequency to 50Hz for servos (period of 20ms)
            self.set_pwm_freq(50)
            
            print(f"PCA9685 initialized at address 0x{self.address:02X}")
            return True
        except Exception as e:
            print(f"Error initializing PCA9685: {e}")
            return False
            
    def set_pwm_freq(self, freq_hz):
        """Set the PWM frequency for all channels"""
        try:
            # Calculate prescale value based on the datasheet formula
            prescale_val = int(25000000.0 / (4096.0 * freq_hz) - 0.5)
            
            # Read current MODE1 register
            old_mode = self.bus.read_byte_data(self.address, PCA_MODE1)
            
            # Put PCA9685 to sleep
            new_mode = (old_mode & 0x7F) | 0x10  # Set sleep bit
            self.bus.write_byte_data(self.address, PCA_MODE1, new_mode)
            
            # Set prescale value
            self.bus.write_byte_data(self.address, PCA_PRESCALE, prescale_val)
            
            # Restore old mode
            self.bus.write_byte_data(self.address, PCA_MODE1, old_mode)
            time.sleep(0.005)  # Wait for oscillator
            
            # Set auto-increment bit
            self.bus.write_byte_data(self.address, PCA_MODE1, old_mode | 0xA0)
            
            print(f"PCA9685 frequency set to {freq_hz}Hz")
        except Exception as e:
            print(f"Error setting PWM frequency: {e}")
            
    def set_pwm(self, channel, on, off):
        """Set the PWM on and off time for a specific channel"""
        try:
            # Ensure channel is within valid range
            if channel < 0 or channel > 15:
                print(f"Error: Invalid channel {channel}")
                return
                
            # Calculate register addresses
            reg_on_l = PCA_LED0_ON_L + (channel * 4)
            
            # Write the on and off values
            self.bus.write_byte_data(self.address, reg_on_l, on & 0xFF)
            self.bus.write_byte_data(self.address, reg_on_l + 1, on >> 8)
            self.bus.write_byte_data(self.address, reg_on_l + 2, off & 0xFF)
            self.bus.write_byte_data(self.address, reg_on_l + 3, off >> 8)
        except Exception as e:
            print(f"Error setting PWM for channel {channel}: {e}")
            
    def set_servo_angle(self, channel, angle):
        """Set servo angle (0-180 degrees)"""
        try:
            # Ensure angle is within valid range
            angle = min(max(angle, 0), 180)
            
            # Map angle to pulse width
            pulse_width = int(SERVO_MIN_PULSE + (angle / 180.0) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE))
            
            # Set PWM values (on_time=0, off_time=pulse_width)
            self.set_pwm(channel, 0, pulse_width)
        except Exception as e:
            print(f"Error setting servo angle for channel {channel}: {e}")

def main():
    print("Starting MPU6050 Servo Control System...")
    
    # First, scan to see what devices are available
    devices = scan_i2c_devices()
    
    if not devices:
        print("No I2C devices found. Exiting.")
        return
        
    if MUX_ADDR not in devices:
        print(f"Multiplexer not found at address 0x{MUX_ADDR:02X}. Exiting.")
        return
    
    # Initialize I2C bus
    bus = smbus.SMBus(1)  # Use 1 for RPi 5

    # First select multiplexer channel 1 to access PCA9685
    print("Selecting multiplexer channel 1 for PCA9685...")
    try:
        bus.write_byte(MUX_ADDR, 1 << 1)  # Select channel 1
        time.sleep(0.1)  # Give it time to switch
    except Exception as e:
        print(f"Error accessing multiplexer: {e}")
        return
    
    # Now check if PCA9685 is accessible
    try:
        bus.read_byte_data(PCA_ADDR, PCA_MODE1)
        print(f"Found PCA9685 at address 0x{PCA_ADDR:02X} on multiplexer channel 1")
    except Exception as e:
        print(f"Error: Cannot access PCA9685 on multiplexer channel 1: {e}")
        return
    
    # Initialize PCA9685 with a custom class for multiplexed access
    class MultiplexedPCA9685(PCA9685):
        def __init__(self, bus, mux_addr, pca_addr, mux_channel):
            self.mux_addr = mux_addr
            self.mux_channel = mux_channel
            self.address = pca_addr
            self.bus = bus
            if not self.initialize():
                print(f"Failed to initialize PCA9685 at address 0x{pca_addr:02X}")
                sys.exit(1)
                
        def select_channel(self):
            """Select the appropriate channel on the multiplexer"""
            try:
                self.bus.write_byte(self.mux_addr, 1 << self.mux_channel)
                time.sleep(0.05)  # Small delay to let the multiplexer settle
                return True
            except Exception as e:
                print(f"Error selecting multiplexer channel {self.mux_channel}: {e}")
                return False
                
        # Override initialize method to select channel first
        def initialize(self):
            if not self.select_channel():
                return False
            return super().initialize()
            
        # Override all methods that access the PCA9685 to select channel first
        def set_pwm_freq(self, freq_hz):
            if not self.select_channel():
                return
            super().set_pwm_freq(freq_hz)
            
        def set_pwm(self, channel, on, off):
            if not self.select_channel():
                return
            super().set_pwm(channel, on, off)
    
    # Initialize PCA9685 through multiplexer channel 1
    pca = MultiplexedPCA9685(bus, MUX_ADDR, PCA_ADDR, 1)
    
    # Initialize MPU6050 sensors on multiplexer channels 0 and 7
    print("Initializing MPU6050 sensors...")
    mpu_ch0 = MPU6050(bus, MUX_ADDR, MPU_ADDR, 0)
    mpu_ch7 = MPU6050(bus, MUX_ADDR, MPU_ADDR, 7)
    
    print("System initialized. Starting control loop...")
    print("Press Ctrl+C to exit")
    
    try:
        # Set servos to center position initially
        print("Setting servos to center position...")
        pca.set_servo_angle(0, 90)
        pca.set_servo_angle(1, 90)
        time.sleep(1)
        
        # Create simple moving average for each channel to smooth the values
        ch0_angles = [90] * 5  # List to store last 5 readings for channel 0
        ch7_angles = [90] * 5  # List to store last 5 readings for channel 7
        
        print("Starting control loop. Press Ctrl+C to exit.")
        print("-" * 50)
        
        while True:
            # Get angles from MPU6050 sensors
            angle_ch0 = mpu_ch0.get_servo_angle('x')  # Use x-axis rotation for servo 0
            angle_ch7 = mpu_ch7.get_servo_angle('y')  # Use y-axis rotation for servo 1
            
            # Update moving averages
            ch0_angles.pop(0)
            ch0_angles.append(angle_ch0)
            ch7_angles.pop(0)
            ch7_angles.append(angle_ch7)
            
            # Calculate average angles (smoothed)
            smooth_angle_ch0 = sum(ch0_angles) / len(ch0_angles)
            smooth_angle_ch7 = sum(ch7_angles) / len(ch7_angles)
            
            # Apply the angles to the servos
            pca.set_servo_angle(0, smooth_angle_ch0)
            pca.set_servo_angle(1, smooth_angle_ch7)
            
            print(f"MPU Ch0: {angle_ch0:.1f}° (smoothed: {smooth_angle_ch0:.1f}°) | MPU Ch7: {angle_ch7:.1f}° (smoothed: {smooth_angle_ch7:.1f}°)", end="\r")
            
            time.sleep(0.05)  # Update rate - increased for smoother motion
            
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
        # Set servos back to center position before exiting
        pca.set_servo_angle(0, 90)
        pca.set_servo_angle(1, 90)

if __name__ == "__main__":
    main()