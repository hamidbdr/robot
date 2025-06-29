from gpiozero import OutputDevice, Button
import time
import threading


# Define pins for stepper motor 1
DIR_PIN_1 = OutputDevice(24)
STEP_PIN_1 = OutputDevice(23)

# Define pins for stepper motor 2
DIR_PIN_2 = OutputDevice(25)
STEP_PIN_2 = OutputDevice(12)

# Define pins for stepper motor 3
DIR_PIN_3 = OutputDevice(6)
STEP_PIN_3 = OutputDevice(4)

# Define pins for stepper motor 4
DIR_PIN_4 = OutputDevice(13)
STEP_PIN_4 = OutputDevice(17)

# Define pins for stepper motor 5
DIR_PIN_5 = OutputDevice(19)
STEP_PIN_5 = OutputDevice(27)

# Define pins for stepper motor 6
DIR_PIN_6 = OutputDevice(26)
STEP_PIN_6 = OutputDevice(22)


# Stop flag
stop_flag = threading.Event()

# Current position tracking
current_angles = [0, 0, 0, 0, 0, 0]  # For 6 motors

# Motor pin arrays for easier management
STEP_PINS = [STEP_PIN_1, STEP_PIN_2, STEP_PIN_3, STEP_PIN_4, STEP_PIN_5, STEP_PIN_6]
DIR_PINS = [DIR_PIN_1, DIR_PIN_2, DIR_PIN_3, DIR_PIN_4, DIR_PIN_5, DIR_PIN_6]
#ENABLE_PINS = [ENABLE_PIN_1, ENABLE_PIN_2, ENABLE_PIN_3, ENABLE_PIN_4, ENABLE_PIN_5, ENABLE_PIN_6]

# Microstepping configuration for each motor
MICRO_STEPPING = [32, 32, 1, 8, 1, 8]  # Motors 1&2: 32x, Motors 3-6: 1x

# Reduction factor for each motor
REDUCTION_FACTORS = [9.5, 9.5, 6.5, 6.5, 6.5, 9.5]  # Motors 1&2: 9.5, Motors 3-5: 6.5, Motor 6: 9.5


def set_directions(directions):
    """Set directions for all motors. directions is a list of 6 boolean values"""
    for i, direction in enumerate(directions):
        DIR_PINS[i].value = direction
    
    dir_str = ", ".join([f"M{i+1}:{'CW' if d else 'CCW'}" for i, d in enumerate(directions)])
    print(f"Directions - {dir_str}")

def rpm_to_delay(rpm, steps_per_revolution):
    if rpm > 0:
        return 60 / (rpm * steps_per_revolution)
    else:
        return float('inf')  # Very high delay when RPM is zero

def move_steps_motor(steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, 
                     decel_fraction, rpm_min, stop_event, step_pin, motor_id):
    if steps == 0:
        return 0
    
    accel_steps = int(abs(steps) * accel_fraction)
    decel_steps = int(abs(steps) * decel_fraction)
    hold_steps = abs(steps) - accel_steps - decel_steps

    hold_start_time = None
    hold_end_time = None

    # Acceleration phase
    for step in range(accel_steps):
        rpm = rpm_start + (rpm_max - rpm_start) * (step / accel_steps)
        rpm = max(rpm, rpm_min)  
        delay = rpm_to_delay(rpm, steps_per_revolution)


        step_pin.on()
        time.sleep(delay)
        step_pin.off()
        time.sleep(delay)

    # Hold phase
    hold_start_time = time.time()
    for step in range(hold_steps):
        delay = rpm_to_delay(rpm_max, steps_per_revolution)  


        step_pin.on()
        time.sleep(delay)
        step_pin.off()
        time.sleep(delay)
    hold_end_time = time.time()

    # Deceleration phase
    for step in range(decel_steps):
        rpm = rpm_max - (rpm_max - rpm_min) * (step / decel_steps)  
        rpm = max(rpm, rpm_min)
        delay = rpm_to_delay(rpm, steps_per_revolution)

        step_pin.on()
        time.sleep(delay)
        step_pin.off()
        time.sleep(delay)

    hold_duration = hold_end_time - hold_start_time
    revolutions = hold_steps / steps_per_revolution
    real_rpm = (revolutions / hold_duration) * 60 if hold_duration > 0 else 0
    print(f"Motor {motor_id} Real RPM in Hold Phase: {real_rpm:.2f}")
    return real_rpm

def angle_to_steps(angle, motor_index):
    """Convert angle in degrees to number of steps for a specific motor"""
    steps_per_revolution = MICRO_STEPPING[motor_index] * 200
    reduction_factor = REDUCTION_FACTORS[motor_index]
    return int((angle / 360) * steps_per_revolution * reduction_factor)

def move_to_angles(target_angles, rpm_start, rpm_max, 
                   accel_fraction, decel_fraction, rpm_min):
    """Move all motors to specified angles"""
    global current_angles
    
    if len(target_angles) != 6:
        raise ValueError("target_angles must contain exactly 6 angles")
    
    # Calculate angle differences and steps for each motor
    angle_diffs = [target_angles[i] - current_angles[i] for i in range(6)]
    steps_list = [angle_to_steps(abs(diff), i) for i, diff in enumerate(angle_diffs)]
    directions = [diff >= 0 for diff in angle_diffs]  # True for positive (CW), False for negative (CCW)
    
    print(f"Moving to angles: {[f'M{i+1}:{target_angles[i]}°' for i in range(6)]}")
    print(f"Angle differences: {[f'M{i+1}:{angle_diffs[i]}°' for i in range(6)]}")
    print(f"Steps to move: {[f'M{i+1}:{steps_list[i]}' for i in range(6)]}")
    print(f"Microstepping: {[f'M{i+1}:{MICRO_STEPPING[i]}x' for i in range(6)]}")
    print(f"Reduction factors: {[f'M{i+1}:{REDUCTION_FACTORS[i]}' for i in range(6)]}")
    
    set_directions(directions)
    
    # Create threads for all motors
    threads = []
    for i in range(6):
        # Calculate steps_per_revolution for this specific motor
        steps_per_revolution = MICRO_STEPPING[i] * 200
        thread = threading.Thread(target=move_steps_motor,
                                args=(steps_list[i], steps_per_revolution, rpm_start, rpm_max, 
                                     accel_fraction, decel_fraction, rpm_min, stop_flag, STEP_PINS[i], i+1))
        threads.append(thread)
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()
    
    # Update current positions
    current_angles[:] = target_angles[:]
    print(f"Motors reached target angles: {[f'M{i+1}:{current_angles[i]}°' for i in range(6)]}")

def move_through_angle_list(angle_list, rpm_start, rpm_max, 
                           accel_fraction, decel_fraction, rpm_min, pause_between_moves=0.1):
    """
    Move through a list of angle sets
    
    Args:
        angle_list: List of lists/tuples [[motor1_angle, motor2_angle, ..., motor6_angle], ...]
        pause_between_moves: Pause time between moves in seconds
    """
    #enable_motors()
    
    for i, angles in enumerate(angle_list):
        print(f"\n--- Move {i+1}/{len(angle_list)} ---")
        move_to_angles(angles, rpm_start, rpm_max, 
                      accel_fraction, decel_fraction, rpm_min)
        
        if i < len(angle_list) - 1:  # Don't pause after the last move
            time.sleep(pause_between_moves)


# Main execution
if __name__ == "__main__":
    try:
        # Speed configuration
        rpm_start = 0.5
        rpm_max = 10
        accel_fraction = 0.005
        decel_fraction = 0.005
        rpm_min = 0.5

        # Display configuration
        print("Motor Configuration:")
        for i in range(6):
            steps_per_rev = MICRO_STEPPING[i] * 200
            reduction = REDUCTION_FACTORS[i]
            total_steps = steps_per_rev * reduction
            print(f"Motor {i+1}: {MICRO_STEPPING[i]}x microstepping, {reduction} reduction = {total_steps} steps/360°")
        print()

        # Define your angle list here - Testing motors 3-6 only (motors 1&2 stay at 0)
        # Each list contains [motor1_angle, motor2_angle, motor3_angle, motor4_angle, motor5_angle, motor6_angle]
        angle_list = [
            [0, 0, 0, 0, 0, 0] ,          # Return to start
            #[-10, 0, 0, 0, 0, 0],          # Starting position - all motors at 0
            [0, 10, 0, 0, 0, 0],          # Starting position - all motors at 0
            [0, -10, 0, 0, 0, 0],           # Return to start
            #[0, 0, -45, 0, 0, 0],          # Starting position - all motors at 0
            [0, 0, 0, 0, 0, 0],           # Return to start
            #[0, 0, 0, 90, 0, 0],          # Starting position - all motors at 0
            [0, 0, 0, 0, 0, 0],           # Return to start
            #[0, 0, 0, 0, 45, 0],          # Starting position - all motors at 0
            #[0, 0, 0, 0, -45, 0],           # Return to start
            #[0, 0, 0, 0, 0, -45],          # Starting position - all motors at 0

            #[10, -80, 0, 0, 0, 0],          # Test motor 3 only
            #[12, -140, 0, 0, 0, 0],          # Test motor 4 only
            #[15, -145, 15, -12, 8, 5],        # Different angles for motors 3-6
            #[10, -140, 20, -15, 12, 10],      # Larger movements for motors 3-6
            #[10, -135, 35, -10, 15, 20],       # Progressive increase
            [0, 0, 0, 0, 0, 0]           # Return to start
        ]
        
        # Alternative test patterns for motors 3-6:
        # Test each motor individually with larger range:
        # angle_list = []
        # for motor in range(2, 6):  # Motors 3-6 (indices 2-5)
        #     for angle in [0, 10, 20, 30, 20, 10, 0]:
        #         angles = [0] * 6
        #         angles[motor] = angle
        #         angle_list.append(angles)
        
        # Sweep test for motors 3-6:
        # angle_list = [[0, 0, angle, angle//2, angle//3, angle//4] for angle in range(0, 31, 5)]
        
        print("Starting motor test sequence - Testing Motors 3-6 only...")
        print("Motors 1 & 2 will remain stationary at 0 degrees")
        print("IMPORTANT: Please verify the DIR and ENABLE pin assignments above match your hardware!")
        print("Current assignments are examples - update them for your specific setup.")
        print("\nMotor Specifications:")
        print("- Motors 1&2: 32x microstepping, 9.5 reduction")
        print("- Motors 3-5: 1x microstepping, 6.5 reduction") 
        print("- Motor 6: 1x microstepping, 9.5 reduction")
        print()
        
        move_through_angle_list(angle_list, rpm_start, rpm_max, 
                               accel_fraction, decel_fraction, rpm_min, 
                               pause_between_moves=0.2)
        
        print("\nAngle sequence completed!")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        #disable_motors()
        print("All motors disabled!")