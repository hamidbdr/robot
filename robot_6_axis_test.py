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
current_angles = [0, 0, 0, 0, 0, 0]

# Motor pin arrays
STEP_PINS = [STEP_PIN_1, STEP_PIN_2, STEP_PIN_3, STEP_PIN_4, STEP_PIN_5, STEP_PIN_6]
DIR_PINS = [DIR_PIN_1, DIR_PIN_2, DIR_PIN_3, DIR_PIN_4, DIR_PIN_5, DIR_PIN_6]

# CORRECTED Microstepping configuration
MICRO_STEPPING = [32, 32, 1, 1, 1, 1]  # Motors 1&2: 32x, Motors 3-6: 1x (not 16x!)

# Reduction factors
REDUCTION_FACTORS = [9.5, 9.5, 6.5, 6.5, 6.5, 9.5]

def set_directions(directions):
    """Set directions for all motors with detailed feedback"""
    for i, direction in enumerate(directions):
        DIR_PINS[i].value = direction
        print(f"Motor {i+1}: DIR pin {DIR_PINS[i].pin.number} set to {'HIGH' if direction else 'LOW'} ({'CW' if direction else 'CCW'})")
    print()

def test_single_motor_direction(motor_index, steps=200, delay=0.01):
    """
    Test a single motor in both directions
    
    Args:
        motor_index: 0-5 for motors 1-6
        steps: Number of steps to move
        delay: Delay between steps in seconds
    """
    motor_num = motor_index + 1
    print(f"\n=== Testing Motor {motor_num} Direction ===")
    print(f"Using pins: DIR={DIR_PINS[motor_index].pin.number}, STEP={STEP_PINS[motor_index].pin.number}")
    
    # Test CW direction (DIR = HIGH)
    print(f"Testing CW direction (DIR=HIGH)...")
    DIR_PINS[motor_index].on()  # Set direction HIGH
    time.sleep(0.1)  # Small delay for direction setup
    
    for step in range(steps):
        if stop_flag.is_set():
            print("Stop flag activated!")
            return
        STEP_PINS[motor_index].on()
        time.sleep(delay)
        STEP_PINS[motor_index].off()
        time.sleep(delay)
    
    print("CW movement complete. Pausing...")
    time.sleep(2)
    
    # Test CCW direction (DIR = LOW)
    print(f"Testing CCW direction (DIR=LOW)...")
    DIR_PINS[motor_index].off()  # Set direction LOW
    time.sleep(0.1)  # Small delay for direction setup
    
    for step in range(steps):
        if stop_flag.is_set():
            print("Stop flag activated!")
            return
        STEP_PINS[motor_index].on()
        time.sleep(delay)
        STEP_PINS[motor_index].off()
        time.sleep(delay)
    
    print("CCW movement complete.")

def test_all_motors_3_to_6():
    """Test motors 3-6 individually for direction verification"""
    print("\n" + "="*50)
    print("MOTOR DIRECTION TEST - Motors 3 to 6")
    print("="*50)
    print("Watch each motor carefully to verify direction:")
    print("- First movement should be CW (clockwise)")
    print("- Second movement should be CCW (counter-clockwise)")
    print("- Motor should return to approximately starting position")
    print()
    
    for motor_index in range(2, 6):  # Motors 3-6 (indices 2-5)
        test_single_motor_direction(motor_index, steps=400, delay=0.005)  # Slower for observation
        
        user_input = input(f"\nDid Motor {motor_index + 1} move correctly? (y/n/s to skip remaining): ").lower()
        if user_input == 's':
            break
        elif user_input == 'n':
            print(f"Motor {motor_index + 1} direction issue noted.")
        print("-" * 30)

def quick_direction_test():
    """Quick test to verify all motor directions simultaneously"""
    print("\n=== Quick Direction Test - All Motors 3-6 ===")
    print("All motors 3-6 will move CW, then CCW")
    
    # Set all motors 3-6 to CW
    directions = [False, False, True, True, True, True]  # Motors 1&2 disabled, 3-6 CW
    set_directions(directions)
    
    print("Moving all motors 3-6 CW...")
    threads = []
    for i in range(2, 6):  # Motors 3-6
        thread = threading.Thread(target=step_motor_simple, args=(i, 300, 0.003))
        threads.append(thread)
        thread.start()
    
    for thread in threads:
        thread.join()
    
    time.sleep(1)
    
    # Set all motors 3-6 to CCW
    directions = [False, False, False, False, False, False]  # Motors 1&2 disabled, 3-6 CCW
    set_directions(directions)
    
    print("Moving all motors 3-6 CCW...")
    threads = []
    for i in range(2, 6):  # Motors 3-6
        thread = threading.Thread(target=step_motor_simple, args=(i, 300, 0.003))
        threads.append(thread)
        thread.start()
    
    for thread in threads:
        thread.join()

def step_motor_simple(motor_index, steps, delay):
    """Simple stepping function for testing"""
    for step in range(steps):
        if stop_flag.is_set():
            return
        STEP_PINS[motor_index].on()
        time.sleep(delay)
        STEP_PINS[motor_index].off()
        time.sleep(delay)

def angle_to_steps(angle, motor_index):
    """Convert angle to steps - CORRECTED VERSION"""
    steps_per_revolution = MICRO_STEPPING[motor_index] * 200
    reduction_factor = REDUCTION_FACTORS[motor_index]
    steps = int((angle / 360) * steps_per_revolution * reduction_factor)
    print(f"Motor {motor_index+1}: {angle}° = {steps} steps (μstep:{MICRO_STEPPING[motor_index]}, reduction:{reduction_factor})")
    return steps

def test_angle_movements():
    """Test specific angle movements for motors 3-6"""
    print("\n=== Testing Angle Movements ===")
    
    test_angles = [10, -10, 20, -20, 0]  # Return to 0
    
    for motor_index in range(2, 6):  # Motors 3-6
        motor_num = motor_index + 1
        print(f"\nTesting Motor {motor_num} angle movements:")
        
        for target_angle in test_angles:
            current_angle = current_angles[motor_index]
            angle_diff = target_angle - current_angle
            steps = angle_to_steps(abs(angle_diff), motor_index)
            direction = angle_diff >= 0
            
            print(f"Moving from {current_angle}° to {target_angle}° (diff: {angle_diff}°)")
            print(f"Direction: {'CW' if direction else 'CCW'}, Steps: {steps}")
            
            # Set direction
            DIR_PINS[motor_index].value = direction
            time.sleep(0.1)
            
            # Move
            for step in range(steps):
                if stop_flag.is_set():
                    return
                STEP_PINS[motor_index].on()
                time.sleep(0.002)
                STEP_PINS[motor_index].off()
                time.sleep(0.002)
            
            current_angles[motor_index] = target_angle
            time.sleep(0.5)
        
        input(f"Press Enter to continue to next motor...")



def print_configuration():
    """Print detailed motor configuration"""
    print("MOTOR CONFIGURATION:")
    print("=" * 50)
    for i in range(6):
        motor_num = i + 1
        steps_per_rev = MICRO_STEPPING[i] * 200
        reduction = REDUCTION_FACTORS[i]
        total_steps = steps_per_rev * reduction
        print(f"Motor {motor_num}:")
        print(f"  DIR Pin: {DIR_PINS[i].pin.number}, STEP Pin: {STEP_PINS[i].pin.number}")
        print(f"  Microstepping: {MICRO_STEPPING[i]}x")
        print(f"  Reduction: {reduction}")
        print(f"  Steps per 360°: {total_steps}")
        print()

if __name__ == "__main__":
    try:
        print_configuration()
        
        print("MOTOR DIRECTION TESTING PROGRAM")
        print("=" * 40)
        print("This program will help you verify motor directions.")
        print("Choose a test mode:")
        print("1. Test each motor individually (recommended)")
        print("2. Quick test all motors 3-6 together")
        print("3. Test angle movements")
        print("4. Exit")
        
        while True:
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                test_all_motors_3_to_6()
            elif choice == "2":
                quick_direction_test()
            elif choice == "3":
                test_angle_movements()
            elif choice == "4":
                break
            else:
                print("Invalid choice. Please enter 1-4.")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Test completed!")