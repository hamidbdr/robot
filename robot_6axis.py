from gpiozero import OutputDevice, Button
import time
import threading
import json
import logging
import random

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class StepperMotorController:
    def __init__(self):
        # Define pins for stepper motors
        self.motor_pins = {
            3: {'dir': OutputDevice(24), 'step': OutputDevice(23)},
            2: {'dir': OutputDevice(25), 'step': OutputDevice(12)},
            1: {'dir': OutputDevice(6), 'step': OutputDevice(4)},
            5: {'dir': OutputDevice(13), 'step': OutputDevice(17)},
            6: {'dir': OutputDevice(19), 'step': OutputDevice(27)},
            4: {'dir': OutputDevice(26), 'step': OutputDevice(22)}
        }
        
        # Motor specifications
        self.motor_specs = {
            2: {'microstepping': 16, 'reduction': 36.5, 'max_rpm': 500},
            3: {'microstepping': 16, 'reduction': 36.5, 'max_rpm': 500},
            1: {'microstepping': 4, 'reduction': 9.5, 'max_rpm': 50},
            5: {'microstepping': 8, 'reduction': 6.5, 'max_rpm': 50},
            6: {'microstepping': 16, 'reduction': 6.5, 'max_rpm': 50},
            4: {'microstepping': 8, 'reduction': 6.5, 'max_rpm': 50}
        }
        
        # State tracking
        self.current_angles = [0.0] * 6
        self.is_moving = False
        self.stop_flag = threading.Event()
        self.movement_lock = threading.Lock()
        
        # Safety limits (degrees)
        self.angle_limits = {
            1: (-60, 60),    # Motor 1 index 0
            2: (0, 45),      # Motor 2 index 1
            3: (-70, 0),     # Motor 3 index 2
            4: (-80, 80),    # Motor 4 index 3
            5: (-15, 100),   # Motor 5 index 4
            6: (-90, 90)     # Motor 6 index 5
        }
        
    def generate_random_angles(self, num_positions=1):
        """Generate random angles within safe limits for all motors"""
        random_positions = []
        
        for _ in range(num_positions):
            angles = []
            for motor_id in range(1, 7):
                min_angle, max_angle = self.angle_limits[motor_id]
                # Generate random angle within limits
                random_angle = random.uniform(min_angle, max_angle)
                # Round to 1 decimal place for cleaner output
                angles.append(round(random_angle, 1))
            random_positions.append(angles)
        
        return random_positions if num_positions > 1 else random_positions[0]
    
    def generate_random_test_sequence(self, num_movements=10, include_home=True):
        """Generate a random test sequence with optional home returns"""
        sequence = []
        
        # Always start at home
        sequence.append([0.0] * 6)
        
        for i in range(num_movements):
            # Generate random position
            random_angles = self.generate_random_angles()
            sequence.append(random_angles)
            
            # Optionally return to home between movements
            if include_home and i < num_movements - 1:
                sequence.append([0.0] * 6)
        
        # Always end at home
        sequence.append([0.0] * 6)
        
        return sequence
    
    def print_angle_constraints(self):
        """Print the angle constraints for all motors"""
        print("\n=== Motor Angle Constraints ===")
        for motor_id in range(1, 7):
            min_angle, max_angle = self.angle_limits[motor_id]
            range_size = max_angle - min_angle
            print(f"Motor {motor_id}: {min_angle:>4.0f}° to {max_angle:>4.0f}° (range: {range_size:>3.0f}°)")
        print()
    
    def validate_angles(self, angles):
        """Validate that all angles are within safe limits"""
        if len(angles) != 6:
            raise ValueError("Must provide exactly 6 angles")
            
        for i, angle in enumerate(angles):
            motor_id = i + 1
            min_angle, max_angle = self.angle_limits[motor_id]
            if not (min_angle <= angle <= max_angle):
                raise ValueError(f"Motor {motor_id} angle {angle}° outside limits ({min_angle}°, {max_angle}°)")
    
    def get_steps_per_revolution(self, motor_id):
        """Calculate steps per revolution for a motor"""
        return self.motor_specs[motor_id]['microstepping'] * 200
    
    def get_total_steps_per_360(self, motor_id):
        """Calculate total steps for 360° rotation including reduction"""
        steps_per_rev = self.get_steps_per_revolution(motor_id)
        reduction = self.motor_specs[motor_id]['reduction']
        return int(steps_per_rev * reduction)
    
    def angle_to_steps(self, angle, motor_id):
        """Convert angle to steps for specific motor"""
        total_steps = self.get_total_steps_per_360(motor_id)
        return int((angle / 360.0) * total_steps)
    
    def set_directions(self, directions):
        """Set direction for all motors"""
        for i, direction in enumerate(directions):
            motor_id = i + 1
            self.motor_pins[motor_id]['dir'].value = direction
        
        dir_str = ", ".join([f"M{i+1}:{'CW' if d else 'CCW'}" for i, d in enumerate(directions)])
        logger.info(f"Directions set - {dir_str}")
    
    def calculate_delay(self, rpm, steps_per_revolution):
        """Calculate delay between steps for given RPM"""
        if rpm <= 0:
            return float('inf')
        return 60.0 / (2.0 * rpm * steps_per_revolution)  # Factor of 2 for on/off cycle
    
    def move_motor_steps(self, motor_id, steps, motor_rpm_profile):
        """Move a single motor with acceleration profile"""
        if steps == 0:
            return
            
        step_pin = self.motor_pins[motor_id]['step']
        steps_per_rev = self.get_steps_per_revolution(motor_id)
        
        # Use motor-specific max RPM, limited by hardware specs
        max_rpm = min(motor_rpm_profile['max'], self.motor_specs[motor_id]['max_rpm'])
        
        # Calculate acceleration phases
        accel_steps = int(abs(steps) * motor_rpm_profile['accel_fraction'])
        decel_steps = int(abs(steps) * motor_rpm_profile['decel_fraction'])
        const_steps = abs(steps) - accel_steps - decel_steps
        
        step_count = 0
        
        # Acceleration phase
        for i in range(accel_steps):
            if self.stop_flag.is_set():
                return
                
            progress = i / accel_steps if accel_steps > 0 else 1
            rpm = motor_rpm_profile['start'] + (max_rpm - motor_rpm_profile['start']) * progress
            rpm = max(rpm, motor_rpm_profile['min'])
            
            delay = self.calculate_delay(rpm, steps_per_rev)
            
            step_pin.on()
            time.sleep(delay)
            step_pin.off()
            time.sleep(delay)
            step_count += 1
        
        # Constant speed phase
        delay = self.calculate_delay(max_rpm, steps_per_rev)
        for i in range(const_steps):
            if self.stop_flag.is_set():
                return
                
            step_pin.on()
            time.sleep(delay)
            step_pin.off()
            time.sleep(delay)
            step_count += 1
        
        # Deceleration phase
        for i in range(decel_steps):
            if self.stop_flag.is_set():
                return
                
            progress = i / decel_steps if decel_steps > 0 else 1
            rpm = max_rpm - (max_rpm - motor_rpm_profile['min']) * progress
            rpm = max(rpm, motor_rpm_profile['min'])
            
            delay = self.calculate_delay(rpm, steps_per_rev)
            
            step_pin.on()
            time.sleep(delay)
            step_pin.off()
            time.sleep(delay)
            step_count += 1
        
        logger.debug(f"Motor {motor_id} completed {step_count} steps at max {max_rpm} RPM")
    
    def move_to_angles(self, target_angles, rpm_profiles=None):
        """Move all motors to target angles with coordinated motion"""
        # Default profile if none provided
        default_profile = {
            'start': 0.5,
            'max': 10,
            'min': 0.5,
            'accel_fraction': 0.1,
            'decel_fraction': 0.1
        }
        
        # Handle rpm_profiles parameter
        if rpm_profiles is None:
            motor_profiles = [default_profile.copy() for _ in range(6)]
        elif isinstance(rpm_profiles, dict):
            motor_profiles = [rpm_profiles.copy() for _ in range(6)]
        elif isinstance(rpm_profiles, list):
            if len(rpm_profiles) != 6:
                raise ValueError("rpm_profiles list must contain exactly 6 profiles")
            motor_profiles = rpm_profiles
        else:
            raise ValueError("rpm_profiles must be a dict or list of 6 dicts")
        
        with self.movement_lock:
            if self.is_moving:
                logger.warning("Movement already in progress")
                return False
            
            try:
                self.validate_angles(target_angles)
                self.is_moving = True
                self.stop_flag.clear()
                
                # Calculate movements
                angle_diffs = [target_angles[i] - self.current_angles[i] for i in range(6)]
                steps_list = [self.angle_to_steps(abs(diff), i+1) for i, diff in enumerate(angle_diffs)]
                directions = [diff >= 0 for diff in angle_diffs]
                
                logger.info(f"Moving to: {[f'M{i+1}:{target_angles[i]:.1f}°' for i in range(6)]}")
                logger.info(f"Steps: {[f'M{i+1}:{steps_list[i]}' for i in range(6)]}")
                
                self.set_directions(directions)
                
                # Create and start motor threads
                threads = []
                for i in range(6):
                    if steps_list[i] > 0:
                        thread = threading.Thread(
                            target=self.move_motor_steps,
                            args=(i+1, steps_list[i], motor_profiles[i]),
                            name=f"Motor_{i+1}"
                        )
                        threads.append(thread)
                        thread.start()
                
                # Wait for completion
                for thread in threads:
                    thread.join()
                
                # Update positions if not stopped
                if not self.stop_flag.is_set():
                    self.current_angles = target_angles.copy()
                    logger.info(f"Movement completed: {[f'M{i+1}:{self.current_angles[i]:.1f}°' for i in range(6)]}")
                else:
                    logger.warning("Movement was stopped before completion")
                
                return True
                
            except Exception as e:
                logger.error(f"Movement error: {e}")
                return False
            finally:
                self.is_moving = False
    
    def move_sequence(self, angle_sequence, rpm_profiles=None, pause_between=0.5):
        """Execute a sequence of movements"""
        logger.info(f"Starting sequence of {len(angle_sequence)} movements")
        
        for i, angles in enumerate(angle_sequence):
            logger.info(f"--- Movement {i+1}/{len(angle_sequence)} ---")
            
            if not self.move_to_angles(angles, rpm_profiles):
                logger.error(f"Movement {i+1} failed, stopping sequence")
                break
            
            if i < len(angle_sequence) - 1 and pause_between > 0:
                time.sleep(pause_between)
        
        logger.info("Sequence completed")
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        logger.warning("EMERGENCY STOP activated")
        self.stop_flag.set()
    
    def cleanup(self):
        """Clean up GPIO resources"""
        logger.info("Cleaning up GPIO resources...")
        for motor_id in self.motor_pins:
            self.motor_pins[motor_id]['dir'].close()
            self.motor_pins[motor_id]['step'].close()


# Test script with random angles
if __name__ == "__main__":
    controller = StepperMotorController()
    
    try:
        # Display motor configuration and constraints
        print("\n=== Motor Configuration ===")
        for motor_id in range(1, 7):
            specs = controller.motor_specs[motor_id]
            total_steps = controller.get_total_steps_per_360(motor_id)
            limits = controller.angle_limits[motor_id]
            print(f"Motor {motor_id}: {specs['microstepping']}x microstepping, "
                  f"{specs['reduction']} reduction, {total_steps} steps/360°, "
                  f"limits: {limits[0]}° to {limits[1]}°, max RPM: {specs['max_rpm']}")
        
        # Display angle constraints
        controller.print_angle_constraints()
        
        # Set random seed for reproducible results (optional)
        random.seed(42)  # Remove this line for truly random results each time
        
        # Generate some example random angles
        print("=== Example Random Angles ===")
        for i in range(5):
            random_angles = controller.generate_random_angles()
            print(f"Random set {i+1}: {[f'M{j+1}:{random_angles[j]:>6.1f}°' for j in range(6)]}")
        
        # RPM profiles for testing
        individual_rpm_profiles = {
            'start': 0.5,
            'max': 20,
            'min': 0.5,
            'accel_fraction': 0.15,
            'decel_fraction': 0.15
        }
        

        # Example 2: Individual RPM profiles for each motor
        individual_rpm_profiles = [
            # Motor 1 - slow and steady
            {'start': 0.5, 'max': 50, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2},
            # Motor 2 - slow and steady
            {'start': 0.5, 'max': 250, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2},
            # Motor 3 - fast
            {'start': 1.0, 'max': 250, 'min': 0.5, 'accel_fraction': 0.3, 'decel_fraction': 0.2},
            # Motor 4 - medium speed
            {'start': 0.8, 'max': 40, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2},
            # Motor 5 - medium speed
            {'start': 0.8, 'max': 40, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2},
            # Motor 6 - fast
            {'start': 1.0, 'max': 80, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2}
        ]

        # Generate and execute random test sequence
        print("\n=== Random Test Sequence ===")
        num_random_movements = 8
        random_sequence = controller.generate_random_test_sequence(
            num_movements=num_random_movements, 
            include_home=False  # Set to True if you want to return home between each movement
        )
        
        print(f"Generated sequence with {len(random_sequence)} positions:")
        for i, angles in enumerate(random_sequence):
            print(f"  Position {i+1}: {[f'M{j+1}:{angles[j]:>6.1f}°' for j in range(6)]}")
        
        # Ask user for confirmation
        response = input(f"\nExecute random sequence? (y/n): ")
        if response.lower() == 'y':
            controller.move_sequence(random_sequence, individual_rpm_profiles, pause_between=1.0)
        else:
            print("Random sequence cancelled by user")
            
        # Alternative: Test individual random positions
        print("\n=== Individual Random Position Tests ===")
        for i in range(3):
            random_angles = controller.generate_random_angles()
            print(f"\nTest {i+1}: {[f'M{j+1}:{random_angles[j]:>6.1f}°' for j in range(6)]}")
            
            response = input("Execute this position? (y/n/q): ")
            if response.lower() == 'q':
                break
            elif response.lower() == 'y':
                controller.move_to_angles(random_angles, individual_rpm_profiles)
                time.sleep(0.5)
                # Return to home
                controller.move_to_angles([0.0] * 6, individual_rpm_profiles)
                time.sleep(0.5)
        
        print(f"\nFinal position: {controller.current_angles}")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        controller.emergency_stop()
    except Exception as e:
        print(f"Error: {e}")
        logger.error(f"Unexpected error: {e}")
    finally:
        controller.cleanup()
        print("Program ended")