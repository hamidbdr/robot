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
            6: {'dir': OutputDevice(5), 'step': OutputDevice(27)},
            4: {'dir': OutputDevice(26), 'step': OutputDevice(22)}
        }
        
        # Motor specifications
        self.motor_specs = {
            2: {'microstepping': 16, 'reduction': 36.5, 'max_rpm': 500},
            3: {'microstepping': 16, 'reduction': 36.5, 'max_rpm': 500},
            1: {'microstepping': 16, 'reduction': 9.5, 'max_rpm': 50},
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
    
    def get_adaptive_rpm_profile(self, angle_diff, motor_id):
        """Generate RPM profile based on movement size, respecting individual motor max_rpm."""
        abs_angle = abs(angle_diff)
        
        # Get the individual motor's maximum RPM from specs
        motor_max_rpm = self.motor_specs[motor_id]['max_rpm']

        # Base profiles for different movement sizes
        if abs_angle < 1.0:  # Very small movements
            return {
                'start': 0.2,
                # Use motor_max_rpm, but cap it for very small movements if desired
                'max': min(2, motor_max_rpm),
                'min': 0.2,
                'accel_fraction': 0.1,  # More gradual acceleration
                'decel_fraction': 0.1
            }
        elif abs_angle < 5.0:  # Small movements
            return {
                'start': 0.5,
                # Use motor_max_rpm, but cap it for small movements if desired
                'max': min(15, motor_max_rpm),
                'min': 0.3,
                'accel_fraction': 0.1,
                'decel_fraction': 0.1
            }
        elif abs_angle < 15.0:  # Medium movements
            return {
                'start': 1.0,
                # Use motor_max_rpm, but cap it for medium movements if desired
                'max': min(25, motor_max_rpm),
                'min': 0.5,
                'accel_fraction': 0.2,
                'decel_fraction': 0.2
            }
        else:  # Large movements
            return {
                'start': 1.0,
                # Use the actual motor_max_rpm for large movements
                'max': motor_max_rpm, # Directly use the motor's max RPM
                'min': 0.8,
                'accel_fraction': 0.15,
                'decel_fraction': 0.15
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
        """Improved motor movement with minimum step enforcement"""
        if steps == 0:
            return
            
        step_pin = self.motor_pins[motor_id]['step']
        steps_per_rev = self.get_steps_per_revolution(motor_id)
        
        # Use motor-specific max RPM, limited by hardware specs
        max_rpm = min(motor_rpm_profile['max'], self.motor_specs[motor_id]['max_rpm'])
        
        # Ensure minimum steps for acceleration phases
        min_accel_steps = 5  # Minimum steps for smooth acceleration
        min_decel_steps = 5
        
        # Calculate acceleration phases with minimums
        accel_steps = max(min_accel_steps, int(abs(steps) * motor_rpm_profile['accel_fraction']))
        decel_steps = max(min_decel_steps, int(abs(steps) * motor_rpm_profile['decel_fraction']))
        
        # Adjust if total accel+decel exceeds available steps
        if accel_steps + decel_steps > abs(steps):
            ratio = abs(steps) / (accel_steps + decel_steps)
            accel_steps = int(accel_steps * ratio)
            decel_steps = int(decel_steps * ratio)
        
        const_steps = abs(steps) - accel_steps - decel_steps
        
        logger.debug(f"Motor {motor_id}: {steps} steps, accel:{accel_steps}, const:{const_steps}, decel:{decel_steps}")
        
        step_count = 0
        
        # Acceleration phase with smoother ramping
        for i in range(accel_steps):
            if self.stop_flag.is_set():
                return
                
            # Use smoother acceleration curve (quadratic instead of linear)
            progress = (i / accel_steps) ** 0.5 if accel_steps > 0 else 1
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
        
        # Deceleration phase with smoother ramping
        for i in range(decel_steps):
            if self.stop_flag.is_set():
                return
                
            # Use smoother deceleration curve
            progress = (i / decel_steps) ** 0.5 if decel_steps > 0 else 1
            rpm = max_rpm - (max_rpm - motor_rpm_profile['min']) * progress
            rpm = max(rpm, motor_rpm_profile['min'])
            
            delay = self.calculate_delay(rpm, steps_per_rev)
            
            step_pin.on()
            time.sleep(delay)
            step_pin.off()
            time.sleep(delay)
            step_count += 1
        
        logger.debug(f"Motor {motor_id} completed {step_count} steps at max {max_rpm} RPM")
    
    def move_to_angles_adaptive(self, target_angles, override_profiles=None, min_movement_threshold=0.1):
        """Move with adaptive speed profiles based on movement size"""
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
                
                # Filter out tiny movements that cause jitter
                significant_movements = []
                for i, diff in enumerate(angle_diffs):
                    if abs(diff) < min_movement_threshold:
                        angle_diffs[i] = 0  # Skip this motor
                        significant_movements.append(False)
                    else:
                        significant_movements.append(True)
                
                steps_list = [self.angle_to_steps(abs(diff), i+1) for i, diff in enumerate(angle_diffs)]
                directions = [diff >= 0 for diff in angle_diffs]
                
                # Check if any significant movement is needed
                if not any(significant_movements):
                    logger.info("No significant movements needed (all angles within threshold)")
                    return True
                
                # Generate adaptive profiles or use override
                if override_profiles is None:
                    motor_profiles = [
                        self.get_adaptive_rpm_profile(angle_diffs[i], i+1) 
                        for i in range(6)
                    ]
                else:
                    # Validate that override_profiles is a list of 6 dictionaries
                    if not (isinstance(override_profiles, list) and len(override_profiles) == 6 and
                            all(isinstance(p, dict) for p in override_profiles)):
                        raise ValueError("override_profiles must be a list of 6 dictionaries")
                    motor_profiles = override_profiles
                
                logger.info(f"Moving to: {[f'M{i+1}:{target_angles[i]:.1f}°' for i in range(6)]}")
                logger.info(f"Angle diffs: {[f'M{i+1}:{angle_diffs[i]:.1f}°' for i in range(6)]}")
                max_rpms = [f"M{i+1}:{min(motor_profiles[i]['max'], self.motor_specs[i+1]['max_rpm'])}" for i in range(6)]
                logger.info(f"Max RPMs: {max_rpms}")
                
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


# Test script
if __name__ == "__main__":
    controller = StepperMotorController()
    
    # Define your custom RPM profiles for each motor
    # These profiles will be used when `override_profiles` is provided
    custom_profiles = [
        # Motor 1: Max 40 RPM
        {'start': 0.1, 'max': 40, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2},
        # Motor 2: Max 250 RPM
        {'start': 0.5, 'max': 300, 'min': 0.5, 'accel_fraction': 0.25, 'decel_fraction': 0.25},
        # Motor 3: Max 250 RPM
        {'start': 0.5, 'max': 300, 'min': 0.5, 'accel_fraction': 0.25, 'decel_fraction': 0.25},
        # Motor 4: Max 80 RPM
        {'start': 0.5, 'max': 40, 'min': 0.5, 'accel_fraction': 0.25, 'decel_fraction': 0.25},
        # Motor 5: Max 80 RPM
        {'start': 0.5, 'max': 40, 'min': 0.5, 'accel_fraction': 0.25, 'decel_fraction': 0.25},
        # Motor 6: Max 80 RPM
        {'start': 0.5, 'max': 40, 'min': 0.5, 'accel_fraction': 0.3, 'decel_fraction': 0.3}
    ]

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
        random.seed(78) # Comment out for truly random results each time
        
        num_tests = 5
        print(f"\n=== Running {num_tests} Random Tests with Custom RPM Profiles Automatically ===")
        
        for i in range(num_tests):
            print(f"\n--- Test Sequence {i+1}/{num_tests} ---")
            
            # Generate new random angles for this test
            random_angles = controller.generate_random_angles()
            print(f"Target random angles: {[f'M{j+1}:{random_angles[j]:>6.1f}°' for j in range(6)]}")
            print("Applying custom RPM profiles for this movement.")
            
            # Move to the random angles using the custom_profiles
            controller.move_to_angles_adaptive(random_angles, override_profiles=custom_profiles)
            
            # Add a pause after reaching the random position
            time.sleep(0.05) 
            
            # Return to home position using the same custom profiles
            print("\nReturning to home position (0.0° for all motors) with custom profiles...")
        controller.move_to_angles_adaptive([0.0] * 6, override_profiles=custom_profiles)
            
            # Add a pause after returning home, before the next sequence
        time.sleep(0.05) 
        
        print(f"\nAll {num_tests} random test sequences completed. Final position: {controller.current_angles}")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Initiating emergency stop.")
        controller.emergency_stop()
    except Exception as e:
        print(f"An error occurred: {e}")
        logger.error(f"Unexpected error: {e}")
    finally:
        controller.cleanup()
        print("Program ended and GPIO resources cleaned up.")