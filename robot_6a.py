from gpiozero import OutputDevice, Button
import time
import threading
import json
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class StepperMotorController:
    def __init__(self):
        # Define pins for stepper motors
        self.motor_pins = {
            1: {'dir': OutputDevice(24), 'step': OutputDevice(23)},
            2: {'dir': OutputDevice(25), 'step': OutputDevice(12)},
            3: {'dir': OutputDevice(6), 'step': OutputDevice(4)},
            4: {'dir': OutputDevice(13), 'step': OutputDevice(17)},
            5: {'dir': OutputDevice(19), 'step': OutputDevice(27)},
            6: {'dir': OutputDevice(26), 'step': OutputDevice(22)}
        }
        
        # Motor specifications
        self.motor_specs = {
            1: {'microstepping': 32, 'reduction': 9.5, 'max_rpm': 8},
            2: {'microstepping': 32, 'reduction': 9.5, 'max_rpm': 8},
            3: {'microstepping': 4, 'reduction': 9.5, 'max_rpm': 50},
            4: {'microstepping': 8, 'reduction': 6.5, 'max_rpm': 50},
            5: {'microstepping': 16, 'reduction': 6.5, 'max_rpm': 50},
            6: {'microstepping': 8, 'reduction': 6.5, 'max_rpm': 50}
        }
        
        # State tracking
        self.current_angles = [0.0] * 6
        self.is_moving = False
        self.stop_flag = threading.Event()
        self.movement_lock = threading.Lock()
        
        # Safety limits (degrees)
        self.angle_limits = {
            1: (-25, 25),
            2: (-25, 25),
            3: (-100, 100),
            4: (-160, 160),
            5: (-160, 160),
            6: (-160, 160)
        }
        
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
        """Move all motors to target angles with coordinated motion
        
        Args:
            target_angles: List of 6 target angles
            rpm_profiles: Either a single profile dict (applied to all motors) or 
                         a list of 6 profile dicts (one per motor)
        """
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
            # Single profile for all motors
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
    
    def home_all_motors(self, rpm_profiles=None):
        """Move all motors to home position (0 degrees)"""
        logger.info("Homing all motors...")
        home_position = [0.0] * 6
        return self.move_to_angles(home_position, rpm_profiles)
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        logger.warning("EMERGENCY STOP activated")
        self.stop_flag.set()
    
    def get_current_position(self):
        """Get current motor positions"""
        return self.current_angles.copy()
    
    def save_position(self, name, filename="positions.json"):
        """Save current position with a name"""
        try:
            try:
                with open(filename, 'r') as f:
                    positions = json.load(f)
            except FileNotFoundError:
                positions = {}
            
            positions[name] = self.current_angles.copy()
            
            with open(filename, 'w') as f:
                json.dump(positions, f, indent=2)
            
            logger.info(f"Position '{name}' saved")
            return True
        except Exception as e:
            logger.error(f"Failed to save position: {e}")
            return False
    
    def load_position(self, name, filename="positions.json"):
        """Load a saved position"""
        try:
            with open(filename, 'r') as f:
                positions = json.load(f)
            
            if name in positions:
                return positions[name]
            else:
                logger.error(f"Position '{name}' not found")
                return None
        except Exception as e:
            logger.error(f"Failed to load position: {e}")
            return None
    
    def cleanup(self):
        """Clean up GPIO resources"""
        logger.info("Cleaning up GPIO resources...")
        for motor_id in self.motor_pins:
            self.motor_pins[motor_id]['dir'].close()
            self.motor_pins[motor_id]['step'].close()


# Example usage
if __name__ == "__main__":
    controller = StepperMotorController()
    
    try:
        # Display motor configuration
        print("\n=== Motor Configuration ===")
        for motor_id in range(1, 7):
            specs = controller.motor_specs[motor_id]
            total_steps = controller.get_total_steps_per_360(motor_id)
            limits = controller.angle_limits[motor_id]
            print(f"Motor {motor_id}: {specs['microstepping']}x microstepping, "
                  f"{specs['reduction']} reduction, {total_steps} steps/360°, "
                  f"limits: {limits[0]}° to {limits[1]}°, max RPM: {specs['max_rpm']}")
        
        # Example 1: Single RPM profile for all motors
        single_rpm_profile = {
            'start': 0.5,
            'max': 25,
            'min': 0.5,
            'accel_fraction': 0.1,
            'decel_fraction': 0.1
        }
        
        # Example 2: Individual RPM profiles for each motor
        individual_rpm_profiles = [
            # Motor 1 - slow and steady
            {'start': 0.5, 'max': 5, 'min': 0.5, 'accel_fraction': 0.1, 'decel_fraction': 0.1},
            # Motor 2 - slow and steady
            {'start': 0.5, 'max': 5, 'min': 0.5, 'accel_fraction': 0.1, 'decel_fraction': 0.1},
            # Motor 3 - fast
            {'start': 1.0, 'max': 40, 'min': 0.5, 'accel_fraction': 0.3, 'decel_fraction': 0.3},
            # Motor 4 - medium speed
            {'start': 0.8, 'max': 80, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2},
            # Motor 5 - medium speed
            {'start': 0.8, 'max': 80, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2},
            # Motor 6 - fast
            {'start': 1.0, 'max': 80, 'min': 0.5, 'accel_fraction': 0.2, 'decel_fraction': 0.2}
        ]
        
        # Test sequence - focusing on motors 2-6 with safe angles
        test_sequence = [
            [0, 0, 0, 0, 0, 0],           # Home position
            [0, 0, 45, 45, 45, 45],          # Test motor 5
            [0, 0, 0, 0, 0, 0],           # Return home
            [-5, 5, -15, 20, 45, 90],      # Combined movement
            [0, 0, 0, 0, 0, 0],           # Return home
            [0, 0, 45, 0, 0, 0],          # Test motor 3
            [0, 0, 0, 0, 0, 0],           # Return home
            [0, 0, -60, 90, 0, 0],          # Test motor 4
            [0, 0, 0, 0, 0, 0],           # Return home
            [0, 0, -60, 90, 90, 90],          # Test motor 6
            [0, 0, 0, 0, 0, 0]            # Return home
        ]
        
        print("\n=== Testing with Individual Motor RPM Profiles ===")
        print("Motor RPM Settings:")
        for i, profile in enumerate(individual_rpm_profiles):
            motor_id = i + 1
            max_allowed = controller.motor_specs[motor_id]['max_rpm']
            actual_max = min(profile['max'], max_allowed)
            print(f"  Motor {motor_id}: {profile['max']} RPM requested, {actual_max} RPM actual (limited by hardware)")
        
        controller.move_sequence(test_sequence, individual_rpm_profiles, pause_between=0.5)
        
        print(f"\nFinal position: {controller.get_current_position()}")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        controller.emergency_stop()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()
        print("Program ended")