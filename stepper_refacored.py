from gpiozero import OutputDevice, Button
import time
import threading


class StepperMotor:
    def __init__(self, dir_pin, step_pin, enable_pin, motor_id=1, limit_switches=None):
        self.dir_pin = OutputDevice(dir_pin)
        self.step_pin = OutputDevice(step_pin)
        self.enable_pin = OutputDevice(enable_pin)
        self.limit_switches = limit_switches if limit_switches else []  # List of limit switches
        self.stop_flag = threading.Event()
        self.motor_id = motor_id

    def enable(self):
        self.enable_pin.off()  # Assuming LOW enables the motor
        print(f"Motor {self.motor_id} enabled.")

    def disable(self):
        self.enable_pin.on()  # Assuming HIGH disables the motor
        print(f"Motor {self.motor_id} disabled.")

    def set_direction(self, clockwise):
        self.dir_pin.value = clockwise  # True for CW, False for CCW
        print(f"Motor {self.motor_id} Direction: {'CW' if clockwise else 'CCW'}")

    def move_steps(self, steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min):
        # Check the status of the limit switches before starting
        self.check_limit_switches()

        accel_steps = int(steps * accel_fraction)
        decel_steps = int(steps * decel_fraction)
        hold_steps = steps - accel_steps - decel_steps

        def calculate_delay(rpm):
            return 60 / (rpm * steps_per_revolution) if rpm > 0 else float('inf')

        # Acceleration phase
        for step in range(accel_steps):
            if self.stop_flag.is_set():
                print(f"Motor {self.motor_id} stopped during acceleration at step {step}.")
                return
            rpm = rpm_start + (rpm_max - rpm_start) * (step / accel_steps)
            delay = calculate_delay(max(rpm, rpm_min))
            self.step_pin.on()
            time.sleep(delay)
            self.step_pin.off()
            time.sleep(delay)

        # Hold phase
        for step in range(hold_steps):
            if self.stop_flag.is_set():
                print(f"Motor {self.motor_id} stopped during hold at step {step}.")
                return
            delay = calculate_delay(rpm_max) / 2
            self.step_pin.on()
            time.sleep(delay)
            self.step_pin.off()
            time.sleep(delay)

        # Deceleration phase
        for step in range(decel_steps):
            if self.stop_flag.is_set():
                print(f"Motor {self.motor_id} stopped during deceleration at step {step}.")
                return
            rpm = rpm_max - (rpm_max - rpm_min) * (step / decel_steps)
            delay = calculate_delay(max(rpm, rpm_min))
            self.step_pin.on()
            time.sleep(delay)
            self.step_pin.off()
            time.sleep(delay)

    def attach_limit_switch(self, limit_switch):
        """Attach a single limit switch."""
        self.limit_switches.append(limit_switch)
        limit_switch.when_pressed = lambda: self._limit_switch_pressed(limit_switch)
        limit_switch.when_released = lambda: self._limit_switch_released(limit_switch)

    def _limit_switch_pressed(self, limit_switch):
        self.stop_flag.set()
        print(f"Motor {self.motor_id}: Limit switch {limit_switch.pin.number} pressed. Stopping motor.")

    def _limit_switch_released(self, limit_switch):
        print(f"Motor {self.motor_id}: Limit switch {limit_switch.pin.number} released.")

    def reset_after_stop(self):
        """Clear the stop flag and reset the motor to continue operation."""
        if self.stop_flag.is_set():
            self.stop_flag.clear()
            print(f"Motor {self.motor_id}: Resetting motor after limit switch activation.")

    def check_limit_switches(self):
        """Check the status of limit switches before movement and set direction accordingly."""
        clockwise = False  # Default direction
        # Check each limit switch independently
        for limit_switch in self.limit_switches:
            if limit_switch.is_pressed:
                if limit_switch.pin.number == 6:  # Assuming Limit Switch 1 is on pin 6
                    print(f"Motor {self.motor_id}: Limit switch 1 pressed. Reversing direction.")
                    clockwise = False  # Reverse direction for Limit Switch 1
                elif limit_switch.pin.number == 12:  # Assuming Limit Switch 2 is on pin 12
                    print(f"Motor {self.motor_id}: Limit switch 2 pressed. Moving forward.")
                    clockwise = True  # Set direction for Limit Switch 2

        # Set motor direction based on limit switch status
        self.set_direction(clockwise)


class MotorController:
    def __init__(self, motors):
        self.motors = motors

    def enable_all(self):
        for motor in self.motors:
            motor.enable()

    def disable_all(self):
        for motor in self.motors:
            motor.disable()

    def move_all(self, steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min):
        threads = []
        for motor in self.motors:
            thread = threading.Thread(
                target=motor.move_steps,
                args=(steps, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min),
            )
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()


# Initialize motors and limit switches
limit_switch_1 = Button(6, pull_up=True, bounce_time=0.0005)
limit_switch_2 = Button(12, pull_up=True, bounce_time=0.0005)

motor1 = StepperMotor(dir_pin=27, step_pin=17, enable_pin=22, motor_id=1)
motor2 = StepperMotor(dir_pin=26, step_pin=16, enable_pin=25, motor_id=2)

# Attach limit switches (optional)
motor2.attach_limit_switch(limit_switch_1)
motor2.attach_limit_switch(limit_switch_2)

# Create a motor controller
controller = MotorController(motors=[motor1])

angle = 5
numb_full_rota = angle / 360
reduction_factor = 9.5
micro_stepping = 4
steps_per_revolution = micro_stepping * 200
steps_motor = int(steps_per_revolution * numb_full_rota * reduction_factor)

rpm_start = 2
rpm_max = 20
accel_fraction = 0.1
decel_fraction = 0.1
rpm_min = 1

# Main execution
try:

    controller.enable_all()
    for i in range(40):
        print(f"Iteration number {i} {i % 2 }{motor1.dir_pin}{not motor1.dir_pin.value}")
        #motor1.set_direction(True)  # Set clockwise if currently 0 (False)

        # Change direction of motor1 periodically (every 10 iterations for example)
        if i % 2 == 0:  # Change direction every 10 iterations
            # Toggle direction based on current value
            print(motor1.dir_pin.value)
            motor1.set_direction(False)  # Set clockwise if currently 0 (False)
            print(f"Motor 1 direction changed at iteration {i} to {'CW' if motor1.dir_pin.value else 'CCW'}")
            
        # Before starting motor2, check the limit switches and set direction
        ##motor2.check_limit_switches()

        controller.move_all(steps_motor, steps_per_revolution, rpm_start, rpm_max, accel_fraction, decel_fraction, rpm_min)

        # Check if motor2 stopped due to limit switch
        #if motor2.stop_flag.is_set():
        #    print("Motor 2 stopped by limit switch.")
        #    motor2.reset_after_stop()

        time.sleep(1)

    controller.disable_all()
    print("Motors disabled!")
except Exception as e:
    controller.disable_all()
    print(f"An error occurred: {e}")
