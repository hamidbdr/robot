# robot

script to control a stepper motor using Raspberry Pi 5, the code has 3 phases:
- acceleration (increasing the speed)
- hold max speed 
- deceleration (decreasing the speed)

it also contains some visualizations of the RPMs and the time deltas (delays) used to construct the pulses to drive the stepper.

the code alow to specify the max RPMs, initial RPMs, number of fuull rotation, if you have some reduction factor else just put it to 1,you can also provide the microsteping, and two fraction one for the accelaration phase and the second is for the decelaration phase.
