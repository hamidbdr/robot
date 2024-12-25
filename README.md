# Contolre the speed acceleration of a Stepper motor using Raspberry Pi 5

Script to control a stepper motor using Raspberry Pi 5, the code has 3 phases:
- acceleration (increasing the speed)
- hold max speed 
- deceleration (decreasing the speed)

it also contains some visualizations of the RPMs and the time deltas (delays) used to construct the pulses to drive the stepper.

The code allows you to specify the max RPMs, initial RPMs, and number of full rotations. If you have some reduction factor, else just put it to 1. You can also provide the micro stepping and two fractions: one for the acceleration phase and the second for the deceleration phase.
