# Differential-Drive-Robot-Motion

A MATLAB simulation of a simple wheeled mobile robot in a 2D environment, used to explore the kinematic models used for dead-reckoning position tracking of differential drive robots.

## Kinematics Guide

For the educational background on differential drive kinematics — pose representation, wheel velocity conversion, and the simple and ICR (instantaneous center of rotation) forward kinematic models — see [docs/kinematics.md](docs/kinematics.md).

## Repository Contents

- [SIMULATION_RobotMotion.m](SIMULATION_RobotMotion.m) — Entry point script. Drives a simulated robot along a sinusoidal wheel-velocity path and animates its motion.
- [differentialDriveKinematics.m](differentialDriveKinematics.m) — Computes the robot's updated pose given its current pose, linear/rotational velocity, and elapsed time. Supports both the `simple` (linear-then-rotate) model and the `icr` (instantaneous center of rotation) model described in the kinematics guide.
- [drawRobot.m](drawRobot.m) — Renders the robot as a circle with a directional arrow at a given pose.
- [docs/kinematics.md](docs/kinematics.md) — Educational write-up of the differential drive kinematics models, with diagrams and equations.

## Running the Code

1. Clone this repository.
2. Open the folder in MATLAB.
3. Run `SIMULATION_RobotMotion.m`.

By default, the script drives the simulated robot along a sinusoidal path using the `icr` kinematic model and animates the result in a figure window. Edit the `model` variable in [SIMULATION_RobotMotion.m](SIMULATION_RobotMotion.m) to switch between `'icr'` and `'simple'`.