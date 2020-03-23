# Differential-Drive-Robot-Motion
A MATLAB Script for simulating a simple wheeled mobile robot in a 2D environment and exploring kinematic models used for dead reckoning position tracking.

## Motion Models for Differential Drive Mobile Robots
In any mobile robotics project knowing how to estimate your robot's position is fundamental for any application. Determining where your robot is within its environment requires some kind of model to estimate its position based on control inputs to the robot. Most commonly, we use the robot's linear velocity and rotational velocity as the inputs to the kinematic models. These velocities can come from a variety of sources, such as commanded velocities to the robot, inertial measurement units, or wheel encoders.

For this application, we'll consider one of the most common scenarios: tracking the 2D pose of a differential drive robot using wheel encoders.

## Defining Variables

Before jumping into the code, let's take a moment to define some terms and give some basic background information you have probably seen before. But for the sake of completion and for readers who may be encountering this information for the first time, I will be rehashing the fundamentals here.

The first thing to define is the "global frame" or the "world frame". This is the coordinate system used to define the environment in which the robot exists. For this project we are considering a 2D robot, so it will exist in a 2D x,y global coordinate system.

We will define the 2D pose of the robot as the following:
<img src="https://render.githubusercontent.com/render/math?math=p_r = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}">

The pose of the robot corresponds to its x,y position and theta orientation within the global frame. The robot is modeled as a single point with some orientation.

The control input for the robot will be derived from its wheel enocders. The wheel encoders measure the rotations of the robot's wheels, which can be used to determine the velocity of each wheel. Typically, this value is produced in radians per second. We will want to transform this value so we get meters per secod for each wheel:
<img src="https://render.githubusercontent.com/render/math?math=v_{m/s} = v_{rad/s} \frac{d}{2\pi}">
Where d is the wheel diameter. This equation gives us the wheel velocity in mps. This will need to be performed for each tire of the robot.

