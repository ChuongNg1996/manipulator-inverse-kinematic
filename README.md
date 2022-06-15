# Manipulator - Inverse Kinematic
A simple implementation of Inverse Kinematic (IK) for Robot Manipulator
(Building)

## Project Description
The project presents a simple implementation of inverse kinematic on robot manipulator with pseudoinverse Jacobian matrix.

The project uses: 
* [ROS Framework](http://wiki.ros.org/) (on Ubuntu) to alleviate concurrency management and module communication.
* [Gazebo](https://gazebosim.org/home) with [UR3 Manipulator](https://github.com/ros-industrial/universal_robot) as testing subject.

## Algorithms & Concepts
The flow of the program to solve IK for a given **Goal Position of End Effector** `[xr,yr,zr]` is presented:
1. Read the **Current Joint Positions** `[theta_1, ..., theta_N]` and use **Forward Kinematic** to derive **Current Position of End Effector** `[xf,yf,zf]`.
2. Define the the number of intermediate segments `I` between `[xr,yr,zr]` and `[xf,yf,zf]` on all 3 axes to derive the **Sub Goals** `[x_sub1, x_sub2, ... x_sub(n), ... xr]`, `[y_sub1, y_sub2, ... y_sub(n) ,... yr]`, `[z_sub1, z_sub2, ... z_sub(n), ... zr]` . Assuming that on each axis, the distances between two **Sub Goals** `[delta_x,delta_y,delta_z]` (e.g., `delta_x = x_sub(n) - x_sub(n-1)`, ...) stay the same, then derive `[delta_x,delta_y,delta_z]` directly (e.g., `delta_x = (xr - xf)/I`, ...)
3. Create a For Loop of `I` iterations. At each iteration, compute the **pseudoinverse Jacobian matrix** using `[theta_1, ..., theta_N]`, then multiply it with `[delta_x,delta_y,delta_z]` (or, again, distance between each sub goal) to get the **CHANGE IN ANGLE** of each joint required to the next **Sub Goal** `[delta_theta_1, ..., delta_theta_N]`. Then sum both **Current Joint Positions** `[theta_1, ..., theta_N]` with the **CHANGE IN ANGLE** required `[delta_x,delta_y,delta_z]` to derive the new **Current Joint Positions** `[theta_1, ..., theta_N]` for the next loop.
4. The final **Current Joint Positions** `[theta_1, ..., theta_N]` will be commanded to the robot to reach **Goal Position of End Effector** `[xr,yr,zr]`
