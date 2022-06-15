# Manipulator - Inverse Kinematic
A simple implementation of Inverse Kinematic (IK) for Robot Manipulator
(Building)

## Project Description
The project presents a simple implementation of inverse kinematic on robot manipulator with pseudoinverse Jacobian matrix

The project uses: 
* [ROS Framework](http://wiki.ros.org/) (on Ubuntu) to alleviate concurrency management and module communication.
* [Gazebo](https://gazebosim.org/home) with [UR3 Manipulator](https://github.com/ros-industrial/universal_robot) as testing subject.

## Algorithms & Concepts
The flow of the program to solve IK for a given Goal Position `[xr,yr,zr]` is presented:
1. Read the current Joint Position `[theta_1, ..., theta_N]` and use Forward Kinematic to derive Current Position `[xf,yf,zf]` of End Effector.
2. Define the the number of intermediate segments `I` between `[xr,yr,zr] and [xf,yf,zf] on all 3 axes to derive the Sub Goals `[x_sub1, x_sub2, ... x_sub(n), ... xr]`, `[y_sub1, y_sub2, ... y_sub(n) ,... yr]`, `[z_sub1, z_sub2, ... z_sub(n), ... zr]` . Assuming that on each axis, the distance between two Sub Goals `[delta_x,delta_y,delta_z]` (where delta_x = x_sub(n) - ) are the 
