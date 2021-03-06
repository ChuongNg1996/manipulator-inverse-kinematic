# Manipulator - Inverse Kinematic
A simple implementation of Inverse Kinematic (IK) for Robot Manipulator
(Building)

## Project Description
The project presents a simple implementation of inverse kinematic on robot manipulator with pseudoinverse Jacobian matrix.

The project uses: 
* [ROS Framework](http://wiki.ros.org/) (on Ubuntu) to alleviate concurrency management and module communication.
* [Gazebo](https://gazebosim.org/home) with [UR3 Manipulator](https://github.com/ros-industrial/universal_robot) as testing subject.

## Algorithms & Concepts
The flow of the **program** to solve IK for a given **Goal Position of End Effector** `[xr,yr,zr]` is presented (**Orietation** is discussed after that shortly):
1. Read the **Current Joint Positions** `[theta_1, ..., theta_N]` and use **Forward Kinematic** to derive **Current Position of End Effector** `[xf,yf,zf]`.
2. Define the the number of intermediate segments `I` between `[xr,yr,zr]` and `[xf,yf,zf]` on all 3 axes to derive the **Sub Goals** `[x_sub1, x_sub2, ... x_sub(n), ... xr]`, `[y_sub1, y_sub2, ... y_sub(n) ,... yr]`, `[z_sub1, z_sub2, ... z_sub(n), ... zr]` . Assuming that on each axis, the distances between two **Sub Goals** `[delta_x,delta_y,delta_z]` (e.g., `delta_x = x_sub(n) - x_sub(n-1)`, ...) stay the same, then derive `[delta_x,delta_y,delta_z]` directly (e.g., `delta_x = (xr - xf)/I`, ...)
3. Create a For Loop of `I` iterations. At each iteration, compute the **pseudoinverse Jacobian matrix** using `[theta_1, ..., theta_N]`, then multiply it with `[delta_x,delta_y,delta_z]` (or, again, distance between each sub goal) to get the **CHANGE IN ANGLE** of each joint required to the next **Sub Goal** `[delta_theta_1, ..., delta_theta_N]`. Then sum both **Current Joint Positions** `[theta_1, ..., theta_N]` with the **CHANGE IN ANGLE** required `[delta_x,delta_y,delta_z]` to derive the new **Current Joint Positions** `[theta_1, ..., theta_N]` for the next loop.
4. The final **Current Joint Positions** `[theta_1, ..., theta_N]` will be commanded to the robot to reach **Goal Position of End Effector** `[xr,yr,zr]`

The components are derived:
* **Goal Position of End Effector** `[xr,yr,zr]` are given by user.
* **Current Joint Positions** `[theta_1, ..., theta_N]` are read from sensors of each joint.
* **Current Position of End Effector** `[xf,yf,zf]` is derived from `[theta_1, ..., theta_N]` using **Forward Kinematic**.
* **Forward Kinematic** formulas are mathematically derived based on robot setting before running the program (using symbolic functions and matrix mutiplication in MATLAB).
* **pseudoinverse Jacobian matrix** formulas are mathematically derived based on robot setting before running the program (using symbolic functions, [diff()](https://www.mathworks.com/help/matlab/ref/diff.html) for partial derivative and [pinv()](https://www.mathworks.com/help/matlab/ref/pinv.html) for Moore-Penrose pseudoinverse in MATLAB).

Thus, the essence of the method lies on the formula: 

<p align="center"> <b> <i> INVERSE_JACOBIAN * DELTA_END_POSE = DELTA_ANGLE </i> </b> </p>

where **INVERSE_JACOBIAN** is the inverse Jacobian matrix, **DELTA_END_POSE** is the distance between each **Sub Goals** and **DELTA_ANGLE** is the computed **CHANGE IN ANGLE** required at each time step to reach the **Sub Goals** (and eventually the **Goal Position of End Effector**). For [more](https://homes.cs.washington.edu/~todorov/courses/cseP590/06_JacobianMethods.pdf) about the formula. Since the Jacobian matrix is often non-squared, direct inverse is not impossible. Thus, **pseudoinverse** is used instead. The formula then become:

<p align="center"> <b> <i> PSEUDO_INVERSE_JACOBIAN * DELTA_END_POSE = DELTA_ANGLE </i> </b> </p>

Assuming that we want to keep the **Orietation** of the **End Effector** the same (e.g. always pointing downward) throughout all operations, then we set such orientation at **HOME Position** for all joints (when the program is initially started). And, for **4-DOF articulated robot arm** for example, angle of the 4th joint will be:

<p align="center"> <b> <i> COMMAND_JOINT_4 = HOME_JOINT_4 (+/-) (COMMAND_JOINT_2 + COMMAND_JOINT_3) </i> </b> </p>

This is specificed for each arm configuration.

## Installation & Implementation
* Install C++.
* [Install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) (any version).
* At /usr/home/"name" ("name" is arbitrary), create a ROS workspace. On terminal: 
   ```sh
   mkdir -p ~/catkin_ws/src
   ```
* Go to the created ROS workspace, clone the repo and build them. On terminal: 
   ```sh
   cd ~/catkin_ws/src
   git clone https://github.com/ros-industrial/universal_robot
   git clone https://github.com/ChuongNg1996/manipulator-inverse-kinematic
   cd ..
   catkin_make
   ```
* Run the example. On terminal 1:
   ```sh
   source ~/catkin_ws/devel/setup.bash
   roslaunch ur_gazebo ur3.launch 
   ```
* On terminal 2:
   ```sh
   source ~/catkin_ws/devel/setup.bash
   rosrun inverse_kinematic_1 inverse_kinematic_1_test 
   ```
* Test the package by releasing point command on ROS topic `/pose_command`.On terminal 3:
   ```sh
   rostopic pub -1 /pose_command geometry_msgs/Point "x: 0.4
   y: 0.2
   z: 0.2" 
   ```
## API
To be updated

## Debugging 
* Use `rostopic list` to see available ROS topics.
* Number of iteration to solve for IK has to be sufficiently large.

## Other
(None)

## To Do
* inline functions
