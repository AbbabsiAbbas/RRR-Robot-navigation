# Robot Arm Kinematics Simulation

## Overview

This project simulates the kinematics of a robotic arm in a 3D space, allowing for visualization of the arm's movement towards a designated product position while avoiding collisions with obstacles.

## Features

- **Direct Kinematics**: Computes the end-effector position based on joint angles.
- **Inverse Kinematics**: Calculates joint angles required to reach a specified position.
- **Path Planning**: Generates a path for the robot arm to follow from a start position to an end position.
- **Collision Checking**: Detects potential collisions with obstacles during movement.
- **Visualization**: Plots the robot arm's trajectory and visualizes the joint angles over time.

## Requirements

Make sure to have the following libraries installed:

- `numpy`
- `matplotlib`
- `scipy`

You can install the required libraries using:

```bash
pip install numpy matplotlib scipy
Code Explanation
Key Classes and Methods
robot_arm: The main class representing the robotic arm.
Attributes:
l1, l2, l3: Lengths of the arm segments.
center, ro, height: Parameters for obstacle avoidance.
lim_min, lim_max: Joint angle limits.
---Methods:
init(): Initialization method.
gen_rand_conf(): Generates random joint configurations within limits.
direct_kinematics(q): Calculates the end-effector position given joint angles q.
inverse_kinematics(position): Determines joint angles to achieve a specified position.
get_indix_of_min_distance_point(Q_free, point): Finds the index of the closest point in the configuration space.
find_path(X_free, start, end): Computes a path from start to end within valid configurations.
collision_checker(x, circle_nb): Checks for potential collisions.
insert_points(path, n_new_points): Interpolates between points in a path.
get_path_angles(path): Calculates joint angles along a specified path.
calculate_angles(path): Computes angles related to the Z-axis.
Main Functionality
The main execution involves:

Creating an instance of the robot_arm class.
Generating a point cloud in configuration space (Q_free) and workspace (X_free).
Using the start and end points to find a collision-free path.
Visualizing the path, angles, and robot movement.
---Visualization
The simulation generates several plots:

The 3D trajectory of the robot arm.
A plot showing joint angles relative to time.
A 3D scatter plot of joint angles throughout the movement.
An animated visualization of the robot arm moving along the calculated path.
Contributing
Feel free to fork the repository and submit pull requests for improvements or additional features!

---License
This project is open-source and available for modification and distribution.
