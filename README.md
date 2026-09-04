# 2-DOF-AI-mathematical-robot
Build an intelligent robotic manipulation pipeline that can perceive a target, determine a valid configuration, plan a movement, and execute it.  

# 2-DOF AI Mathematical Robot

## Project Overview

This project develops a 2-DOF robotic arm from mathematical modeling and simulation toward an intelligent physical robot.

The initial objective is to build a mathematical model of a two-link planar robotic arm, simulate its movement, automatically generate data from its configuration space, and use a neural network to learn the relationship between the robot's Cartesian position and its joint angles.

The project will progressively develop from:

**Mathematical Modeling → Simulation → Dataset Generation → Neural Network → Inverse Kinematics → Motion Planning → Grasping → Physical Robot**

The final goal is to demonstrate an end-to-end robotic manipulation system capable of determining a suitable robot configuration, reaching an object, grasping it, and placing it at a target location.

Although the initial system is limited to 2D and two degrees of freedom, the project is intentionally designed as a foundation for extending the same concepts to higher-DOF and 3D robotic arms.

