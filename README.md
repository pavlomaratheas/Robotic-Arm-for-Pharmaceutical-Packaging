# Robotic Arm for Pharmaceutical Packaging

**Bachelor's Thesis**: Design and simulation of a robotic arm for precise drug transport and packaging using MATLAB Robotics System Toolbox.

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023a-brightorange.svg)](https://mathworks.com)
[![Robotics Toolbox](https://img.shields.io/badge/Robotics%20System%20Toolbox-v4.0-blue.svg)](https://mathworks.com/help/robotics)

## Project Summary

Developed a **6-DOF robotic arm** (Robotis OpenManipulator) for automated pharmaceutical packaging. The system picks small packages from a conveyor, navigates obstacles, and places them accurately in a target container.

**Key Achievements**:
- **Motion Planning**: Smooth trajectory generation with B-spline interpolation.
- **Inverse Kinematics**: Real-time joint solutions for precise end-effector positioning.
- **Collision Avoidance**: Wall detection with emergency stop.
- **Pharma-Grade Precision**: Sub-millimeter accuracy for drug handling.

## Features

- **Pick & Place**: From conveyor `[0.2, -0.2, 0.05]` to container `[0.2, 0.2, 0.05]`.
- **Obstacle Navigation**: Clear 10cm wall at `[0.2, 0, 0.05]`.
- **Collision Detection**: Real-time end-effector monitoring.
- **Animated Simulation**: 30-step trajectory visualization.
- **Environment Modeling**: Conveyor, wall, target box, animated cube payload.

### Core Algorithms

```matlab
% Robot loading & IK setup
robot = loadrobot("robotisOpenManipulator");
ik = robotics.InverseKinematics('RigidBodyTree', robot);

% Smooth trajectory generation
wayPoints = [startPoint; 0.2,-0.2,0.1; 0.2,0,0.15; 0.2,0.2,0.1; endPoint];
trajectory = cscvn(wayPoints');
eePositions = ppval(trajectory, linspace(0, 1, 30));
