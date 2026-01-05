---
sidebar_position: 16
---

# Chapter 15: Humanoid Kinematics

## Learning Objectives

By the end of this chapter, the reader will be able to:
- Understand forward and inverse kinematics for humanoid robots
- Implement kinematic models for humanoid robots
- Solve kinematic problems for manipulation tasks
- Understand balance and center of mass management
- Apply kinematic constraints in humanoid systems

## 15.1 Introduction to Humanoid Kinematics

Humanoid kinematics is the study of motion in human-like robots without considering the forces that cause the motion. Understanding kinematics is crucial for controlling humanoid robots, especially for tasks like walking, reaching, and manipulation.

Key concepts in humanoid kinematics:
- Forward kinematics: Calculating end-effector position from joint angles
- Inverse kinematics: Calculating joint angles to achieve desired end-effector position
- Jacobian matrices: Relating joint velocities to end-effector velocities
- Kinematic chains: Series of connected links and joints

## 15.2 Forward Kinematics

Forward kinematics calculates the position and orientation of the end-effector given the joint angles. For a humanoid robot, this involves:

1. Defining the kinematic chain from base to end-effector
2. Using transformation matrices for each joint
3. Multiplying transformations to get the final position

For a simple arm with 3 joints:
```
T_total = T_base * T1(θ1) * T2(θ2) * T3(θ3)
```

Where T represents transformation matrices.

## 15.3 Inverse Kinematics

Inverse kinematics solves the opposite problem: finding joint angles to achieve a desired end-effector position. This is more complex and often has multiple solutions or no solution.

Common approaches:
- Analytical solutions: For simple kinematic chains
- Numerical methods: Jacobian-based approaches
- Optimization-based methods: Minimizing error functions

## 15.4 Humanoid Kinematic Structure

A typical humanoid robot has multiple kinematic chains:
- Left arm: Shoulder, elbow, wrist joints
- Right arm: Shoulder, elbow, wrist joints
- Left leg: Hip, knee, ankle joints
- Right leg: Hip, knee, ankle joints
- Torso and head: Spine, neck joints

Each chain has its own kinematic model and constraints.

## 15.5 Balance and Center of Mass

Maintaining balance is critical for humanoid robots:
- Center of Mass (CoM): Average position of all mass
- Zero Moment Point (ZMP): Point where net moment of ground reaction forces is zero
- Support polygon: Convex hull of ground contact points

For stability, the ZMP must remain within the support polygon.

## 15.6 Kinematic Constraints

Humanoid robots have various kinematic constraints:
- Joint limits: Physical limits on joint angles
- Collision avoidance: Preventing self-collision
- Workspace constraints: Physical limits of reachable space
- Balance constraints: Maintaining stability

## 15.7 Implementation Example

Using the KDL (Kinematics and Dynamics Library) in ROS:

```python
import PyKDL
from math import pi

# Create a simple kinematic chain
chain = PyKDL.Chain()

# Add joints and segments
chain.addSegment(PyKDL.Segment(
    PyKDL.Joint(PyKDL.Joint.RotZ),
    PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.1))
))

chain.addSegment(PyKDL.Segment(
    PyKDL.Joint(PyKDL.Joint.RotY),
    PyKDL.Frame(PyKDL.Vector(0.0, 0.0, 0.3))
))

# Forward kinematics solver
fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)

# Joint angles
joint_array = PyKDL.JointArray(2)
joint_array[0] = 0.5  # radians
joint_array[1] = 0.3  # radians

# Calculate end-effector position
end_frame = PyKDL.Frame()
fk_solver.JntToCart(joint_array, end_frame)

print(f"End-effector position: ({end_frame.p.x()}, {end_frame.p.y()}, {end_frame.p.z()})")
```

## 15.8 Chapter Summary

In this chapter, we explored humanoid kinematics, including forward and inverse kinematics, balance considerations, and implementation approaches. Understanding kinematics is essential for controlling humanoid robots effectively.

In the next chapter, we'll cover manipulation and grasping strategies.

## Exercises

1. Implement forward kinematics for a simple humanoid arm
2. Solve inverse kinematics for a reaching task
3. Calculate the center of mass for a humanoid model

---