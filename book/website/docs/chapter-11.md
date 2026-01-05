---
sidebar_position: 12
---

# Chapter 11: Navigation & Control

## Learning Objectives

By the end of this chapter, the reader will be able to:
- Understand Navigation2 (Nav2) architecture
- Configure navigation for humanoid robots
- Implement path planning algorithms
- Set up obstacle avoidance systems
- Integrate navigation with perception systems

## 11.1 Introduction to Navigation2 (Nav2)

Navigation2 (Nav2) is the ROS 2 navigation stack that provides path planning, obstacle avoidance, and localization capabilities for mobile robots. It's the successor to the ROS 1 navigation stack with significant improvements in architecture and functionality.

Nav2 components include:
- **Navigator**: Coordinates the navigation process
- **Planner Server**: Global path planning
- **Controller Server**: Local path following
- **Recovery Server**: Behavior for getting unstuck
- **BT Navigator**: Behavior tree-based navigation logic

## 11.2 Global Path Planning

Global path planning finds a path from the robot's current location to the goal in a known map:

### A* Algorithm
- Finds optimal path considering cost
- Uses heuristic to guide search
- Guarantees optimal solution if admissible heuristic is used

### Dijkstra's Algorithm
- Explores all possible paths
- Guarantees optimal solution
- More computationally expensive than A*

## 11.3 Local Path Following and Control

Local path following keeps the robot on the global path while avoiding obstacles:

### Trajectory Rollout
- Evaluates multiple possible trajectories
- Selects the best one based on criteria
- Considers robot dynamics and constraints

### Dynamic Window Approach (DWA)
- Considers robot's kinodynamic constraints
- Evaluates velocities in a feasible window
- Balances goal reaching and obstacle avoidance

## 11.4 Costmap Configuration

Costmaps represent the environment with different cost values:
- **Static Layer**: Permanent obstacles from the map
- **Obstacle Layer**: Dynamic obstacles from sensors
- **Inflation Layer**: Safety margins around obstacles

Costmap parameters:
- Resolution: Size of each cell in meters
- Robot footprint: Physical size of the robot
- Inflation radius: Safety margin around obstacles

## 11.5 Navigation for Humanoid Robots

Navigation for humanoid robots has unique challenges:
- Bipedal locomotion constraints
- Balance requirements
- Different kinematics than wheeled robots
- Step planning for walking

Special considerations:
- Footstep planning algorithms
- Center of mass management
- Balance recovery behaviors
- Terrain adaptability

## 11.6 Behavior Trees in Navigation

Nav2 uses behavior trees for navigation logic:
- Composable navigation behaviors
- Reactive to environmental changes
- Modular and configurable

Common behavior tree nodes:
- **Sequences**: Execute children in order until one fails
- **Fallbacks**: Try children until one succeeds
- **Decorators**: Modify child behavior
- **Conditions**: Check environment conditions
- **Actions**: Execute navigation tasks

## 11.7 Chapter Summary

In this chapter, we explored Navigation2 and its components for robot navigation. We covered path planning algorithms, costmap configuration, and special considerations for humanoid robots.

In the next chapter, we'll explore Vision-Language-Action (VLA) systems.

## Exercises

1. Configure Nav2 for a simple robot model
2. Implement a custom path planner plugin
3. Test navigation with dynamic obstacles

---