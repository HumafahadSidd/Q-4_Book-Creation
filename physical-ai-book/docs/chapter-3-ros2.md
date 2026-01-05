---
sidebar_position: 4
---

# Chapter 3: ROS 2 Architecture

## Learning Objectives

By the end of this chapter, the reader will be able to:
- Understand the core concepts of ROS 2 architecture
- Explain the roles of nodes, topics, services, and actions
- Describe the DDS communication model
- Identify how ROS 2 enables distributed robotic systems

## 3.1 Introduction to ROS 2

ROS 2 (Robot Operating System 2) is not an operating system but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 and provide features like:
- Improved security
- Better real-time support
- Cross-platform compatibility
- Production readiness

## 3.2 Core Concepts

### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node performs a specific task and communicates with other nodes to achieve complex robot behaviors.

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are data structures that are passed between nodes. The communication is based on a publisher-subscriber model where publishers send messages to topics and subscribers receive messages from topics.

### Services
Services provide a request/reply communication pattern. A service client sends a request to a service server and waits for a response. This is useful for operations that require a specific response or completion confirmation.

### Actions
Actions are similar to services but designed for long-running tasks. They support goal requests, feedback during execution, and result reporting. Actions are ideal for tasks like navigation where you need to know the progress and be able to cancel the operation.

## 3.3 DDS Communication Model

ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware. DDS provides:
- Publisher-subscriber communication
- Service-based communication
- Discovery mechanisms
- Quality of Service (QoS) policies

QoS policies allow fine-tuning of communication behavior:
- Reliability: Best effort or reliable delivery
- Durability: Volatile or transient local data
- History: Keep all samples or only the last few
- Deadline: Maximum time between sample publications

## 3.4 ROS 2 Ecosystem

ROS 2 includes various tools and packages:
- **rclpy**: Python client library for ROS 2
- **rclcpp**: C++ client library for ROS 2
- **Launch**: System for starting multiple nodes
- **Parameters**: Configuration system
- **TF2**: Transform library for coordinate frames
- **Navigation2**: Navigation stack for mobile robots

## 3.5 Chapter Summary

In this chapter, we introduced ROS 2 architecture and its core concepts: nodes, topics, services, and actions. We also discussed the DDS communication model and QoS policies that make ROS 2 suitable for production robotic systems.

In the next chapter, we'll explore how to work with ROS 2 using Python and implement our first ROS 2 nodes.

## Exercises

1. Explain the difference between topics, services, and actions with examples.
2. Describe a scenario where you would use each communication pattern.
3. What are QoS policies and why are they important in robotics?

---