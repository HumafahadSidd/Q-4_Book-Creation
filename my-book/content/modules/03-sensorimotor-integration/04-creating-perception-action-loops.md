# Chapter 3.4: Creating Perception-Action Loops

## Learning Objectives

By the end of this chapter, students will be able to:
- Design and implement perception-action loops for robotic systems
- Integrate sensor processing, decision-making, and actuation components
- Apply control theory principles to sensorimotor systems
- Implement reactive and deliberative control strategies
- Create closed-loop systems with appropriate feedback mechanisms
- Evaluate the stability and performance of perception-action loops
- Design robust loops that handle sensor noise and uncertainty

## Introduction

Perception-action loops are the fundamental mechanism by which physical AI systems interact with the real world. These loops continuously process sensory information, make decisions based on that information, and execute actions that affect the environment, which in turn affects future sensory inputs. Understanding how to design and implement effective perception-action loops is crucial for developing intelligent robotic systems.

The perception-action loop embodies the principles of embodied intelligence by tightly coupling sensing, processing, and acting. Unlike digital AI systems that operate on abstract data, physical AI systems must operate in real-time with noisy, uncertain, and incomplete information. The perception-action loop provides the framework for handling these challenges while enabling adaptive behavior.

In this chapter, we'll explore how to design, implement, and optimize perception-action loops for various robotic applications. We'll cover both simple reactive systems and more complex deliberative approaches, examining the trade-offs between different control strategies.

## Understanding Perception-Action Loops

### The Basic Loop Structure

The perception-action loop consists of four fundamental stages:

```
Perception → Decision → Action → Environment → Perception
```

1. **Perception**: Sensors gather information about the environment and robot state
2. **Decision**: Information is processed to determine appropriate actions
3. **Action**: Actuators execute physical actions based on decisions
4. **Environment**: Actions affect the environment, which affects future perception

### Characteristics of Effective Loops

**Real-time Operation**: The loop must complete within the time constraints of the application.

**Robustness**: The system must continue operating despite sensor noise, failures, and environmental changes.

**Adaptability**: The system should adjust its behavior based on changing conditions.

**Stability**: The system should converge to appropriate behaviors rather than oscillating.

## Reactive Control Systems

Reactive systems respond directly to sensory input without complex planning. They are characterized by simple if-then rules that map sensory inputs to actions.

### Simple Reactive Architecture

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ReactiveController : public rclcpp::Node
{
public:
    ReactiveController() : Node("reactive_controller")
    {
        // Create subscription to laser scan
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ReactiveController::scanCallback, this, std::placeholders::_1));
        
        // Create publisher for velocity commands
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Reactive controller initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process scan data to find minimum distance
        float min_distance = std::numeric_limits<float>::max();
        int min_index = -1;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] < min_distance && 
                msg->ranges[i] > msg->range_min && 
                msg->ranges[i] < msg->range_max) {
                
                min_distance = msg->ranges[i];
                min_index = i;
            }
        }
        
        // Create velocity command based on scan
        auto cmd_msg = geometry_msgs::msg::Twist();
        
        if (min_index != -1 && min_distance < safety_distance_) {
            // Obstacle detected - turn away
            float angle_to_obstacle = msg->angle_min + min_index * msg->angle_increment;
            
            if (angle_to_obstacle < 0) {
                // Obstacle on the left - turn right
                cmd_msg.linear.x = 0.0;  // Stop forward motion
                cmd_msg.angular.z = -angular_speed_;  // Turn right
            } else {
                // Obstacle on the right - turn left
                cmd_msg.linear.x = 0.0;  // Stop forward motion
                cmd_msg.angular.z = angular_speed_;   // Turn left
            }
            
            RCLCPP_WARN(
                this->get_logger(),
                "Obstacle detected at %.2f m, turning away",
                min_distance
            );
        } else {
            // Clear path - move forward
            cmd_msg.linear.x = linear_speed_;
            cmd_msg.angular.z = 0.0;
        }
        
        cmd_publisher_->publish(cmd_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    
    // Parameters
    float safety_distance_ = 1.0;
    float linear_speed_ = 0.5;
    float angular_speed_ = 0.8;
};
```

### Behavior-Based Robotics

Behavior-based robotics decomposes complex behaviors into simple, reactive components:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Base class for behaviors
class Behavior
{
public:
    virtual geometry_msgs::msg::Twist calculateCommand() = 0;
    virtual bool isActive() const = 0;
    virtual ~Behavior() = default;
};

// Avoid obstacles behavior
class AvoidObstacles : public Behavior
{
public:
    AvoidObstacles(float* ranges, size_t size, float min_range, float max_range) 
        : ranges_(ranges), size_(size), min_range_(min_range), max_range_(max_range) {}
    
    geometry_msgs::msg::Twist calculateCommand() override
    {
        geometry_msgs::msg::Twist cmd;
        
        // Find closest obstacle in front
        float min_dist = max_range_;
        for (size_t i = size_/4; i < 3*size_/4; ++i) {  // Front 180 degrees
            if (ranges_[i] > min_range_ && ranges_[i] < min_dist) {
                min_dist = ranges_[i];
            }
        }
        
        if (min_dist < 1.0) {
            // Obstacle detected - turn
            cmd.angular.z = 0.5;
        }
        
        return cmd;
    }
    
    bool isActive() const override { return true; }  // Always active

private:
    float* ranges_;
    size_t size_;
    float min_range_, max_range_;
};

// Move forward behavior
class MoveForward : public Behavior
{
public:
    MoveForward(float target_speed) : target_speed_(target_speed) {}
    
    geometry_msgs::msg::Twist calculateCommand() override
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = target_speed_;
        return cmd;
    }
    
    bool isActive() const override { return true; }

private:
    float target_speed_;
};

// Behavior coordinator
class BehaviorCoordinator
{
public:
    void addBehavior(std::unique_ptr<Behavior> behavior)
    {
        behaviors_.push_back(std::move(behavior));
    }
    
    geometry_msgs::msg::Twist execute()
    {
        geometry_msgs::msg::Twist final_cmd;
        
        for (const auto& behavior : behaviors_) {
            if (behavior->isActive()) {
                auto cmd = behavior->calculateCommand();
                
                // Arbitrate between behaviors (simple priority-based)
                if (std::abs(cmd.angular.z) > std::abs(final_cmd.angular.z)) {
                    final_cmd.angular.z = cmd.angular.z;
                }
                if (std::abs(cmd.linear.x) < std::abs(final_cmd.linear.x)) {
                    final_cmd.linear.x = cmd.linear.x;  // Lower priority: reduce speed
                }
            }
        }
        
        return final_cmd;
    }

private:
    std::vector<std::unique_ptr<Behavior>> behaviors_;
};
```

## Deliberative Control Systems

Deliberative systems perform planning and reasoning before acting. They maintain internal models of the world and use these models to plan sequences of actions.

### Planning-Based Architecture

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DeliberativeController : public rclcpp::Node
{
public:
    DeliberativeController() : Node("deliberative_controller")
    {
        // Subscriptions
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&DeliberativeController::mapCallback, this, std::placeholders::_1));
        
        goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&DeliberativeController::goalCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DeliberativeController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Deliberative controller initialized");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = msg;
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Map received with resolution: %.2f", msg->info.resolution);
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = msg->pose;
        goal_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal received");
        
        // Plan path to goal
        if (map_received_) {
            planPath();
        }
    }
    
    void planPath()
    {
        if (!map_ || !goal_received_) return;
        
        // Simplified path planning (in practice, use A*, Dijkstra, etc.)
        planned_path_.clear();
        
        // For demonstration, create a straight line to goal
        // In reality, this would involve actual path planning algorithms
        planned_path_.push_back(current_pose_);
        planned_path_.push_back(goal_);
        
        path_index_ = 0;
        following_path_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Path planned with %zu waypoints", planned_path_.size());
    }
    
    void controlLoop()
    {
        if (!following_path_ || planned_path_.empty()) {
            // Stop if not following a path
            auto cmd = geometry_msgs::msg::Twist();
            cmd_publisher_->publish(cmd);
            return;
        }
        
        // Get current waypoint
        if (path_index_ >= planned_path_.size()) {
            // Reached end of path
            following_path_ = false;
            auto cmd = geometry_msgs::msg::Twist();
            cmd_publisher_->publish(cmd);
            return;
        }
        
        auto& current_waypoint = planned_path_[path_index_];
        
        // Calculate distance to waypoint
        double dx = current_waypoint.position.x - current_pose_.position.x;
        double dy = current_waypoint.position.y - current_pose_.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        
        // Check if reached waypoint
        if (distance < waypoint_tolerance_) {
            path_index_++;
            if (path_index_ >= planned_path_.size()) {
                following_path_ = false;
                auto cmd = geometry_msgs::msg::Twist();
                cmd_publisher_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Reached goal!");
                return;
            }
        }
        
        // Generate command to move toward waypoint
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = std::min(linear_speed_, distance * 0.5);  // Slow down as we approach
        cmd.angular.z = calculateAngularCommand(dx, dy);
        
        cmd_publisher_->publish(cmd);
    }
    
    double calculateAngularCommand(double dx, double dy)
    {
        // Calculate angle to goal
        double goal_angle = atan2(dy, dx);
        
        // Get current angle from quaternion
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        double current_yaw;
        tf2::Matrix3x3(q).getRPY(roll_, pitch_, current_yaw);
        
        // Calculate angular error
        double angle_error = goal_angle - current_yaw;
        
        // Normalize angle to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
        // Return angular command
        return std::max(std::min(angle_error * 2.0, max_angular_speed_), -max_angular_speed_);
    }
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    geometry_msgs::msg::Pose goal_;
    geometry_msgs::msg::Pose current_pose_ = {};  // Would come from odometry
    std::vector<geometry_msgs::msg::Pose> planned_path_;
    
    bool map_received_ = false;
    bool goal_received_ = false;
    bool following_path_ = false;
    size_t path_index_ = 0;
    
    // Parameters
    double waypoint_tolerance_ = 0.5;
    double linear_speed_ = 0.5;
    double max_angular_speed_ = 1.0;
    double roll_, pitch_, yaw_;
};
```

## Hybrid Control Systems

Many effective systems combine reactive and deliberative approaches, using reactive behaviors for immediate responses and deliberative planning for long-term goals.

### Subsumption Architecture

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SubsumptionController : public rclcpp::Node
{
public:
    SubsumptionController() : Node("subsumption_controller")
    {
        // Subscriptions
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SubsumptionController::scanCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SubsumptionController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Subsumption controller initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_data_ = msg;
    }
    
    void controlLoop()
    {
        if (!scan_data_) return;
        
        // Level 0: Avoid immediate obstacles (highest priority)
        auto cmd = avoidImmediateObstacles();
        
        // Level 1: Wall following
        if (std::abs(cmd.linear.x) < 0.01 && std::abs(cmd.angular.z) < 0.01) {
            cmd = followWall();
        }
        
        // Level 2: Explore (lowest priority)
        if (std::abs(cmd.linear.x) < 0.01 && std::abs(cmd.angular.z) < 0.01) {
            cmd = explore();
        }
        
        cmd_publisher_->publish(cmd);
    }
    
    geometry_msgs::msg::Twist avoidImmediateObstacles()
    {
        geometry_msgs::msg::Twist cmd;
        
        if (!scan_data_) return cmd;
        
        // Check for immediate obstacles (front 60 degrees)
        float min_dist = std::numeric_limits<float>::max();
        for (size_t i = scan_data_->ranges.size()/2 - 30; i < scan_data_->ranges.size()/2 + 30; ++i) {
            if (i < scan_data_->ranges.size() && 
                scan_data_->ranges[i] > scan_data_->range_min && 
                scan_data_->ranges[i] < min_dist) {
                min_dist = scan_data_->ranges[i];
            }
        }
        
        if (min_dist < 0.5) {
            // Immediate obstacle - emergency stop and turn
            cmd.angular.z = 1.0;  // Turn sharply
            RCLCPP_WARN(this->get_logger(), "Immediate obstacle detected!");
        }
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist followWall()
    {
        geometry_msgs::msg::Twist cmd;
        
        if (!scan_data_) return cmd;
        
        // Simplified wall following
        // Check left side for wall distance
        float left_dist = getRangeAtAngle(-M_PI/2);  // 90 degrees left
        float front_dist = getRangeAtAngle(0);       // Front
        
        if (left_dist > 0.5 && left_dist < 2.0) {
            // Wall detected at appropriate distance
            cmd.linear.x = 0.3;
            cmd.angular.z = (left_dist - 1.0) * 0.5;  // Adjust to maintain distance
        } else if (left_dist >= 2.0) {
            // Too far from wall - turn left
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.3;
        } else if (left_dist > 0 && left_dist < 0.5) {
            // Too close to wall - turn right
            cmd.linear.x = 0.2;
            cmd.angular.z = -0.3;
        }
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist explore()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.4;  // Move forward at moderate speed
        cmd.angular.z = 0.0;
        return cmd;
    }
    
    float getRangeAtAngle(float target_angle)
    {
        if (!scan_data_) return -1.0;
        
        // Find the index corresponding to the target angle
        int index = (target_angle - scan_data_->angle_min) / scan_data_->angle_increment;
        
        if (index >= 0 && index < static_cast<int>(scan_data_->ranges.size())) {
            return scan_data_->ranges[index];
        }
        
        return -1.0;  // Invalid range
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    sensor_msgs::msg::LaserScan::SharedPtr scan_data_;
};
```

## Control Theory in Perception-Action Loops

### PID Control

Proportional-Integral-Derivative (PID) control is widely used in robotics for precise control:

```cpp
class PIDController
{
public:
    PIDController(double kp, double ki, double kd) 
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}
    
    double compute(double setpoint, double process_variable, double dt)
    {
        double error = setpoint - process_variable;
        
        // Proportional term
        double p_term = kp_ * error;
        
        // Integral term
        integral_ += error * dt;
        double i_term = ki_ * integral_;
        
        // Derivative term
        double derivative = (error - prev_error_) / dt;
        double d_term = kd_ * derivative;
        
        // Store error for next iteration
        prev_error_ = error;
        
        return p_term + i_term + d_term;
    }
    
    void reset()
    {
        prev_error_ = 0.0;
        integral_ = 0.0;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};
```

### PID Controller Integration

```cpp
class PIDControlNode : public rclcpp::Node
{
public:
    PIDControlNode() : Node("pid_control_node")
    {
        // Subscriptions
        feedback_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PIDControlNode::feedbackCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PIDControlNode::controlLoop, this));
        
        // Initialize PID controllers
        distance_pid_ = std::make_unique<PIDController>(1.0, 0.1, 0.05);  // Distance control
        angle_pid_ = std::make_unique<PIDController>(2.0, 0.05, 0.1);    // Angle control
        
        RCLCPP_INFO(this->get_logger(), "PID control node initialized");
    }

private:
    void feedbackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_data_ = msg;
    }
    
    void controlLoop()
    {
        if (!scan_data_) return;
        
        // Calculate current distance to obstacle in front
        float current_distance = getFrontDistance();
        
        // Setpoint: maintain 1.5m distance from obstacle
        float setpoint_distance = 1.5;
        
        // Compute control command using PID
        double dt = 0.05;  // 50ms
        double linear_cmd = distance_pid_->compute(setpoint_distance, current_distance, dt);
        
        // Also maintain heading using angle PID
        float current_angle = getAngleToObstacle();
        double angle_cmd = angle_pid_->compute(0.0, current_angle, dt);
        
        // Create and publish command
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = std::max(-0.5, std::min(0.5, linear_cmd));  // Limit speed
        cmd.angular.z = std::max(-1.0, std::min(1.0, angle_cmd));  // Limit turning
        
        cmd_publisher_->publish(cmd);
    }
    
    float getFrontDistance()
    {
        if (!scan_data_) return 0.0;
        
        // Get distance in front (middle of scan)
        size_t mid_idx = scan_data_->ranges.size() / 2;
        return scan_data_->ranges[mid_idx];
    }
    
    float getAngleToObstacle()
    {
        if (!scan_data_) return 0.0;
        
        // Find closest obstacle and its angle
        float min_dist = std::numeric_limits<float>::max();
        int min_idx = -1;
        
        for (size_t i = 0; i < scan_data_->ranges.size(); ++i) {
            if (scan_data_->ranges[i] > scan_data_->range_min && 
                scan_data_->ranges[i] < min_dist) {
                min_dist = scan_data_->ranges[i];
                min_idx = i;
            }
        }
        
        if (min_idx >= 0) {
            return scan_data_->angle_min + min_idx * scan_data_->angle_increment;
        }
        
        return 0.0;
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr feedback_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    sensor_msgs::msg::LaserScan::SharedPtr scan_data_;
    std::unique_ptr<PIDController> distance_pid_;
    std::unique_ptr<PIDController> angle_pid_;
};
```

## Stability and Performance Analysis

### Loop Stability

For a perception-action loop to be stable, it must converge to appropriate behaviors rather than oscillating:

```cpp
class StabilityAnalyzer
{
public:
    struct StabilityMetrics
    {
        double oscillation_frequency;  // Frequency of oscillations
        double settling_time;          // Time to reach steady state
        double overshoot;              // Maximum deviation from target
        bool stable;                   // Whether the system is stable
    };
    
    StabilityMetrics analyze(const std::vector<double>& signal, double threshold = 0.01)
    {
        StabilityMetrics metrics;
        
        // Count zero crossings to detect oscillations
        int zero_crossings = 0;
        bool last_positive = signal[0] > 0;
        
        for (size_t i = 1; i < signal.size(); ++i) {
            bool current_positive = signal[i] > 0;
            if (current_positive != last_positive) {
                zero_crossings++;
                last_positive = current_positive;
            }
        }
        
        metrics.oscillation_frequency = static_cast<double>(zero_crossings) / signal.size();
        
        // Calculate settling time (time to stay within threshold of final value)
        if (!signal.empty()) {
            double final_value = signal.back();
            for (size_t i = 0; i < signal.size(); ++i) {
                if (std::abs(signal[i] - final_value) <= threshold) {
                    metrics.settling_time = i;
                    break;
                }
            }
        }
        
        // Calculate overshoot
        double max_deviation = 0.0;
        double initial_value = signal.empty() ? 0.0 : signal[0];
        for (double value : signal) {
            max_deviation = std::max(max_deviation, std::abs(value - initial_value));
        }
        metrics.overshoot = max_deviation;
        
        // Determine stability based on metrics
        metrics.stable = (metrics.oscillation_frequency < 0.1) && 
                        (metrics.overshoot < 2.0 * threshold);
        
        return metrics;
    }
};
```

## Handling Sensor Noise and Uncertainty

### Robust Control Design

```cpp
class RobustController
{
public:
    RobustController() : 
        nominal_controller_(0.5, 0.1, 0.05),  // Nominal PID
        robust_threshold_(0.2) {}              // Robustness threshold
    
    geometry_msgs::msg::Twist computeCommand(
        double setpoint, 
        const std::vector<double>& sensor_readings,
        double dt)
    {
        // Preprocess sensor data to reduce noise
        auto filtered_readings = preprocessSensors(sensor_readings);
        
        // Calculate statistics of sensor readings
        double mean_reading = calculateMean(filtered_readings);
        double variance = calculateVariance(filtered_readings, mean_reading);
        
        // Adjust control parameters based on sensor uncertainty
        auto cmd = computeRobustCommand(setpoint, mean_reading, variance, dt);
        
        return cmd;
    }

private:
    std::vector<double> preprocessSensors(const std::vector<double>& raw_readings)
    {
        // Apply median filter to remove outliers
        std::vector<double> filtered = raw_readings;
        
        // Sort to find median
        std::sort(filtered.begin(), filtered.end());
        
        // Replace extreme values with median if they're outliers
        double median = filtered[filtered.size() / 2];
        double threshold = 2.0;  // Outlier threshold
        
        for (auto& reading : filtered) {
            if (std::abs(reading - median) > threshold) {
                reading = median;
            }
        }
        
        return filtered;
    }
    
    double calculateMean(const std::vector<double>& readings)
    {
        if (readings.empty()) return 0.0;
        
        double sum = 0.0;
        for (double value : readings) {
            sum += value;
        }
        return sum / readings.size();
    }
    
    double calculateVariance(const std::vector<double>& readings, double mean)
    {
        if (readings.empty()) return 0.0;
        
        double sum_sq_diff = 0.0;
        for (double value : readings) {
            sum_sq_diff += (value - mean) * (value - mean);
        }
        return sum_sq_diff / readings.size();
    }
    
    geometry_msgs::msg::Twist computeRobustCommand(double setpoint, double process_var, double uncertainty, double dt)
    {
        geometry_msgs::msg::Twist cmd;
        
        // Adjust PID parameters based on uncertainty
        double kp = nominal_controller_.getKp() / (1.0 + uncertainty);
        double ki = nominal_controller_.getKi() / (1.0 + uncertainty);
        double kd = nominal_controller_.getKd() / (1.0 + uncertainty);
        
        // Create temporary PID controller with adjusted parameters
        PIDController temp_controller(kp, ki, kd);
        double control_output = temp_controller.compute(setpoint, process_var, dt);
        
        cmd.linear.x = control_output;
        cmd.angular.z = 0.0;  // Simplified for this example
        
        return cmd;
    }
    
    PIDController nominal_controller_;
    double robust_threshold_;
};
```

## Real-time Considerations

### Timing and Synchronization

```cpp
#include <chrono>

class RealTimeController
{
public:
    RealTimeController(int control_frequency) 
        : control_period_(std::chrono::microseconds(1000000 / control_frequency)) {}
    
    void runControlLoop()
    {
        auto next_iteration = std::chrono::steady_clock::now();
        
        while (true) {
            auto start_time = std::chrono::steady_clock::now();
            
            // Execute control iteration
            executeControlIteration();
            
            auto end_time = std::chrono::steady_clock::now();
            auto execution_time = end_time - start_time;
            
            // Calculate remaining time in period
            auto remaining_time = control_period_ - execution_time;
            if (remaining_time > std::chrono::microseconds(0)) {
                std::this_thread::sleep_for(remaining_time);
            } else {
                // Control iteration took too long - log warning
                RCLCPP_WARN(rclcpp::get_logger("real_time_controller"), 
                           "Control iteration exceeded period by %ld microseconds", 
                           -std::chrono::duration_cast<std::chrono::microseconds>(remaining_time).count());
            }
            
            next_iteration += control_period_;
        }
    }

private:
    void executeControlIteration()
    {
        // Perform sensor acquisition
        acquireSensors();
        
        // Process sensor data
        processSensors();
        
        // Execute control algorithm
        auto command = computeControlCommand();
        
        // Send command to actuators
        sendCommand(command);
    }
    
    void acquireSensors() { /* Acquire sensor data */ }
    void processSensors() { /* Process sensor data */ }
    geometry_msgs::msg::Twist computeControlCommand() { /* Compute control command */ }
    void sendCommand(const geometry_msgs::msg::Twist& cmd) { /* Send command to actuators */ }
    
    std::chrono::microseconds control_period_;
};
```

## Performance Evaluation

### Evaluation Metrics

```cpp
class PerformanceEvaluator
{
public:
    struct PerformanceMetrics
    {
        double response_time;      // Time to respond to changes
        double tracking_error;     // Error in following desired trajectory
        double energy_efficiency;  // Energy consumed per unit task
        double robustness_score;   // Performance under disturbances
        double stability_index;    // Measure of system stability
    };
    
    PerformanceMetrics evaluate(
        const std::vector<double>& reference_signal,
        const std::vector<double>& actual_signal,
        const std::vector<double>& control_effort)
    {
        PerformanceMetrics metrics;
        
        // Calculate response time (time to reach 90% of final value)
        metrics.response_time = calculateResponseTime(actual_signal);
        
        // Calculate tracking error (integral of absolute error)
        metrics.tracking_error = calculateIAE(reference_signal, actual_signal);
        
        // Calculate energy efficiency (simplified as integral of control effort squared)
        metrics.energy_efficiency = calculateEnergy(control_effort);
        
        // Calculate robustness (simplified as inverse of error variance)
        metrics.robustness_score = 1.0 / calculateVariance(actual_signal);
        
        // Calculate stability index (simplified as inverse of oscillation frequency)
        metrics.stability_index = 1.0 / calculateOscillationFrequency(actual_signal);
        
        return metrics;
    }

private:
    double calculateResponseTime(const std::vector<double>& signal)
    {
        if (signal.empty()) return 0.0;
        
        double final_value = signal.back();
        double target = 0.9 * final_value;  // 90% of final value
        
        for (size_t i = 0; i < signal.size(); ++i) {
            if (std::abs(signal[i] - final_value) <= 0.1 * std::abs(final_value) && 
                signal[i] >= target) {
                return i;  // Return index as time unit
            }
        }
        
        return signal.size();  // Never reached target
    }
    
    double calculateIAE(const std::vector<double>& ref, const std::vector<double>& actual)
    {
        if (ref.size() != actual.size()) return 0.0;
        
        double iae = 0.0;
        for (size_t i = 0; i < ref.size(); ++i) {
            iae += std::abs(ref[i] - actual[i]);
        }
        
        return iae / ref.size();  // Average IAE
    }
    
    double calculateEnergy(const std::vector<double>& control_effort)
    {
        double energy = 0.0;
        for (double effort : control_effort) {
            energy += effort * effort;  // Proportional to effort squared
        }
        
        return energy / control_effort.size();
    }
    
    double calculateVariance(const std::vector<double>& signal)
    {
        if (signal.empty()) return 0.0;
        
        double mean = 0.0;
        for (double value : signal) {
            mean += value;
        }
        mean /= signal.size();
        
        double variance = 0.0;
        for (double value : signal) {
            variance += (value - mean) * (value - mean);
        }
        
        return variance / signal.size();
    }
    
    double calculateOscillationFrequency(const std::vector<double>& signal)
    {
        if (signal.size() < 2) return 0.0;
        
        // Count zero crossings to estimate frequency
        int zero_crossings = 0;
        double last_diff = signal[1] - signal[0];
        
        for (size_t i = 2; i < signal.size(); ++i) {
            double current_diff = signal[i] - signal[i-1];
            if ((last_diff > 0 && current_diff <= 0) || (last_diff < 0 && current_diff >= 0)) {
                zero_crossings++;
            }
            last_diff = current_diff;
        }
        
        return static_cast<double>(zero_crossings) / signal.size();
    }
};
```

## Best Practices for Perception-Action Loop Design

### 1. Design for Robustness
- Handle sensor failures gracefully
- Implement fallback behaviors
- Use redundant sensors when critical

### 2. Ensure Real-time Performance
- Design efficient algorithms
- Consider computational complexity
- Implement proper timing mechanisms

### 3. Validate Stability
- Analyze system response
- Test under various conditions
- Monitor for oscillations

### 4. Plan for Adaptability
- Design modular architectures
- Use parameterized behaviors
- Implement learning capabilities

### 5. Test Thoroughly
- Simulate before deployment
- Test edge cases
- Validate in real environments

## Summary

Perception-action loops are the fundamental mechanism by which physical AI systems interact with the real world. Effective loop design requires understanding the trade-offs between different control strategies, from simple reactive systems to complex deliberative approaches.

Key concepts include:
- The basic structure of perception-action loops
- Reactive vs. deliberative control strategies
- PID control for precise actuation
- Stability analysis and performance evaluation
- Real-time implementation considerations
- Robustness to sensor noise and uncertainty

Creating effective perception-action loops requires careful consideration of timing, stability, and robustness. In the next chapter, we'll explore how to implement feedback control systems that ensure stable and predictable robot behavior.