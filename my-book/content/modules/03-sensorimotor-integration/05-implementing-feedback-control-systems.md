# Chapter 3.5: Implementing Feedback Control Systems

## Learning Objectives

By the end of this chapter, students will be able to:
- Design and implement feedback control systems for robotic applications
- Apply classical control theory to physical AI systems
- Implement PID controllers with appropriate tuning methods
- Design state feedback controllers using modern control techniques
- Analyze the stability and performance of control systems
- Implement adaptive and robust control strategies
- Integrate control systems with sensor processing and actuation

## Introduction

Feedback control is the backbone of physical AI systems, enabling robots to achieve desired behaviors despite uncertainties, disturbances, and model inaccuracies. A feedback control system continuously measures the system's output, compares it to a desired reference, and adjusts the control input to minimize the error between the actual and desired outputs.

In robotics, feedback control systems are essential for tasks ranging from precise joint positioning to complex whole-body motion control. The tight coupling between sensing, processing, and actuation in physical AI systems creates a natural framework for feedback control implementation.

This chapter covers both classical and modern approaches to feedback control, providing practical implementations for robotic applications. We'll explore various control strategies, their implementation in ROS 2, and methods for analyzing and tuning control systems.

## Fundamentals of Feedback Control

### Basic Feedback Control Loop

The fundamental feedback control system consists of four components:

```
Reference → Controller → Plant → Output
     ↑                   ↓
     └------ Error <---- Measurement
```

- **Reference (r)**: The desired output of the system
- **Controller (C)**: Processes the error to generate control input
- **Plant (P)**: The physical system being controlled
- **Measurement (y)**: The actual output of the system
- **Error (e)**: The difference between reference and measurement (e = r - y)

### Control System Performance Metrics

For evaluating feedback control systems:

- **Steady-state error**: Error after the system has settled
- **Rise time**: Time to reach a specified percentage of the final value
- **Settling time**: Time to reach and stay within a specified tolerance of the final value
- **Overshoot**: Maximum deviation above the final value
- **Stability**: Whether the system remains bounded over time

## Proportional-Integral-Derivative (PID) Control

PID control is the most widely used feedback control strategy in robotics due to its simplicity and effectiveness.

### PID Controller Implementation

```cpp
#include <chrono>

class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double output_min, double output_max)
        : kp_(kp), ki_(ki), kd_(kd), output_min_(output_min), output_max_(output_max),
          prev_error_(0.0), integral_(0.0), last_time_(std::chrono::steady_clock::now()) {}

    double compute(double setpoint, double process_variable)
    {
        auto current_time = std::chrono::steady_clock::now();
        auto time_delta = std::chrono::duration<double>(current_time - last_time_).count();
        last_time_ = current_time;

        if (time_delta <= 0.0) {
            return 0.0;  // Avoid division by zero
        }

        double error = setpoint - process_variable;

        // Proportional term
        double p_term = kp_ * error;

        // Integral term
        integral_ += error * time_delta;
        
        // Anti-windup: limit integral term
        integral_ = std::max(std::min(integral_, integral_limit_), -integral_limit_);

        double i_term = ki_ * integral_;

        // Derivative term
        double derivative = (error - prev_error_) / time_delta;
        double d_term = kd_ * derivative;

        // Store error for next iteration
        prev_error_ = error;

        // Calculate total output
        double output = p_term + i_term + d_term;

        // Limit output to specified range
        output = std::max(std::min(output, output_max_), output_min_);

        return output;
    }

    void reset()
    {
        prev_error_ = 0.0;
        integral_ = 0.0;
        last_time_ = std::chrono::steady_clock::now();
    }

    void setTunings(double kp, double ki, double kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setOutputLimits(double min, double max)
    {
        output_min_ = min;
        output_max_ = max;
    }

private:
    double kp_, ki_, kd_;
    double output_min_, output_max_;
    double prev_error_, integral_;
    std::chrono::steady_clock::time_point last_time_;
    double integral_limit_ = 100.0;  // Limit for integral windup
};
```

### PID Tuning Methods

#### Ziegler-Nichols Method

```cpp
class PIDTuner
{
public:
    struct PIDParams
    {
        double kp, ki, kd;
    };

    // Ziegler-Nichols method for tuning PID controller
    static PIDParams tuneZieglerNichols(double ku, double tu)
    {
        PIDParams params;
        params.kp = 0.6 * ku;      // Ultimate gain * 0.6
        params.ki = 2.0 * params.kp / tu;  // 2*Kp/Tu
        params.kd = params.kp * tu / 8.0;  // Kp*Tu/8
        return params;
    }

    // Relay method for finding ultimate gain and period
    static std::pair<double, double> relayMethod(double initial_amplitude)
    {
        // This is a simplified representation
        // In practice, this would involve injecting a relay signal
        // and measuring the resulting oscillations
        return std::make_pair(1.0, 1.0);  // ku, tu (example values)
    }
};
```

#### Manual Tuning Approach

```cpp
class ManualPIDTuner
{
public:
    static PIDParams tuneManually(double kp, double ki, double kd)
    {
        PIDParams params;
        params.kp = kp;
        params.ki = ki;
        params.kd = kd;
        
        // Guidelines for manual tuning:
        // 1. Start with Ki = Kd = 0, tune Kp for desired response
        // 2. Add Ki to eliminate steady-state error
        // 3. Add Kd to reduce overshoot and improve stability
        
        return params;
    }
};
```

### PID Controller with Advanced Features

```cpp
class AdvancedPIDController
{
public:
    AdvancedPIDController(double kp, double ki, double kd, 
                         double output_min, double output_max)
        : pid_controller_(kp, ki, kd, output_min, output_max) {}

    double compute(double setpoint, double process_variable, 
                   bool enable_feedforward = false, double feedforward = 0.0)
    {
        double feedback = pid_controller_.compute(setpoint, process_variable);
        double output = feedback;
        
        if (enable_feedforward) {
            output += feedforward;
        }
        
        // Apply output constraints
        output = std::max(std::min(output, output_max_), output_min_);
        
        return output;
    }

    void setDerivativeOnMeasurement(bool enabled)
    {
        derivative_on_measurement_ = enabled;
    }

    void setOutputOn(bool enabled)
    {
        output_enabled_ = enabled;
    }

private:
    PIDController pid_controller_;
    bool derivative_on_measurement_ = false;
    bool output_enabled_ = true;
    double output_min_ = -1.0;
    double output_max_ = 1.0;
};
```

## State-Space Control Systems

Modern control theory uses state-space representation for more complex control systems.

### State-Space Representation

A linear time-invariant system can be represented as:
```
dx/dt = Ax + Bu
y = Cx + Du
```

Where:
- x: state vector
- u: input vector
- y: output vector
- A, B, C, D: system matrices

### Discrete-Time State-Space Controller

```cpp
#include <vector>
#include <Eigen/Dense>

class StateSpaceController
{
public:
    StateSpaceController(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, 
                        const Eigen::MatrixXd& C, const Eigen::MatrixXd& D)
        : A_(A), B_(B), C_(C), D_(D), 
          state_(A_.rows(), 1), state_(Eigen::MatrixXd::Zero(A_.rows(), 1)) {}

    Eigen::VectorXd compute(const Eigen::VectorXd& setpoint, 
                           const Eigen::VectorXd& measurement)
    {
        // Calculate error
        Eigen::VectorXd error = setpoint - measurement;
        
        // State feedback control: u = -Kx + K_r * r
        Eigen::VectorXd control_input = -K_ * state_ + K_r_ * setpoint;
        
        // Update state: x(k+1) = A*x(k) + B*u(k)
        state_ = A_ * state_ + B_ * control_input;
        
        return control_input;
    }

    // Method to compute state feedback gain using pole placement
    void placePoles(const std::vector<std::complex<double>>& desired_poles)
    {
        // This would typically use Ackermann's formula or other pole placement methods
        // For simplicity, we'll use a placeholder implementation
        K_ = Eigen::MatrixXd::Zero(B_.cols(), A_.rows());
        K_r_ = Eigen::MatrixXd::Identity(B_.cols(), B_.cols());
    }

    // Linear Quadratic Regulator (LQR) design
    void designLQR(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
    {
        // Solve the algebraic Riccati equation to find optimal gain
        // This is a simplified placeholder - full LQR requires solving ARE
        K_ = Eigen::MatrixXd::Zero(B_.cols(), A_.rows());
        K_r_ = Eigen::MatrixXd::Identity(B_.cols(), B_.cols());
    }

private:
    Eigen::MatrixXd A_, B_, C_, D_;
    Eigen::MatrixXd state_;
    Eigen::MatrixXd K_;   // State feedback gain
    Eigen::MatrixXd K_r_; // Feedforward gain
};
```

## Robot Joint Control Example

Let's implement a complete joint controller for a robotic manipulator:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class JointController : public rclcpp::Node
{
public:
    JointController() : Node("joint_controller")
    {
        // Initialize PID controllers for each joint
        initializeControllers();
        
        // Create subscriptions
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointController::jointStateCallback, this, std::placeholders::_1));
        
        // Create publishers
        joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_commands", 10);
        
        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100 Hz control frequency
            std::bind(&JointController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Joint controller initialized");
    }

private:
    void initializeControllers()
    {
        // Define joint names and initialize controllers
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        
        for (const auto& name : joint_names_) {
            // Initialize PID controller for each joint
            // Tuned for position control of robotic joints
            pid_controllers_[name] = std::make_unique<PIDController>(
                5.0,   // Kp: Proportional gain
                0.1,   // Ki: Integral gain  
                0.01,  // Kd: Derivative gain
                -10.0, // Output min
                10.0   // Output max
            );
        }
        
        // Initialize joint positions to zero
        current_positions_.resize(joint_names_.size(), 0.0);
        current_velocities_.resize(joint_names_.size(), 0.0);
        desired_positions_.resize(joint_names_.size(), 0.0);
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update current joint positions and velocities
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
            if (it != msg->name.end()) {
                size_t msg_idx = std::distance(msg->name.begin(), it);
                if (msg_idx < msg->position.size()) {
                    current_positions_[i] = msg->position[msg_idx];
                }
                if (msg_idx < msg->velocity.size()) {
                    current_velocities_[i] = msg->velocity[msg_idx];
                }
            }
        }
    }
    
    void controlLoop()
    {
        std::vector<double> commands(joint_names_.size());
        
        // Compute control commands for each joint
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            double current_pos = current_positions_[i];
            double desired_pos = desired_positions_[i];
            
            // Compute control command using PID
            commands[i] = pid_controllers_[joint_names_[i]]->compute(desired_pos, current_pos);
        }
        
        // Publish commands
        publishCommands(commands);
    }
    
    void publishCommands(const std::vector<double>& commands)
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = commands;
        joint_cmd_pub_->publish(msg);
    }
    
    // Trajectory following function
    void followTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        // Implement trajectory following logic
        // This would involve interpolating between trajectory points
        // and setting desired_positions_ accordingly
    }
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::vector<std::string> joint_names_;
    std::vector<double> current_positions_;
    std::vector<double> current_velocities_;
    std::vector<double> desired_positions_;
    std::unordered_map<std::string, std::unique_ptr<PIDController>> pid_controllers_;
};
```

## Mobile Robot Control Example

Now let's implement a control system for mobile robot navigation:

```cpp
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class MobileRobotController : public rclcpp::Node
{
public:
    MobileRobotController() : Node("mobile_robot_controller")
    {
        // Create subscriptions
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MobileRobotController::odometryCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10,
            std::bind(&MobileRobotController::goalCallback, this, std::placeholders::_1));
        
        // Create publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Initialize PID controllers
        linear_pid_ = std::make_unique<PIDController>(1.5, 0.1, 0.05, -1.0, 1.0);
        angular_pid_ = std::make_unique<PIDController>(2.0, 0.1, 0.1, -1.0, 1.0);
        
        // Create timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz control frequency
            std::bind(&MobileRobotController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Mobile robot controller initialized");
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        current_twist_ = msg->twist.twist;
    }
    
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = msg->pose;
        goal_received_ = true;
        reached_goal_ = false;
        RCLCPP_INFO(this->get_logger(), "New goal received");
    }
    
    void controlLoop()
    {
        if (!goal_received_ || reached_goal_) {
            // Stop robot if no goal or goal reached
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
            return;
        }
        
        // Calculate distance and angle to goal
        double dx = goal_pose_.position.x - current_pose_.position.x;
        double dy = goal_pose_.position.y - current_pose_.position.y;
        double distance_to_goal = std::sqrt(dx*dx + dy*dy);
        
        // Check if reached goal
        if (distance_to_goal < goal_tolerance_) {
            reached_goal_ = true;
            auto stop_cmd = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop_cmd);
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return;
        }
        
        // Calculate angle to goal
        double goal_angle = std::atan2(dy, dx);
        
        // Get current robot angle from quaternion
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        double current_yaw;
        tf2::Matrix3x3(q).getRPY(roll_, pitch_, current_yaw);
        
        // Calculate angle error (normalized to [-π, π])
        double angle_error = goal_angle - current_yaw;
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
        // Control strategy: first align, then go forward
        auto cmd = geometry_msgs::msg::Twist();
        
        if (std::abs(angle_error) > angular_tolerance_) {
            // First align with goal direction
            cmd.linear.x = 0.0;
            cmd.angular.z = angular_pid_->compute(0.0, angle_error);
        } else {
            // Move forward toward goal
            cmd.linear.x = linear_pid_->compute(0.0, -distance_to_goal);
            cmd.angular.z = angular_pid_->compute(0.0, angle_error);
        }
        
        // Limit velocities
        cmd.linear.x = std::max(std::min(cmd.linear.x, max_linear_speed_), -max_linear_speed_);
        cmd.angular.z = std::max(std::min(cmd.angular.z, max_angular_speed_), -max_angular_speed_);
        
        cmd_vel_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_twist_;
    geometry_msgs::msg::Pose goal_pose_;
    bool goal_received_ = false;
    bool reached_goal_ = false;
    
    std::unique_ptr<PIDController> linear_pid_;
    std::unique_ptr<PIDController> angular_pid_;
    
    // Parameters
    double goal_tolerance_ = 0.2;      // meters
    double angular_tolerance_ = 0.1;   // radians
    double max_linear_speed_ = 0.5;    // m/s
    double max_angular_speed_ = 1.0;   // rad/s
    double roll_, pitch_, yaw_;
};
```

## Adaptive Control Systems

Adaptive control adjusts controller parameters in real-time to accommodate changes in system dynamics or operating conditions.

### Model Reference Adaptive Control (MRAC)

```cpp
class ModelReferenceAdaptiveController
{
public:
    ModelReferenceAdaptiveController(double reference_natural_freq, double reference_damping_ratio)
        : ref_wn_(reference_natural_freq), ref_zeta_(reference_damping_ratio),
          theta1_(1.0), theta2_(0.0), gamma1_(0.1), gamma2_(0.1) {}
    
    double compute(double reference, double output, double output_derivative)
    {
        // Reference model: second-order system
        double ref_output = reference_model_.compute(reference);
        
        // Error between reference model and actual system
        double e = ref_output - output;
        double e_dot = reference_model_output_dot_ - output_derivative;
        
        // Adaptation law (gradient method)
        double phi1 = output;
        double phi2 = output_derivative;
        
        // Update parameters
        theta1_ += gamma1_ * e * phi1;
        theta2_ += gamma2_ * e * phi2;
        
        // Control law
        double control_signal = theta1_ * output + theta2_ * output_derivative;
        
        return control_signal;
    }

private:
    class ReferenceModel
    {
    public:
        ReferenceModel(double wn, double zeta) 
            : wn_(wn), zeta_(zeta), x1_(0.0), x2_(0.0) {}
        
        double compute(double input)
        {
            // Second-order reference model: wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
            double x1_dot = x2_;
            double x2_dot = -wn_*wn_ * x1_ - 2*zeta_*wn_ * x2_ + wn_*wn_ * input;
            
            x1_ += x1_dot * dt_;
            x2_ += x2_dot * dt_;
            
            return x1_;
        }
        
    private:
        double wn_, zeta_;
        double x1_, x2_;
        double dt_ = 0.01;  // Time step
    };
    
    ReferenceModel reference_model_;
    double ref_wn_, ref_zeta_;
    double theta1_, theta2_;  // Adaptive parameters
    double gamma1_, gamma2_;  // Adaptation gains
    double reference_model_output_dot_;
};
```

### Self-Tuning Regulator

```cpp
class SelfTuningRegulator
{
public:
    SelfTuningRegulator()
        : a1_(0.5), a2_(0.2), b0_(1.0), b1_(0.3), 
          theta_(3, 1), // Parameter vector [a1, a2, b0, b1]
          P_(4, 4),    // Covariance matrix
          lambda_(0.98) // Forgetting factor
    {
        // Initialize parameter vector
        theta_ << -0.5, 0.2, 1.0, 0.3;
        
        // Initialize covariance matrix
        P_ = Eigen::MatrixXd::Identity(4, 4) * 1000.0;
    }
    
    double compute(double reference, double output, double control_input)
    {
        // Recursive least squares parameter estimation
        Eigen::VectorXd phi(4);
        phi << -output, -prev_output_, control_input, prev_control_input_;
        
        // Kalman gain
        Eigen::VectorXd K = P_ * phi / (lambda_ + phi.transpose() * P_ * phi);
        
        // Parameter update
        double prediction_error = output - phi.transpose() * theta_;
        theta_ = theta_ + K * prediction_error;
        
        // Covariance update
        P_ = (P_ - K * phi.transpose() * P_) / lambda_;
        
        // Extract updated parameters
        a1_ = -theta_(0);
        a2_ = -theta_(1);
        b0_ = theta_(2);
        b1_ = theta_(3);
        
        // Update memory
        prev_output_ = output;
        prev_control_input_ = control_input;
        
        // Compute control using updated model
        return computeControl(reference, output);
    }

private:
    double computeControl(double reference, double output)
    {
        // Simple pole placement control based on estimated model
        double error = reference - output;
        double control = 0.5 * error;  // Simplified control law
        return control;
    }
    
    double a1_, a2_, b0_, b1_;
    Eigen::VectorXd theta_;  // Parameter vector
    Eigen::MatrixXd P_;      // Covariance matrix
    double lambda_;          // Forgetting factor
    double prev_output_ = 0.0;
    double prev_control_input_ = 0.0;
};
```

## Robust Control Systems

Robust control systems maintain performance despite uncertainties in the system model.

### H-infinity Control Concept

```cpp
class RobustController
{
public:
    RobustController(double nominal_kp, double nominal_ki, double nominal_kd)
        : nominal_kp_(nominal_kp), nominal_ki_(nominal_ki), nominal_kd_(nominal_kd),
          uncertainty_bound_(0.2) {}
    
    double compute(double setpoint, double process_variable, double uncertainty_estimate)
    {
        // Adjust gains based on uncertainty estimate
        double adaptive_kp = nominal_kp_ / (1.0 + uncertainty_estimate / uncertainty_bound_);
        double adaptive_ki = nominal_ki_ / (1.0 + uncertainty_estimate / uncertainty_bound_);
        double adaptive_kd = nominal_kd_ / (1.0 + uncertainty_estimate / uncertainty_bound_);
        
        // Create temporary PID with adjusted parameters
        PIDController temp_controller(adaptive_kp, adaptive_ki, adaptive_kd, -10.0, 10.0);
        
        return temp_controller.compute(setpoint, process_variable);
    }
    
    void updateUncertaintyEstimate(double new_estimate)
    {
        uncertainty_estimate_ = std::min(new_estimate, uncertainty_bound_);
    }

private:
    double nominal_kp_, nominal_ki_, nominal_kd_;
    double uncertainty_bound_;
    double uncertainty_estimate_ = 0.0;
};
```

## Control System Analysis

### Stability Analysis

```cpp
#include <complex>

class ControlSystemAnalyzer
{
public:
    struct StabilityReport
    {
        bool stable;
        std::vector<std::complex<double>> poles;
        double gain_margin;
        double phase_margin;
        std::string analysis;
    };
    
    static StabilityReport analyzeStability(const std::vector<double>& numerator, 
                                          const std::vector<double>& denominator)
    {
        StabilityReport report;
        
        // Find poles (roots of denominator)
        report.poles = findRoots(denominator);
        
        // Check stability (all poles in left half plane for continuous, 
        // inside unit circle for discrete)
        report.stable = true;
        for (const auto& pole : report.poles) {
            if (pole.real() >= 0) {  // Continuous system
                report.stable = false;
                break;
            }
        }
        
        // Calculate gain and phase margins using frequency response
        std::tie(report.gain_margin, report.phase_margin) = calculateMargins(numerator, denominator);
        
        report.analysis = report.stable ? "System is stable" : "System is unstable";
        
        return report;
    }

private:
    static std::vector<std::complex<double>> findRoots(const std::vector<double>& coeffs)
    {
        // Simplified root finding - in practice, use numerical methods
        // like Jenkins-Traub algorithm
        std::vector<std::complex<double>> roots;
        
        // For quadratic: ax^2 + bx + c = 0
        if (coeffs.size() == 3) {
            double a = coeffs[0], b = coeffs[1], c = coeffs[2];
            double discriminant = b*b - 4*a*c;
            
            if (discriminant >= 0) {
                roots.push_back(std::complex<double>(-b + std::sqrt(discriminant), 0.0) / (2*a));
                roots.push_back(std::complex<double>(-b - std::sqrt(discriminant), 0.0) / (2*a));
            } else {
                roots.push_back(std::complex<double>(-b, std::sqrt(-discriminant)) / (2*a));
                roots.push_back(std::complex<double>(-b, -std::sqrt(-discriminant)) / (2*a));
            }
        }
        
        return roots;
    }
    
    static std::pair<double, double> calculateMargins(const std::vector<double>& num, 
                                                    const std::vector<double>& den)
    {
        // Simplified calculation - in practice, evaluate frequency response
        // and find where phase crosses -180° and magnitude crosses 0dB
        return std::make_pair(6.0, 45.0);  // Example values
    }
};
```

### Performance Analysis

```cpp
class PerformanceAnalyzer
{
public:
    struct PerformanceMetrics
    {
        double rise_time;
        double settling_time;
        double overshoot;
        double steady_state_error;
        double integral_square_error;
        double integral_absolute_error;
    };
    
    static PerformanceMetrics analyzeResponse(const std::vector<double>& time, 
                                           const std::vector<double>& response,
                                           double final_value, double tolerance = 0.02)
    {
        PerformanceMetrics metrics = {};
        
        if (time.empty() || response.empty()) return metrics;
        
        // Find rise time (time to go from 10% to 90% of final value)
        double ten_percent = 0.1 * final_value;
        double ninety_percent = 0.9 * final_value;
        
        int rise_start = -1, rise_end = -1;
        for (size_t i = 0; i < response.size(); ++i) {
            if (rise_start == -1 && response[i] >= ten_percent) {
                rise_start = i;
            }
            if (rise_start != -1 && response[i] >= ninety_percent) {
                rise_end = i;
                break;
            }
        }
        
        if (rise_start != -1 && rise_end != -1) {
            metrics.rise_time = time[rise_end] - time[rise_start];
        }
        
        // Find settling time (time to stay within tolerance of final value)
        double upper_bound = final_value * (1.0 + tolerance);
        double lower_bound = final_value * (1.0 - tolerance);
        
        for (int i = response.size() - 1; i >= 0; --i) {
            if (response[i] < lower_bound || response[i] > upper_bound) {
                if (i + 1 < static_cast<int>(time.size())) {
                    metrics.settling_time = time[i + 1];
                }
                break;
            }
        }
        
        // Find overshoot
        double max_value = *std::max_element(response.begin(), response.end());
        if (final_value != 0) {
            metrics.overshoot = std::max(0.0, (max_value - final_value) / final_value * 100.0);
        }
        
        // Calculate steady-state error
        if (!response.empty()) {
            metrics.steady_state_error = std::abs(final_value - response.back());
        }
        
        // Calculate integral performance metrics
        double ise = 0.0, iae = 0.0;
        for (size_t i = 0; i < response.size(); ++i) {
            double error = final_value - response[i];
            ise += error * error;
            iae += std::abs(error);
        }
        
        metrics.integral_square_error = ise;
        metrics.integral_absolute_error = iae;
        
        return metrics;
    }
};
```

## Control System Design Guidelines

### 1. System Modeling
- Develop accurate mathematical models of the system
- Identify sources of uncertainty and disturbances
- Consider the physical constraints of actuators and sensors

### 2. Controller Selection
- Choose appropriate control strategy based on system characteristics
- Consider trade-offs between performance and complexity
- Plan for real-time implementation constraints

### 3. Tuning and Validation
- Use systematic tuning methods (Ziegler-Nichols, pole placement, LQR)
- Validate performance through simulation
- Test robustness to model uncertainties

### 4. Implementation Considerations
- Implement anti-windup for integral action
- Use derivative filtering to reduce noise sensitivity
- Consider sampling rate and computational delays

## Best Practices for Robotic Control Systems

### 1. Safety First
- Implement hard limits on control outputs
- Use safety interlocks and emergency stops
- Design fail-safe behaviors

### 2. Modularity
- Separate control algorithm from hardware interfaces
- Use standardized interfaces for easy replacement
- Implement clear separation of concerns

### 3. Monitoring and Diagnostics
- Log control signals for debugging
- Implement health monitoring
- Provide real-time performance metrics

### 4. Testing and Validation
- Test in simulation before hardware deployment
- Validate performance across operating range
- Test failure modes and recovery procedures

## Summary

Feedback control systems are essential for physical AI systems, enabling precise and stable robot behavior despite uncertainties and disturbances. This chapter covered classical PID control, modern state-space methods, adaptive control, and robust control techniques.

Key concepts include:
- PID controller design and tuning methods
- State-space representation and control
- Adaptive and robust control strategies
- Stability and performance analysis techniques
- Practical implementation guidelines for robotic systems

Effective feedback control requires careful system modeling, appropriate controller selection, systematic tuning, and thorough validation. In the next module, we'll explore how to implement these control systems in simulation environments before deploying them on real hardware.