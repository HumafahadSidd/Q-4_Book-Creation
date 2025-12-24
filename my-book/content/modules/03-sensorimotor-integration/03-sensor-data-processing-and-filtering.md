# Chapter 3.3: Sensor Data Processing and Filtering

## Learning Objectives

By the end of this chapter, students will be able to:
- Apply various filtering techniques to improve sensor data quality
- Implement moving average, Kalman, and particle filters for sensor data
- Design preprocessing pipelines for different sensor types
- Handle sensor noise, outliers, and missing data
- Evaluate the effectiveness of different filtering approaches
- Integrate filtering techniques into ROS 2 sensor processing nodes
- Optimize filter parameters for specific applications

## Introduction

Raw sensor data from physical AI systems is rarely perfect. Sensors are subject to various sources of noise, errors, and inconsistencies that can significantly impact the performance of robotic systems. Effective sensor data processing and filtering are essential for extracting meaningful information from noisy sensor readings and enabling reliable decision-making in physical AI systems.

The goal of sensor data processing is to transform raw sensor measurements into reliable, accurate, and useful information that can guide robotic behavior. This involves removing noise, correcting for sensor biases, handling missing or erroneous data, and fusing information from multiple sensors when available.

In this chapter, we'll explore various techniques for processing and filtering sensor data, from simple methods like moving averages to sophisticated approaches like Kalman and particle filters. We'll also discuss how to implement these techniques in ROS 2 systems and optimize them for real-time performance.

## Sources of Sensor Data Issues

### Noise

Noise is random variation in sensor measurements that can come from various sources:

**Electronic Noise:**
- Thermal noise in electronic components
- Quantization noise from digital conversion
- Power supply fluctuations

**Environmental Noise:**
- Electromagnetic interference
- Vibrations affecting sensor readings
- Temperature variations

**Mechanical Noise:**
- Vibrations from robot motion
- Flexibility in mechanical structures
- Gear backlash and friction

### Outliers

Outliers are sensor readings that deviate significantly from expected values:

**Spike Noise:**
- Sudden, brief changes in sensor readings
- May result from electrical interference
- Often appear as extreme values

**Systematic Errors:**
- Consistent bias in sensor readings
- Drift over time or temperature
- Calibration errors

### Missing Data

Sensors may occasionally fail to provide readings:

**Temporal Gaps:**
- Sensor processing delays
- Communication failures
- Hardware malfunctions

**Environmental Occlusions:**
- Physical obstacles blocking sensors
- Adverse weather conditions
- Sensor limitations in specific environments

## Basic Filtering Techniques

### Moving Average Filter

The moving average filter is one of the simplest and most commonly used filtering techniques. It smooths data by averaging recent measurements.

#### Simple Moving Average

```cpp
#include <vector>
#include <numeric>

class MovingAverageFilter
{
public:
    MovingAverageFilter(int window_size) : window_size_(window_size) {}
    
    double update(double new_value)
    {
        if (values_.size() >= window_size_) {
            values_.pop_front();
        }
        values_.push_back(new_value);
        
        double sum = std::accumulate(values_.begin(), values_.end(), 0.0);
        return sum / values_.size();
    }

private:
    int window_size_;
    std::deque<double> values_;
};
```

#### Exponential Moving Average

The exponential moving average gives more weight to recent measurements:

```cpp
class ExponentialMovingAverage
{
public:
    ExponentialMovingAverage(double alpha) : alpha_(alpha), initialized_(false) {}
    
    double update(double new_value)
    {
        if (!initialized_) {
            filtered_value_ = new_value;
            initialized_ = true;
        } else {
            filtered_value_ = alpha_ * new_value + (1.0 - alpha_) * filtered_value_;
        }
        return filtered_value_;
    }

private:
    double alpha_;  // Smoothing factor (0 < alpha < 1)
    double filtered_value_;
    bool initialized_;
};
```

### Median Filter

The median filter is effective for removing outliers while preserving sharp edges in the signal:

```cpp
#include <algorithm>
#include <vector>

class MedianFilter
{
public:
    MedianFilter(int window_size) : window_size_(window_size) {}
    
    double update(double new_value)
    {
        if (values_.size() >= window_size_) {
            values_.pop_front();
        }
        values_.push_back(new_value);
        
        std::vector<double> sorted_values(values_.begin(), values_.end());
        std::sort(sorted_values.begin(), sorted_values.end());
        
        size_t size = sorted_values.size();
        if (size % 2 == 0) {
            return (sorted_values[size/2 - 1] + sorted_values[size/2]) / 2.0;
        } else {
            return sorted_values[size/2];
        }
    }

private:
    int window_size_;
    std::deque<double> values_;
};
```

## Advanced Filtering Techniques

### Kalman Filter

The Kalman filter is an optimal recursive filter for linear systems with Gaussian noise. It estimates the state of a system by combining predictions and measurements.

#### Simple 1D Kalman Filter

```cpp
class KalmanFilter1D
{
public:
    KalmanFilter1D(double process_noise, double measurement_noise, double initial_estimate, double initial_error)
        : process_noise_(process_noise), measurement_noise_(measurement_noise),
          estimate_(initial_estimate), error_(initial_error) {}
    
    double update(double measurement)
    {
        // Prediction step
        // In 1D, prediction doesn't change estimate (assuming constant model)
        double prediction_error = error_ + process_noise_;
        
        // Update step
        double kalman_gain = prediction_error / (prediction_error + measurement_noise_);
        estimate_ = estimate_ + kalman_gain * (measurement - estimate_);
        error_ = (1.0 - kalman_gain) * prediction_error;
        
        return estimate_;
    }

private:
    double process_noise_;
    double measurement_noise_;
    double estimate_;
    double error_;
};
```

#### Extended Kalman Filter (EKF)

For non-linear systems, the Extended Kalman Filter linearizes the system around the current estimate:

```cpp
#include <Eigen/Dense>

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(const Eigen::MatrixXd& process_noise, 
                        const Eigen::MatrixXd& measurement_noise,
                        const Eigen::VectorXd& initial_state,
                        const Eigen::MatrixXd& initial_covariance)
        : Q_(process_noise), R_(measurement_noise), 
          x_(initial_state), P_(initial_covariance) {}
    
    void predict(const Eigen::VectorXd& control)
    {
        // Non-linear state transition function (example: constant velocity model)
        x_(0) += x_(1) * dt_;  // position = position + velocity * dt
        // x_(1) remains unchanged in constant velocity model
        
        // Jacobian of state transition function
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
        F(0, 1) = dt_;  // dx/dv = dt
        
        P_ = F * P_ * F.transpose() + Q_;
    }
    
    void update(const Eigen::VectorXd& measurement)
    {
        // Measurement function (example: direct position measurement)
        Eigen::VectorXd h = Eigen::VectorXd::Zero(measurement_dim_);
        h(0) = x_(0);  // measurement is position
        
        // Jacobian of measurement function
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(measurement_dim_, state_dim_);
        H(0, 0) = 1.0;  // dh/dx = 1
        
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        
        x_ = x_ + K * (measurement - h);
        P_ = (Eigen::MatrixXd::Identity(state_dim_, state_dim_) - K * H) * P_;
    }
    
    Eigen::VectorXd getState() const { return x_; }

private:
    static const int state_dim_ = 2;      // position, velocity
    static const int measurement_dim_ = 1; // position measurement
    static const double dt_ = 0.1;        // time step
    
    Eigen::MatrixXd Q_;  // Process noise covariance
    Eigen::MatrixXd R_;  // Measurement noise covariance
    Eigen::VectorXd x_;  // State vector
    Eigen::MatrixXd P_;  // Error covariance matrix
};
```

### Particle Filter

Particle filters are useful for non-linear, non-Gaussian systems. They represent probability distributions with samples (particles).

```cpp
#include <vector>
#include <random>
#include <algorithm>

struct Particle
{
    double state;
    double weight;
    
    Particle(double s, double w) : state(s), weight(w) {}
};

class ParticleFilter
{
public:
    ParticleFilter(int num_particles, double state_min, double state_max)
        : num_particles_(num_particles), state_min_(state_min), state_max_(state_max)
    {
        particles_.reserve(num_particles_);
        initialize_particles();
        uniform_dist_ = std::uniform_real_distribution<double>(state_min_, state_max_);
        normal_dist_ = std::normal_distribution<double>(0.0, 1.0);
    }
    
    void predict(double control, double process_noise_std)
    {
        for (auto& particle : particles_) {
            // Add process noise
            particle.state += control + process_noise_std * normal_dist_(gen_);
        }
    }
    
    void update(const std::vector<double>& measurements)
    {
        // Calculate weights based on measurement likelihood
        for (auto& particle : particles_) {
            double likelihood = 1.0;
            for (const auto& meas : measurements) {
                // Simple Gaussian likelihood model
                double diff = meas - particle.state;
                likelihood *= std::exp(-0.5 * diff * diff / measurement_variance_);
            }
            particle.weight = likelihood;
        }
        
        // Normalize weights
        double total_weight = 0.0;
        for (const auto& particle : particles_) {
            total_weight += particle.weight;
        }
        
        if (total_weight > 0.0) {
            for (auto& particle : particles_) {
                particle.weight /= total_weight;
            }
        }
        
        // Resample particles
        resample();
    }
    
    double estimate() const
    {
        double estimate = 0.0;
        for (const auto& particle : particles_) {
            estimate += particle.state * particle.weight;
        }
        return estimate;
    }

private:
    void initialize_particles()
    {
        particles_.clear();
        for (int i = 0; i < num_particles_; ++i) {
            double state = uniform_dist_(gen_);
            particles_.emplace_back(state, 1.0 / num_particles_);
        }
    }
    
    void resample()
    {
        std::vector<Particle> new_particles;
        new_particles.reserve(num_particles_);
        
        // Calculate cumulative weights
        std::vector<double> cumulative_weights;
        cumulative_weights.reserve(num_particles_);
        double sum = 0.0;
        for (const auto& particle : particles_) {
            sum += particle.weight;
            cumulative_weights.push_back(sum);
        }
        
        // Generate new particles
        std::uniform_real_distribution<double> uniform_dist(0.0, sum);
        for (int i = 0; i < num_particles_; ++i) {
            double r = uniform_dist(gen_);
            
            // Find particle corresponding to random value
            auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), r);
            int index = std::distance(cumulative_weights.begin(), it);
            
            if (index >= num_particles_) index = num_particles_ - 1;
            
            new_particles.push_back(particles_[index]);
            new_particles.back().weight = 1.0 / num_particles_;
        }
        
        particles_ = std::move(new_particles);
    }
    
    int num_particles_;
    double state_min_, state_max_;
    double measurement_variance_ = 1.0;
    
    std::vector<Particle> particles_;
    std::default_random_engine gen_{std::random_device{}()};
    std::uniform_real_distribution<double> uniform_dist_;
    std::normal_distribution<double> normal_dist_;
};
```

## Sensor-Specific Processing

### Processing Camera Data

Camera data processing often involves image filtering, feature extraction, and noise reduction:

```cpp
#include <opencv2/opencv.hpp>

class CameraProcessor
{
public:
    cv::Mat processImage(const cv::Mat& input)
    {
        cv::Mat processed;
        
        // Denoise image
        cv::fastNlMeansDenoisingColored(input, processed);
        
        // Apply Gaussian blur to reduce noise
        cv::GaussianBlur(processed, processed, cv::Size(5, 5), 0);
        
        // Convert to grayscale if needed
        if (processed.channels() > 1) {
            cv::cvtColor(processed, processed, cv::COLOR_BGR2GRAY);
        }
        
        return processed;
    }
    
    std::vector<cv::Point2f> extractFeatures(const cv::Mat& image)
    {
        std::vector<cv::Point2f> corners;
        
        // Detect corners using Shi-Tomasi method
        cv::goodFeaturesToTrack(image, corners, 100, 0.01, 10);
        
        return corners;
    }
};
```

### Processing LIDAR Data

LIDAR data often requires outlier removal and clustering:

```cpp
#include <vector>
#include <algorithm>

class LIDARProcessor
{
public:
    struct Point
    {
        double x, y;
        double range;
    };
    
    std::vector<Point> processScan(const std::vector<double>& ranges, double angle_min, double angle_increment)
    {
        std::vector<Point> points;
        points.reserve(ranges.size());
        
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] > min_range_ && ranges[i] < max_range_) {
                double angle = angle_min + i * angle_increment;
                Point p;
                p.x = ranges[i] * cos(angle);
                p.y = ranges[i] * sin(angle);
                p.range = ranges[i];
                points.push_back(p);
            }
        }
        
        // Remove outliers using statistical method
        removeOutliers(points);
        
        return points;
    }

private:
    void removeOutliers(std::vector<Point>& points)
    {
        if (points.size() < 3) return;
        
        // Calculate distances to nearest neighbors
        for (auto& point : points) {
            double min_dist = std::numeric_limits<double>::max();
            for (const auto& other : points) {
                if (&point != &other) {
                    double dist = sqrt(pow(point.x - other.x, 2) + pow(point.y - other.y, 2));
                    min_dist = std::min(min_dist, dist);
                }
            }
            point.range = min_dist; // Temporarily store neighbor distance
        }
        
        // Calculate statistics
        std::vector<double> distances;
        for (const auto& point : points) {
            distances.push_back(point.range);
        }
        std::sort(distances.begin(), distances.end());
        
        double median = distances[distances.size() / 2];
        double threshold = median * outlier_threshold_;
        
        // Remove points with large neighbor distances
        points.erase(
            std::remove_if(points.begin(), points.end(),
                          [threshold](const Point& p) { return p.range > threshold; }),
            points.end());
    }
    
    double min_range_ = 0.1;
    double max_range_ = 30.0;
    double outlier_threshold_ = 2.0;
};
```

### Processing IMU Data

IMU data processing often involves sensor fusion and drift correction:

```cpp
#include <vector>
#include <cmath>

class IMUProcessor
{
public:
    struct IMUData
    {
        double ax, ay, az;  // Acceleration
        double gx, gy, gz;  // Angular velocity
        double mx, my, mz;  // Magnetic field (if available)
    };
    
    struct Orientation
    {
        double roll, pitch, yaw;
    };
    
    Orientation processIMU(const IMUData& raw_data, double dt)
    {
        // Integrate angular velocity to get orientation change
        double delta_roll = raw_data.gx * dt;
        double delta_pitch = raw_data.gy * dt;
        double delta_yaw = raw_data.gz * dt;
        
        // Update orientation
        orientation_.roll += delta_roll;
        orientation_.pitch += delta_pitch;
        orientation_.yaw += delta_yaw;
        
        // Apply complementary filter to reduce drift
        applyComplementaryFilter(raw_data, dt);
        
        return orientation_;
    }

private:
    void applyComplementaryFilter(const IMUData& raw_data, double dt)
    {
        // Calculate pitch and roll from accelerometer
        double acc_pitch = atan2(raw_data.ax, sqrt(raw_data.ay*raw_data.ay + raw_data.az*raw_data.az));
        double acc_roll = atan2(-raw_data.ay, sqrt(raw_data.ax*raw_data.ax + raw_data.az*raw_data.az));
        
        // Apply complementary filter
        orientation_.pitch = alpha_ * (orientation_.pitch + raw_data.gy * dt) + (1.0 - alpha_) * acc_pitch;
        orientation_.roll = alpha_ * (orientation_.roll + raw_data.gx * dt) + (1.0 - alpha_) * acc_roll;
    }
    
    Orientation orientation_ = {0.0, 0.0, 0.0};
    double alpha_ = 0.98;  // Filter parameter (0-1)
};
```

## Outlier Detection and Handling

### Statistical Methods

```cpp
#include <vector>
#include <algorithm>
#include <cmath>

class OutlierDetector
{
public:
    std::vector<bool> detectOutliers(const std::vector<double>& data, double threshold = 2.0)
    {
        if (data.empty()) return std::vector<bool>();
        
        // Calculate mean and standard deviation
        double sum = 0.0;
        for (double value : data) {
            sum += value;
        }
        double mean = sum / data.size();
        
        double sum_sq_diff = 0.0;
        for (double value : data) {
            sum_sq_diff += (value - mean) * (value - mean);
        }
        double std_dev = sqrt(sum_sq_diff / data.size());
        
        // Identify outliers
        std::vector<bool> is_outlier(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            if (std_dev > 0.0 && std::abs(data[i] - mean) > threshold * std_dev) {
                is_outlier[i] = true;
            } else {
                is_outlier[i] = false;
            }
        }
        
        return is_outlier;
    }
    
    // Replace outliers with interpolated values
    std::vector<double> cleanData(const std::vector<double>& data, double threshold = 2.0)
    {
        std::vector<bool> outliers = detectOutliers(data, threshold);
        std::vector<double> cleaned = data;
        
        for (size_t i = 0; i < data.size(); ++i) {
            if (outliers[i]) {
                // Replace outlier with interpolated value from neighbors
                double replacement = interpolate(data, i);
                cleaned[i] = replacement;
            }
        }
        
        return cleaned;
    }

private:
    double interpolate(const std::vector<double>& data, size_t index)
    {
        size_t n = data.size();
        
        if (n <= 1) return 0.0;
        
        // Find nearest valid neighbors
        double left_val = data[index];
        double right_val = data[index];
        
        // Look left
        for (int i = index - 1; i >= 0; --i) {
            if (std::abs(data[i] - data[index]) <= 2.0 * estimateStdDev(data)) {
                left_val = data[i];
                break;
            }
        }
        
        // Look right
        for (size_t i = index + 1; i < n; ++i) {
            if (std::abs(data[i] - data[index]) <= 2.0 * estimateStdDev(data)) {
                right_val = data[i];
                break;
            }
        }
        
        return (left_val + right_val) / 2.0;
    }
    
    double estimateStdDev(const std::vector<double>& data)
    {
        if (data.empty()) return 0.0;
        
        double sum = 0.0;
        for (double value : data) {
            sum += value;
        }
        double mean = sum / data.size();
        
        double sum_sq_diff = 0.0;
        for (double value : data) {
            sum_sq_diff += (value - mean) * (value - mean);
        }
        
        return sqrt(sum_sq_diff / data.size());
    }
};
```

## ROS 2 Integration

### Filter Node Implementation

```cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

class SensorFilterNode : public rclcpp::Node
{
public:
    SensorFilterNode() : Node("sensor_filter_node")
    {
        // Create subscriptions
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw", 10,
            std::bind(&SensorFilterNode::scanCallback, this, std::placeholders::_1));
        
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_raw", 10,
            std::bind(&SensorFilterNode::imuCallback, this, std::placeholders::_1));
        
        // Create publishers
        filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
        filtered_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_filtered", 10);
        filtered_distance_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/distance_filtered", 10);
        
        // Initialize filters
        distance_filter_ = std::make_unique<KalmanFilter1D>(0.1, 0.1, 0.0, 1.0);
        
        RCLCPP_INFO(this->get_logger(), "Sensor filter node initialized");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Apply median filter to LIDAR data
        auto filtered_msg = *msg;  // Copy original message
        
        // Process each range reading
        for (auto& range : filtered_msg.ranges) {
            if (range > msg->range_min && range < msg->range_max) {
                // Apply some filtering (simplified example)
                range = applyRangeFilter(range);
            }
        }
        
        filtered_scan_publisher_->publish(filtered_msg);
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Apply filtering to IMU data (simplified example)
        auto filtered_msg = *msg;  // Copy original message
        
        // Example: Apply simple smoothing to angular velocity
        static double last_gx = msg->angular_velocity.x;
        static double last_gy = msg->angular_velocity.y;
        static double last_gz = msg->angular_velocity.z;
        
        double alpha = 0.2;  // Smoothing factor
        filtered_msg.angular_velocity.x = alpha * msg->angular_velocity.x + (1.0 - alpha) * last_gx;
        filtered_msg.angular_velocity.y = alpha * msg->angular_velocity.y + (1.0 - alpha) * last_gy;
        filtered_msg.angular_velocity.z = alpha * msg->angular_velocity.z + (1.0 - alpha) * last_gz;
        
        last_gx = filtered_msg.angular_velocity.x;
        last_gy = filtered_msg.angular_velocity.y;
        last_gz = filtered_msg.angular_velocity.z;
        
        filtered_imu_publisher_->publish(filtered_msg);
    }
    
    double applyRangeFilter(double range)
    {
        // Simple example of applying a Kalman filter to distance measurements
        if (distance_filter_) {
            return distance_filter_->update(range);
        }
        return range;
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr filtered_imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr filtered_distance_publisher_;
    
    std::unique_ptr<KalmanFilter1D> distance_filter_;
};
```

## Performance Considerations

### Real-time Processing

For real-time applications, consider the computational complexity of filtering algorithms:

```cpp
// Efficient implementation of moving average using circular buffer
class EfficientMovingAverage
{
public:
    EfficientMovingAverage(int window_size) 
        : window_size_(window_size), buffer_(window_size_), sum_(0.0), index_(0), count_(0) {}
    
    double update(double new_value)
    {
        if (count_ < window_size_) {
            // Buffer not full yet
            buffer_[index_] = new_value;
            sum_ += new_value;
            count_++;
        } else {
            // Buffer full, replace oldest value
            sum_ -= buffer_[index_];  // Remove old value from sum
            buffer_[index_] = new_value;
            sum_ += new_value;        // Add new value to sum
        }
        
        index_ = (index_ + 1) % window_size_;
        
        return sum_ / count_;
    }

private:
    int window_size_;
    std::vector<double> buffer_;
    double sum_;
    int index_;
    int count_;
};
```

### Adaptive Filtering

Adjust filter parameters based on changing conditions:

```cpp
class AdaptiveFilter
{
public:
    AdaptiveFilter(double initial_alpha = 0.1) : alpha_(initial_alpha), error_(0.0) {}
    
    double update(double measurement, double prediction)
    {
        // Calculate prediction error
        double new_error = std::abs(measurement - prediction);
        
        // Update error estimate
        error_ = 0.9 * error_ + 0.1 * new_error;  // Exponential average
        
        // Adjust alpha based on error
        if (error_ > threshold_) {
            alpha_ = std::min(alpha_ + 0.01, 0.9);  // Increase alpha if error is high
        } else {
            alpha_ = std::max(alpha_ - 0.005, 0.01); // Decrease alpha if error is low
        }
        
        // Apply exponential moving average
        if (!initialized_) {
            filtered_value_ = measurement;
            initialized_ = true;
        } else {
            filtered_value_ = alpha_ * measurement + (1.0 - alpha_) * filtered_value_;
        }
        
        return filtered_value_;
    }

private:
    double alpha_;
    double error_;
    double filtered_value_;
    bool initialized_ = false;
    double threshold_ = 0.5;
};
```

## Evaluation Metrics

### Filter Performance Evaluation

```cpp
#include <vector>
#include <cmath>

class FilterEvaluator
{
public:
    struct PerformanceMetrics
    {
        double rmse;          // Root Mean Square Error
        double mean_error;    // Mean Error
        double max_error;     // Maximum Error
        double std_error;     // Standard Deviation of Error
    };
    
    PerformanceMetrics evaluate(const std::vector<double>& true_values, 
                               const std::vector<double>& estimated_values)
    {
        if (true_values.size() != estimated_values.size()) {
            throw std::invalid_argument("Vector sizes must match");
        }
        
        std::vector<double> errors;
        double sum_sq_error = 0.0;
        double sum_error = 0.0;
        
        for (size_t i = 0; i < true_values.size(); ++i) {
            double error = true_values[i] - estimated_values[i];
            errors.push_back(error);
            sum_sq_error += error * error;
            sum_error += error;
        }
        
        PerformanceMetrics metrics;
        size_t n = true_values.size();
        
        metrics.rmse = std::sqrt(sum_sq_error / n);
        metrics.mean_error = sum_error / n;
        
        // Find max error
        auto max_it = std::max_element(errors.begin(), errors.end(),
                                      [](double a, double b) { return std::abs(a) < std::abs(b); });
        metrics.max_error = std::abs(*max_it);
        
        // Calculate standard deviation of errors
        double mean_error = metrics.mean_error;
        double sum_sq_diff = 0.0;
        for (double error : errors) {
            sum_sq_diff += (error - mean_error) * (error - mean_error);
        }
        metrics.std_error = std::sqrt(sum_sq_diff / n);
        
        return metrics;
    }
};
```

## Best Practices for Sensor Filtering

### 1. Understand Your Data
- Analyze the characteristics of your sensor data
- Identify the types of noise and errors present
- Consider the frequency content of your signals

### 2. Choose Appropriate Filters
- Simple moving averages for smoothing
- Kalman filters for tracking with noise
- Particle filters for non-linear systems
- Median filters for outlier removal

### 3. Consider Computational Complexity
- Real-time systems require efficient algorithms
- Pre-compute constants when possible
- Use efficient data structures (circular buffers)

### 4. Validate Filter Performance
- Test with known signals when possible
- Use simulation to validate before deployment
- Monitor performance metrics during operation

### 5. Tune Parameters Carefully
- Balance noise reduction with responsiveness
- Consider the trade-offs in your specific application
- Validate performance across different operating conditions

## Summary

Sensor data processing and filtering are essential components of physical AI systems, transforming noisy raw measurements into reliable information for decision-making. The choice of filtering technique depends on the specific requirements of the application, including the type of sensor, noise characteristics, and computational constraints.

Key takeaways:
- Different sensors require different processing approaches
- Simple filters like moving averages can be effective for basic smoothing
- Advanced filters like Kalman and particle filters handle complex noise models
- Real-time performance considerations are crucial for physical AI applications
- Proper validation and tuning of filters is essential for reliable operation

In the next chapter, we'll explore how to create perception-action loops that integrate these filtering techniques into complete sensorimotor systems.