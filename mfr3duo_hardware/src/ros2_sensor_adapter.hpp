#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>

#include "mfr3duo_hardware/robot_hardware.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mfr3duo_hardware {

/**
 * @brief Publishes RobotHardware sensor samples as ROS 2 messages.
 *
 * The adapter only converts C++ samples into ROS messages, maps timestamps and
 * publishes them outside the control loop. Acquisition, MuJoCo rendering and
 * LiDAR ray casting stay inside RobotHardware and its backend.
 */
class Ros2SensorAdapter {
public:
    explicit Ros2SensorAdapter(RobotHardware& robot);
    ~Ros2SensorAdapter();

    Ros2SensorAdapter(const Ros2SensorAdapter&) = delete;
    Ros2SensorAdapter& operator=(const Ros2SensorAdapter&) = delete;

    void start();
    void stop();

private:
    void run();
    bool publish_lidar(Lidar id, std::size_t index);
    bool publish_camera(Camera id, std::size_t index);
    rclcpp::Time sample_stamp(std::uint64_t timestamp_ns) const;

    RobotHardware& robot_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time clock_epoch_{0, 0, RCL_ROS_TIME};
    std::array<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr, 2> lidar_publishers_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr, 14> image_publishers_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr, 14> info_publishers_;
    std::array<std::uint64_t, 2> lidar_sequences_{};
    std::array<std::uint64_t, 14> camera_sequences_{};
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace mfr3duo_hardware
