#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include "mfr3duo_mujoco/simulation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mfr3duo_hardware {

/** Publishes variable-size sensor samples outside the control loop. */
class MujocoSensorBridge {
public:
  explicit MujocoSensorBridge(mfr3duo_mujoco::Simulation& simulation);
  ~MujocoSensorBridge();
  MujocoSensorBridge(const MujocoSensorBridge&) = delete;
  MujocoSensorBridge& operator=(const MujocoSensorBridge&) = delete;
  void start();
  void stop();

private:
  void run();
  void publish_lidar(std::size_t index);
  void publish_camera(std::size_t index);
  rclcpp::Time sample_stamp(std::uint64_t simulation_nanoseconds) const;
  mfr3duo_mujoco::Simulation& simulation_;
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
