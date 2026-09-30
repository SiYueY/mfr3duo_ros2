#include "mujoco_sensor_bridge.hpp"

#include <chrono>
#include <string>

namespace mfr3duo_hardware {
namespace {

constexpr std::array<const char*, 2> kLidarNames{"lidar_front", "lidar_rear"};
constexpr std::array<const char*, 14> kCameraNames{
    "front_color", "front_depth", "rear_color", "rear_depth", "right_color",
    "right_depth", "left_color", "left_depth", "left_wrist_color",
    "left_wrist_depth", "right_wrist_color", "right_wrist_depth",
    "head_zed_left", "head_zed_right"};

}  // namespace

MujocoSensorBridge::MujocoSensorBridge(mfr3duo_mujoco::Simulation& simulation)
    : simulation_(simulation), node_(std::make_shared<rclcpp::Node>("mfr3duo_mujoco_sensors")) {
  clock_epoch_ = node_->now() - rclcpp::Duration::from_seconds(simulation_.time());
  for (std::size_t index = 0; index < kLidarNames.size(); ++index)
    lidar_publishers_[index] = node_->create_publisher<sensor_msgs::msg::LaserScan>(
        std::string("sensors/") + kLidarNames[index] + "/scan", 10);
  for (std::size_t index = 0; index < kCameraNames.size(); ++index) {
    const auto prefix = std::string("sensors/") + kCameraNames[index];
    image_publishers_[index] = node_->create_publisher<sensor_msgs::msg::Image>(
        prefix + "/image_raw", rclcpp::SensorDataQoS());
    info_publishers_[index] = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        prefix + "/camera_info", rclcpp::SensorDataQoS());
  }
}

rclcpp::Time MujocoSensorBridge::sample_stamp(std::uint64_t simulation_nanoseconds) const {
  return clock_epoch_ + rclcpp::Duration::from_nanoseconds(
                            static_cast<std::int64_t>(simulation_nanoseconds));
}

MujocoSensorBridge::~MujocoSensorBridge() { stop(); }

void MujocoSensorBridge::start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread(&MujocoSensorBridge::run, this);
}

void MujocoSensorBridge::stop() {
  running_.store(false);
  if (thread_.joinable()) thread_.join();
}

void MujocoSensorBridge::run() {
  while (running_.load()) {
    // Keep stamps near wall time when physics runs slower than real time.
    clock_epoch_ = node_->now() - rclcpp::Duration::from_seconds(simulation_.time());
    for (std::size_t index = 0; index < kLidarNames.size(); ++index)
      publish_lidar(index);
    for (std::size_t index = 0; index < kCameraNames.size(); ++index)
      publish_camera(index);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void MujocoSensorBridge::publish_lidar(std::size_t index) {
  mfr3duo_mujoco::LaserScan scan;
  if (!simulation_.read_state(static_cast<mfr3duo_mujoco::Lidar>(index), scan) ||
      scan.sequence == lidar_sequences_[index] || scan.ranges.empty()) return;
  lidar_sequences_[index] = scan.sequence;
  sensor_msgs::msg::LaserScan message;
  message.header.stamp = sample_stamp(scan.timestamp);
  message.header.frame_id = scan.frame_id;
  message.angle_min = scan.angle_min;
  message.angle_max = scan.angle_max;
  message.angle_increment = scan.angle_increment;
  message.time_increment = scan.time_increment;
  message.scan_time = scan.scan_time;
  message.range_min = scan.range_min;
  message.range_max = scan.range_max;
  message.ranges = std::move(scan.ranges);
  message.intensities = std::move(scan.intensities);
  lidar_publishers_[index]->publish(std::move(message));
}

void MujocoSensorBridge::publish_camera(std::size_t index) {
  mfr3duo_mujoco::CameraFrame frame;
  if (!simulation_.read_state(static_cast<mfr3duo_mujoco::Camera>(index), frame) ||
      frame.sequence == camera_sequences_[index]) return;
  auto& source = frame.image.data.empty() ? frame.depth_image : frame.image;
  if (source.data.empty()) return;
  camera_sequences_[index] = frame.sequence;
  const auto stamp = sample_stamp(source.timestamp);
  const auto& optical_frame = frame.optical_frame_id.empty()
                                  ? source.frame_id : frame.optical_frame_id;
  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = optical_frame;
  image.height = source.height;
  image.width = source.width;
  image.encoding = source.encoding;
  image.is_bigendian = source.is_bigendian;
  image.step = source.step;
  image.data = std::move(source.data);
  image_publishers_[index]->publish(std::move(image));

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = optical_frame;
  info.height = source.height;
  info.width = source.width;
  info.distortion_model = frame.camera_info.distortion_model.empty()
                              ? "plumb_bob" : frame.camera_info.distortion_model;
  info.d = frame.camera_info.d;
  if (info.d.empty()) info.d.assign(5, 0.0);
  info.k = frame.camera_info.k;
  info.r = frame.camera_info.r;
  if (info.r == std::array<double, 9>{}) info.r = {1.0, 0.0, 0.0, 0.0, 1.0,
                                                 0.0, 0.0, 0.0, 1.0};
  info.p = frame.camera_info.p;
  info.binning_x = frame.camera_info.binning_x;
  info.binning_y = frame.camera_info.binning_y;
  info_publishers_[index]->publish(std::move(info));
}

}  // namespace mfr3duo_hardware
