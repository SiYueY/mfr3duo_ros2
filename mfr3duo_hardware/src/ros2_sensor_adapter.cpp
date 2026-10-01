#include "ros2_sensor_adapter.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <string>
#include <utility>

namespace mfr3duo_hardware {
namespace {

constexpr std::array<const char*, 2> kLidarNames{"lidar_front", "lidar_rear"};

// Indexed by the documented Camera enum order.
constexpr std::array<const char*, 14> kCameraNames{
    "front_color",      "front_depth",      "rear_color",        "rear_depth",
    "left_color",       "left_depth",       "right_color",       "right_depth",
    "left_wrist_color", "left_wrist_depth", "right_wrist_color", "right_wrist_depth",
    "head_zed_left",    "head_zed_right"};

}  // namespace

Ros2SensorAdapter::Ros2SensorAdapter(RobotHardware& robot)
: robot_(robot), node_(std::make_shared<rclcpp::Node>("mfr3duo_sensor_adapter")) {
    for (std::size_t index = 0; index < kLidarNames.size(); ++index) {
        lidar_publishers_[index] = node_->create_publisher<sensor_msgs::msg::LaserScan>(
            std::string("sensors/") + kLidarNames[index] + "/scan", 10);
    }
    for (std::size_t index = 0; index < kCameraNames.size(); ++index) {
        const auto prefix = std::string("sensors/") + kCameraNames[index];
        image_publishers_[index] = node_->create_publisher<sensor_msgs::msg::Image>(
            prefix + "/image_raw", rclcpp::SensorDataQoS());
        info_publishers_[index] = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            prefix + "/camera_info", rclcpp::SensorDataQoS());
    }
}

Ros2SensorAdapter::~Ros2SensorAdapter() { stop(); }

void Ros2SensorAdapter::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&Ros2SensorAdapter::run, this);
}

void Ros2SensorAdapter::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

rclcpp::Time Ros2SensorAdapter::sample_stamp(std::uint64_t timestamp_ns) const {
    return clock_epoch_ +
           rclcpp::Duration::from_nanoseconds(static_cast<std::int64_t>(timestamp_ns));
}

void Ros2SensorAdapter::run() {
    while (running_.load()) {
        // Re-anchor the robot clock on the coherent snapshot so stamps stay near
        // wall time when physics runs slower than real time.
        RobotState state;
        if (robot_.read_state(state)) {
            clock_epoch_ = node_->now() - rclcpp::Duration::from_nanoseconds(
                                              static_cast<std::int64_t>(state.timestamp_ns));
        }
        for (std::size_t index = 0; index < kLidarNames.size(); ++index) {
            publish_lidar(static_cast<Lidar>(index), index);
        }
        for (std::size_t index = 0; index < kCameraNames.size(); ++index) {
            publish_camera(static_cast<Camera>(index), index);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool Ros2SensorAdapter::publish_lidar(Lidar id, std::size_t index) {
    LaserScan scan;
    if (!robot_.read_state(id, scan) || scan.sequence == lidar_sequences_[index] ||
        scan.ranges.empty()) {
        return false;
    }
    lidar_sequences_[index] = scan.sequence;
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = sample_stamp(scan.timestamp_ns);
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
    return true;
}

bool Ros2SensorAdapter::publish_camera(Camera id, std::size_t index) {
    CameraFrame frame;
    if (!robot_.read_state(id, frame) || frame.sequence == camera_sequences_[index]) return false;
    if (frame.image.data.empty()) return false;
    camera_sequences_[index] = frame.sequence;
    const auto stamp = sample_stamp(frame.timestamp_ns);
    const auto& optical_frame =
        frame.optical_frame_id.empty() ? frame.frame_id : frame.optical_frame_id;

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = optical_frame;
    image.height = frame.image.height;
    image.width = frame.image.width;
    image.encoding = frame.image.encoding;
    image.is_bigendian = frame.image.is_bigendian;
    image.step = frame.image.step;
    image.data = std::move(frame.image.data);
    image_publishers_[index]->publish(std::move(image));

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = optical_frame;
    info.height = frame.image.height;
    info.width = frame.image.width;
    info.distortion_model = frame.camera_info.distortion_model.empty()
                                ? "plumb_bob"
                                : frame.camera_info.distortion_model;
    info.d = frame.camera_info.d;
    if (info.d.empty()) info.d.assign(5, 0.0);
    info.k = frame.camera_info.k;
    info.r = frame.camera_info.r;
    if (info.r == std::array<double, 9>{}) {
        info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    }
    info.p = frame.camera_info.p;
    info.binning_x = frame.camera_info.binning_x;
    info.binning_y = frame.camera_info.binning_y;
    info_publishers_[index]->publish(std::move(info));
    return true;
}

}  // namespace mfr3duo_hardware
