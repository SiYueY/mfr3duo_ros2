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

Ros2SensorAdapter::Ros2SensorAdapter(
    RobotHardware& robot, const std::vector<SimulationObjectMapping>& objects)
: robot_(robot), node_(std::make_shared<rclcpp::Node>("mfr3duo_sensor_adapter")) {
    const auto objects_root = node_->declare_parameter<std::string>(
        "perception.objects_topic_prefix", "/perception/objects");
    const auto tools_root =
        node_->declare_parameter<std::string>("perception.tools_topic_prefix", "/perception/tools");
    std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr, 2> tools;
    for (const auto& object : objects) {
        auto object_publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            objects_root + "/" + object.object_id + "/pose", 10);
        for (std::size_t i = 0; i < 2; ++i) {
            if (!tools[i])
                tools[i] = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                    tools_root + (i == 0 ? "/left/pose" : "/right/pose"), 10);
            observation_publishers_.push_back(
                {object.object_id, i == 0 ? GraspManipulator::Left : GraspManipulator::Right,
                 i == 0 ? object_publisher : nullptr, tools[i]});
        }
    }
    RCLCPP_INFO(
        node_->get_logger(), "Perception poses are simulation ground truth, not camera detections");
    passive_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    base_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "sensors/simulation/base_pose", 10);
    time_reference_publisher_ = node_->create_publisher<sensor_msgs::msg::TimeReference>(
        "sensors/simulation/time_reference", 10);
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
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    while (running_.load()) {
        // Re-anchor the robot clock on the coherent snapshot so stamps stay near
        // wall time when physics runs slower than real time.
        RobotState state;
        if (robot_.read_state(state)) {
            if (state.sequence != motion_sequence_) {
                motion_sequence_ = state.sequence;
                motion_received_ = std::chrono::steady_clock::now();
            }
            clock_epoch_ = node_->now() - rclcpp::Duration::from_nanoseconds(
                                              static_cast<std::int64_t>(state.timestamp_ns));
        }
        executor.spin_some();
        publish_observations();
        PassiveJointStates passive_state;
        BasePoseState base_pose;
        if (robot_.read_state(base_pose)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = sample_stamp(base_pose.timestamp_ns);
            pose.header.frame_id = "simulation_world";
            pose.pose.position.x = base_pose.position.x;
            pose.pose.position.y = base_pose.position.y;
            pose.pose.position.z = base_pose.position.z;
            pose.pose.orientation.x = base_pose.orientation.x;
            pose.pose.orientation.y = base_pose.orientation.y;
            pose.pose.orientation.z = base_pose.orientation.z;
            pose.pose.orientation.w = base_pose.orientation.w;
            sensor_msgs::msg::TimeReference reference;
            reference.header = pose.header;
            reference.time_ref = rclcpp::Time(static_cast<std::int64_t>(base_pose.timestamp_ns));
            reference.source = "same_instance_mujoco";
            base_pose_publisher_->publish(pose);
            time_reference_publisher_->publish(reference);
        }
        if (robot_.read_state(passive_state)) {
            sensor_msgs::msg::JointState passive;
            passive.header.stamp = sample_stamp(passive_state.timestamp_ns);
            passive.name = {
                "caster_front_left_steering_joint", "caster_front_left_joint", "rocker_arm_joint",
                "caster_rear_right_steering_joint", "caster_rear_right_joint"};
            for (const auto& joint : passive_state.joints) {
                passive.position.push_back(joint.position);
                passive.velocity.push_back(joint.velocity);
            }
            passive_publisher_->publish(std::move(passive));
        }
        for (std::size_t index = 0; index < kLidarNames.size(); ++index) {
            publish_lidar(static_cast<Lidar>(index), index);
        }
        for (std::size_t index = 0; index < kCameraNames.size(); ++index) {
            publish_camera(static_cast<Camera>(index), index);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    executor.remove_node(node_);
}

void Ros2SensorAdapter::publish_observations() {
    if (std::chrono::steady_clock::now() - motion_received_ > std::chrono::milliseconds(300))
        return;
    for (const auto& publishers : observation_publishers_) {
        GraspObservation observation;
        if (!robot_.observe_grasp(publishers.object_id, publishers.hand, observation) ||
            !observation.valid || !observation.object_visible)
            continue;
        const auto convert = [&](const std::array<double, 3>& position,
                                 const std::array<double, 4>& orientation) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "simulation_world";
            pose.header.stamp = sample_stamp(observation.timestamp_ns);
            pose.pose.position.x = position[0];
            pose.pose.position.y = position[1];
            pose.pose.position.z = position[2];
            pose.pose.orientation.x = orientation[0];
            pose.pose.orientation.y = orientation[1];
            pose.pose.orientation.z = orientation[2];
            pose.pose.orientation.w = orientation[3];
            return pose;
        };
        const auto object = convert(observation.object_position, observation.object_orientation);
        const auto tool = convert(observation.tool_position, observation.tool_orientation);
        if (publishers.object) publishers.object->publish(object);
        publishers.tool->publish(tool);
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
