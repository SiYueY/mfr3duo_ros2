#include "ros2_control_adapter.hpp"

#include <sstream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <string>
#include <unordered_set>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "ros2_sensor_adapter.hpp"

namespace mfr3duo_hardware {
namespace {

constexpr std::array<const char*, 3> kJointInterfaces{"position", "velocity", "effort"};
constexpr std::array<const char*, 4> kGripperStates{"width", "velocity", "effort", "stalled"};
constexpr std::array<const char*, 10> kImuInterfaces{
    "orientation.x",         "orientation.y",        "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",   "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z"};
constexpr std::array<const char*, 4> kTmrNames{
    "tmrv0_2_joint_0", "tmrv0_2_joint_1", "tmrv0_2_joint_2", "tmrv0_2_joint_3"};
constexpr std::array<const char*, 2> kGripperNames{"left_gripper", "right_gripper"};
constexpr std::array<const char*, 2> kFingerNames{
    "left_fr3v2_1_finger_joint1", "right_fr3v2_1_finger_joint1"};
constexpr const char* kSpineName = "franka_spine_vertical_joint";
constexpr const char* kControlPeriodParameter = "control_period";

std::string arm_name(std::size_t arm, std::size_t joint) {
    return std::string(arm == 0 ? "left_fr3v2_1_joint" : "right_fr3v2_1_joint") +
           std::to_string(joint + 1);
}

bool has_exact_interfaces(
    const std::vector<hardware_interface::InterfaceInfo>& actual,
    std::initializer_list<const char*> expected) {
    if (actual.size() != expected.size()) return false;
    std::unordered_set<std::string> names;
    for (const auto& item : actual) names.insert(item.name);
    if (names.size() != actual.size()) return false;
    for (const char* name : expected) {
        if (names.count(name) == 0) return false;
    }
    return true;
}

bool matches(
    const hardware_interface::ComponentInfo& item, const std::string& name,
    std::initializer_list<const char*> commands, std::initializer_list<const char*> states) {
    return item.name == name && has_exact_interfaces(item.command_interfaces, commands) &&
           has_exact_interfaces(item.state_interfaces, states);
}

// Parse a decimal seconds string such as "0.002" into a nanosecond duration.
bool parse_control_period(const std::string& text, std::chrono::nanoseconds& period) {
    try {
        std::size_t consumed = 0;
        const double seconds = std::stod(text, &consumed);
        if (consumed != text.size() || !std::isfinite(seconds) || seconds <= 0.0) return false;
        const auto nanoseconds =
            std::chrono::nanoseconds(static_cast<std::int64_t>(std::llround(seconds * 1.0e9)));
        if (nanoseconds <= std::chrono::nanoseconds::zero()) return false;
        period = nanoseconds;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

Ros2ControlAdapter::Ros2ControlAdapter() = default;
Ros2ControlAdapter::~Ros2ControlAdapter() = default;

hardware_interface::CallbackReturn Ros2ControlAdapter::on_init(
    const hardware_interface::HardwareInfo& info) {
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS ||
        !valid_info()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto it = info.hardware_parameters.find(kControlPeriodParameter);
    if (it != info.hardware_parameters.end() &&
        !parse_control_period(it->second, control_period_)) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    const auto viewer = info.hardware_parameters.find("viewer_enabled");
    viewer_enabled_ = false;
    if (viewer != info.hardware_parameters.end()) {
        if (viewer->second == "true" || viewer->second == "1") {
            viewer_enabled_ = true;
        } else if (viewer->second != "false" && viewer->second != "0") {
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool Ros2ControlAdapter::valid_info() const {
    if (info_.joints.size() != 21 || info_.gpios.size() != 2 || info_.sensors.size() != 1) {
        return false;
    }
    const auto find_named =
        [](const auto& components,
           const std::string& name) -> const hardware_interface::ComponentInfo* {
        const auto found = std::find_if(
            components.begin(), components.end(),
            [&](const auto& item) { return item.name == name; });
        return found == components.end() ? nullptr : &*found;
    };
    for (std::size_t arm = 0; arm < 2; ++arm) {
        for (std::size_t joint = 0; joint < kArmCount; ++joint) {
            const auto name = arm_name(arm, joint);
            const auto* item = find_named(info_.joints, name);
            if (item == nullptr || !matches(
                                       *item, name, {"position", "velocity", "effort"},
                                       {"position", "velocity", "effort"})) {
                return false;
            }
        }
    }
    for (const auto* name : kFingerNames) {
        const auto* item = find_named(info_.joints, name);
        if (item == nullptr || !matches(*item, name, {}, {"position", "velocity"})) return false;
    }
    const auto* spine = find_named(info_.joints, kSpineName);
    if (spine == nullptr || !matches(*spine, kSpineName, {"position"}, {"position", "velocity"})) {
        return false;
    }
    for (std::size_t index = 0; index < 4; ++index) {
        const auto* item = find_named(info_.joints, kTmrNames[index]);
        if (item == nullptr || !matches(
                                   *item, kTmrNames[index],
                                   index % 2 == 0 ? std::initializer_list<const char*>{"position"}
                                                  : std::initializer_list<const char*>{"velocity"},
                                   {"position", "velocity"})) {
            return false;
        }
    }
    for (std::size_t index = 0; index < 2; ++index) {
        const auto* item = find_named(info_.gpios, kGripperNames[index]);
        if (item == nullptr || !matches(
                                   *item, kGripperNames[index], {"width", "velocity", "effort"},
                                   {"width", "velocity", "effort", "stalled"})) {
            return false;
        }
    }
    return matches(
        info_.sensors[0], "imu", {},
        {"orientation.x", "orientation.y", "orientation.z", "orientation.w", "angular_velocity.x",
         "angular_velocity.y", "angular_velocity.z", "linear_acceleration.x",
         "linear_acceleration.y", "linear_acceleration.z"});
}

hardware_interface::CallbackReturn Ros2ControlAdapter::on_configure(
    const rclcpp_lifecycle::State&) {
    RobotHardwareOptions options;
    options.control_period = control_period_;
    options.viewer_enabled = viewer_enabled_;
    const auto model = info_.hardware_parameters.find("model_path");
    if (model != info_.hardware_parameters.end()) options.model_path = model->second;
    const auto keyframe = info_.hardware_parameters.find("initial_keyframe");
    if (keyframe != info_.hardware_parameters.end()) options.initial_keyframe = keyframe->second;
    const auto mappings = info_.hardware_parameters.find("grasp_objects");
    if (mappings != info_.hardware_parameters.end()) {
        std::istringstream input(mappings->second);
        std::string entry;
        while (std::getline(input, entry, ';')) {
            if (entry.empty()) continue;
            std::istringstream fields(entry);
            SimulationObjectMapping mapping;
            std::string extra;
            if (!std::getline(fields, mapping.object_id, '=') ||
                !std::getline(fields, mapping.body_name, '=') ||
                !std::getline(fields, mapping.collision_geom, '=') ||
                std::getline(fields, extra, '='))
                return hardware_interface::CallbackReturn::ERROR;
            options.grasp_objects.push_back(std::move(mapping));
        }
    }
    if (!robot_.initialize(options)) return hardware_interface::CallbackReturn::ERROR;
    try {
        sensor_adapter_ = std::make_unique<Ros2SensorAdapter>(robot_, options.grasp_objects);
    } catch (const std::exception&) {
        robot_.shutdown();
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Roll back a partially completed activation. RobotHardware::shutdown() performs
// a best-effort safe stop from Active and is idempotent, so a later
// on_error/on_cleanup path stays harmless.
hardware_interface::CallbackReturn Ros2ControlAdapter::activate_failed() {
    if (sensor_adapter_) {
        sensor_adapter_->stop();
        sensor_adapter_.reset();
    }
    robot_.shutdown();
    return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn Ros2ControlAdapter::on_activate(const rclcpp_lifecycle::State&) {
    if (!robot_.activate()) return hardware_interface::CallbackReturn::ERROR;
    if (!robot_.read_state(snapshot_) || !robot_.read_state(imu_snapshot_)) {
        return activate_failed();
    }
    copy_state(snapshot_);
    copy_imu_state(imu_snapshot_);
    initialize_commands();
    try {
        sensor_adapter_->start();
    } catch (const std::exception&) {
        return activate_failed();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2ControlAdapter::on_deactivate(
    const rclcpp_lifecycle::State&) {
    if (sensor_adapter_) sensor_adapter_->stop();
    if (!robot_.deactivate()) return hardware_interface::CallbackReturn::ERROR;
    // Drop any command the controllers left behind; the hardware already holds.
    initialize_commands();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ros2ControlAdapter::on_cleanup(const rclcpp_lifecycle::State&) {
    if (sensor_adapter_) sensor_adapter_->stop();
    sensor_adapter_.reset();
    return robot_.shutdown() ? hardware_interface::CallbackReturn::SUCCESS
                             : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn Ros2ControlAdapter::on_shutdown(
    const rclcpp_lifecycle::State& state) {
    return on_cleanup(state);
}

hardware_interface::CallbackReturn Ros2ControlAdapter::on_error(
    const rclcpp_lifecycle::State& state) {
    return on_cleanup(state);
}

std::vector<hardware_interface::StateInterface> Ros2ControlAdapter::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> output;
    output.reserve(14 * 3 + 2 + 4 * 2 + 2 * 4 + 4 + 10);
    for (std::size_t index = 0; index < arms_.size(); ++index) {
        const auto name = arm_name(index / kArmCount, index % kArmCount);
        for (std::size_t channel = 0; channel < 3; ++channel) {
            output.emplace_back(name, kJointInterfaces[channel], &arms_[index].state[channel]);
        }
    }
    for (std::size_t channel = 0; channel < 2; ++channel) {
        output.emplace_back(kSpineName, kJointInterfaces[channel], &spine_.state[channel]);
    }
    for (std::size_t index = 0; index < tmr_.size(); ++index) {
        for (std::size_t channel = 0; channel < 2; ++channel) {
            output.emplace_back(
                kTmrNames[index], kJointInterfaces[channel], &tmr_[index].state[channel]);
        }
    }
    for (std::size_t index = 0; index < grippers_.size(); ++index) {
        for (std::size_t channel = 0; channel < 4; ++channel) {
            output.emplace_back(
                kGripperNames[index], kGripperStates[channel], &grippers_[index].state[channel]);
        }
    }
    for (std::size_t side = 0; side < fingers_.size(); ++side) {
        for (std::size_t channel = 0; channel < 2; ++channel)
            output.emplace_back(
                kFingerNames[side], kJointInterfaces[channel], &fingers_[side][channel]);
    }
    for (std::size_t channel = 0; channel < imu_.size(); ++channel) {
        output.emplace_back("imu", kImuInterfaces[channel], &imu_[channel]);
    }
    return output;
}

std::vector<hardware_interface::CommandInterface> Ros2ControlAdapter::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> output;
    output.reserve(14 * 3 + 1 + 4 + 2 * 3);
    for (std::size_t index = 0; index < arms_.size(); ++index) {
        const auto name = arm_name(index / kArmCount, index % kArmCount);
        for (std::size_t channel = 0; channel < 3; ++channel) {
            output.emplace_back(name, kJointInterfaces[channel], &arms_[index].command[channel]);
        }
    }
    output.emplace_back(kSpineName, "position", &spine_.command[0]);
    for (std::size_t index = 0; index < tmr_.size(); ++index) {
        const auto channel = index % 2;
        output.emplace_back(
            kTmrNames[index], kJointInterfaces[channel], &tmr_[index].command[channel]);
    }
    for (std::size_t index = 0; index < grippers_.size(); ++index) {
        for (std::size_t channel = 0; channel < 3; ++channel) {
            output.emplace_back(
                kGripperNames[index], kGripperStates[channel], &grippers_[index].command[channel]);
        }
    }
    return output;
}

void Ros2ControlAdapter::copy_state(const RobotState& state) {
    for (std::size_t index = 0; index < kArmCount; ++index) {
        const auto& left = state.left_arm.joints[index];
        const auto& right = state.right_arm.joints[index];
        arms_[index].state = {left.position, left.velocity, left.effort};
        arms_[kArmCount + index].state = {right.position, right.velocity, right.effort};
    }
    spine_.state = {state.spine.position, state.spine.velocity, 0.0};
    const std::array<JointState, 4> tmr{
        state.tmr.front_steering, state.tmr.front_drive, state.tmr.rear_steering,
        state.tmr.rear_drive};
    for (std::size_t index = 0; index < tmr.size(); ++index) {
        tmr_[index].state = {tmr[index].position, tmr[index].velocity, tmr[index].effort};
    }
    const std::array<GripperState, 2> grippers{state.left_gripper, state.right_gripper};
    for (std::size_t index = 0; index < grippers.size(); ++index) {
        fingers_[index] = {grippers[index].width * 0.5, grippers[index].velocity * 0.5};
        grippers_[index].state = {
            grippers[index].width, grippers[index].velocity, grippers[index].effort,
            grippers[index].stalled ? 1.0 : 0.0};
    }
}

void Ros2ControlAdapter::copy_imu_state(const ImuState& imu) {
    imu_ = {imu.orientation.x,        imu.orientation.y,         imu.orientation.z,
            imu.orientation.w,        imu.angular_velocity.x,    imu.angular_velocity.y,
            imu.angular_velocity.z,   imu.linear_acceleration.x, imu.linear_acceleration.y,
            imu.linear_acceleration.z};
}

void Ros2ControlAdapter::initialize_commands() {
    for (auto& joint : arms_) joint.command = {joint.state[0], 0.0, 0.0};
    spine_.command = {spine_.state[0], 0.0, 0.0};
    for (std::size_t index = 0; index < tmr_.size(); ++index) {
        tmr_[index].command = {index % 2 == 0 ? tmr_[index].state[0] : 0.0, 0.0, 0.0};
    }
    for (auto& gripper : grippers_) gripper.command = {gripper.state[0], 0.0, 0.0};
}

hardware_interface::return_type Ros2ControlAdapter::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!robot_.update() || !robot_.read_state(snapshot_)) {
        return hardware_interface::return_type::ERROR;
    }
    copy_state(snapshot_);
    if (!robot_.read_state(imu_snapshot_)) return hardware_interface::return_type::ERROR;
    copy_imu_state(imu_snapshot_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ros2ControlAdapter::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
    RobotCommand command;
    command.left_arm.mode = arm_modes_[0];
    command.right_arm.mode = arm_modes_[1];
    for (std::size_t index = 0; index < kArmCount; ++index) {
        auto& left = command.left_arm.joints[index];
        auto& right = command.right_arm.joints[index];
        left.position = arms_[index].command[0];
        left.velocity = arms_[index].command[1];
        left.effort = arms_[index].command[2];
        right.position = arms_[kArmCount + index].command[0];
        right.velocity = arms_[kArmCount + index].command[1];
        right.effort = arms_[kArmCount + index].command[2];
    }
    command.spine.position = spine_.command[0];
    command.tmr.front_steering_position = tmr_[0].command[0];
    command.tmr.front_drive_velocity = tmr_[1].command[1];
    command.tmr.rear_steering_position = tmr_[2].command[0];
    command.tmr.rear_drive_velocity = tmr_[3].command[1];
    command.left_gripper = {
        grippers_[0].command[0], grippers_[0].command[1], grippers_[0].command[2]};
    command.right_gripper = {
        grippers_[1].command[0], grippers_[1].command[1], grippers_[1].command[2]};
    return robot_.write_command(command) ? hardware_interface::return_type::OK
                                         : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type Ros2ControlAdapter::prepare_command_mode_switch(
    const std::vector<std::string>& start, const std::vector<std::string>& stop) {
    auto requested = arm_modes_;
    for (std::size_t arm = 0; arm < 2; ++arm) {
        std::size_t started = 0;
        std::size_t stopped = 0;
        int mode = -1;
        for (std::size_t joint = 0; joint < kArmCount; ++joint) {
            const auto name = arm_name(arm, joint) + "/";
            int joint_starts = 0;
            int joint_stops = 0;
            for (std::size_t channel = 0; channel < 3; ++channel) {
                const auto key = name + kJointInterfaces[channel];
                if (std::find(start.begin(), start.end(), key) != start.end()) {
                    ++started;
                    ++joint_starts;
                    if (mode != -1 && mode != static_cast<int>(channel)) {
                        return hardware_interface::return_type::ERROR;
                    }
                    mode = static_cast<int>(channel);
                }
                if (std::find(stop.begin(), stop.end(), key) != stop.end()) {
                    ++stopped;
                    ++joint_stops;
                    if (channel != static_cast<std::size_t>(arm_modes_[arm])) {
                        return hardware_interface::return_type::ERROR;
                    }
                }
            }
            if (joint_starts > 1 || joint_stops > 1) {
                return hardware_interface::return_type::ERROR;
            }
        }
        if ((started != 0 && started != kArmCount) || (stopped != 0 && stopped != kArmCount)) {
            return hardware_interface::return_type::ERROR;
        }
        if (started == kArmCount) requested[arm] = static_cast<JointControlMode>(mode);
    }
    pending_modes_ = requested;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ros2ControlAdapter::perform_command_mode_switch(
    const std::vector<std::string>&, const std::vector<std::string>& stop) {
    for (std::size_t arm = 0; arm < 2; ++arm) {
        const std::string prefix = arm == 0 ? "left_fr3v2_1_joint" : "right_fr3v2_1_joint";
        const bool stopped = std::any_of(stop.begin(), stop.end(), [&](const auto& key) {
            return key.compare(0, prefix.size(), prefix) == 0;
        });
        if (!stopped && arm_modes_[arm] == pending_modes_[arm]) continue;
        for (std::size_t joint = 0; joint < kArmCount; ++joint) {
            auto& values = arms_[arm * kArmCount + joint];
            values.command = {values.state[0], 0.0, 0.0};
        }
    }
    arm_modes_ = pending_modes_;
    return hardware_interface::return_type::OK;
}

}  // namespace mfr3duo_hardware

PLUGINLIB_EXPORT_CLASS(mfr3duo_hardware::Ros2ControlAdapter, hardware_interface::SystemInterface)
