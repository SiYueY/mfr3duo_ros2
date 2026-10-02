#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_info.hpp"
#include "ros2_control_adapter.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    using hardware_interface::CallbackReturn;
    using hardware_interface::return_type;
    mfr3duo_hardware::Ros2ControlAdapter system;
    auto info = mfr3duo_hardware_test::make_info();
    auto invalid = info;
    invalid.hardware_parameters["viewer_enabled"] = "invalid";
    if (!check(system.on_init(invalid) == CallbackReturn::ERROR, "invalid viewer flag accepted"))
        return 1;
    auto viewer_info = info;
    viewer_info.hardware_parameters["viewer_enabled"] = "true";
    if (!check(system.on_init(viewer_info) == CallbackReturn::SUCCESS, "viewer flag rejected"))
        return 1;
    invalid = info;
    invalid.hardware_parameters["control_period"] = "0";
    if (!check(system.on_init(invalid) == CallbackReturn::ERROR, "invalid period accepted") ||
        !check(system.on_init(info) == CallbackReturn::SUCCESS, "initialization failed"))
        return 1;

    mfr3duo_hardware::Ros2ControlAdapter reordered;
    auto shuffled = info;
    std::reverse(shuffled.joints.begin(), shuffled.joints.end());
    std::reverse(shuffled.gpios.begin(), shuffled.gpios.end());
    if (!check(
            reordered.on_init(shuffled) == CallbackReturn::SUCCESS,
            "interface validation depends on URDF declaration order"))
        return 1;

    const auto states = system.export_state_interfaces();
    auto commands = system.export_command_interfaces();
    if (!check(states.size() == 70, "wrong state interface count") ||
        !check(commands.size() == 53, "wrong command interface count"))
        return 1;
    const auto has = [](const auto& items, const std::string& name) {
        return std::any_of(
            items.begin(), items.end(), [&](const auto& item) { return item.get_name() == name; });
    };
    if (!check(has(commands, "left_fr3v2_1_joint1/effort"), "left effort missing") ||
        !check(has(commands, "right_fr3v2_1_joint7/velocity"), "right velocity missing") ||
        !check(has(commands, "tmrv0_2_joint_3/velocity"), "TMR drive missing") ||
        !check(has(states, "imu/orientation.w"), "IMU missing") ||
        !check(has(states, "right_gripper/stalled"), "gripper stalled missing"))
        return 1;

    std::vector<std::string> partial{"left_fr3v2_1_joint1/velocity"};
    if (!check(
            system.prepare_command_mode_switch(partial, {}) == return_type::ERROR,
            "partial arm mode accepted"))
        return 1;
    std::vector<std::string> complete;
    for (int joint = 1; joint <= 7; ++joint)
        complete.push_back("left_fr3v2_1_joint" + std::to_string(joint) + "/velocity");
    for (auto& item : commands)
        if (item.get_name() == "right_fr3v2_1_joint1/position") item.set_value(0.123);
    auto mixed = complete;
    mixed.back() = "left_fr3v2_1_joint7/effort";
    if (!check(
            system.prepare_command_mode_switch(mixed, {}) == return_type::ERROR,
            "mixed arm mode accepted"))
        return 1;
    if (!check(
            system.prepare_command_mode_switch(complete, {}) == return_type::OK,
            "complete arm mode rejected") ||
        !check(
            system.perform_command_mode_switch(complete, {}) == return_type::OK,
            "arm mode switch failed"))
        return 1;
    if (!check(
            std::any_of(
                commands.begin(), commands.end(),
                [](const auto& item) {
                    return item.get_name() == "right_fr3v2_1_joint1/position" &&
                           item.get_value() == 0.123;
                }),
            "switching left arm reset right arm command"))
        return 1;
    if (!check(
            system.prepare_command_mode_switch({}, complete) == return_type::OK,
            "complete arm stop rejected") ||
        !check(
            system.prepare_command_mode_switch({}, {"left_fr3v2_1_joint1/position"}) ==
                return_type::ERROR,
            "wrong arm mode stop accepted"))
        return 1;

    const rclcpp_lifecycle::State lifecycle_state;
    const rclcpp::Time now;
    const auto period = rclcpp::Duration::from_seconds(0.002);
    mfr3duo_hardware::Ros2ControlAdapter running;
    if (!check(running.on_init(info) == CallbackReturn::SUCCESS, "runtime init failed")) return 1;
    if (!check(
            running.on_configure(lifecycle_state) == CallbackReturn::SUCCESS,
            "runtime configure failed"))
        return 1;
    auto running_commands = running.export_command_interfaces();
    const auto command = [&](const std::string& name) -> hardware_interface::CommandInterface* {
        const auto found = std::find_if(
            running_commands.begin(), running_commands.end(),
            [&](const auto& item) { return item.get_name() == name; });
        return found == running_commands.end() ? nullptr : &*found;
    };
    auto* drive_command = command("tmrv0_2_joint_1/velocity");
    auto* arm_command = command("left_fr3v2_1_joint1/position");
    if (!check(
            drive_command != nullptr && arm_command != nullptr,
            "runtime command interfaces missing"))
        return 1;
    drive_command->set_value(2.0);
    arm_command->set_value(1.0);
    if (!check(
            running.on_activate(lifecycle_state) == CallbackReturn::SUCCESS,
            "runtime activate failed") ||
        !check(
            drive_command->get_value() == 0.0 && arm_command->get_value() != 1.0,
            "activation retained stale commands") ||
        !check(
            running.write(now, period) == return_type::OK &&
                running.read(now, period) == return_type::OK,
            "active control cycle failed"))
        return 1;
    if (!check(
            running.prepare_command_mode_switch(partial, {}) == return_type::ERROR,
            "active partial mode switch accepted") ||
        !check(
            running.write(now, period) == return_type::OK &&
                running.read(now, period) == return_type::OK,
            "control cycle did not recover from rejected mode switch"))
        return 1;
    drive_command->set_value(2.0);
    if (!check(
            running.on_deactivate(lifecycle_state) == CallbackReturn::SUCCESS,
            "runtime deactivate failed") ||
        !check(drive_command->get_value() == 0.0, "deactivation did not stop drive command"))
        return 1;
    drive_command->set_value(2.0);
    if (!check(
            running.on_activate(lifecycle_state) == CallbackReturn::SUCCESS,
            "runtime reactivation failed") ||
        !check(drive_command->get_value() == 0.0, "reactivation retained stale drive command") ||
        !check(
            running.write(now, period) == return_type::OK &&
                running.read(now, period) == return_type::OK,
            "reactivated control cycle failed") ||
        !check(
            running.on_cleanup(lifecycle_state) == CallbackReturn::SUCCESS,
            "runtime cleanup failed"))
        return 1;
    // on_cleanup() was called from an active adapter; it must have stopped the
    // hardware safely and left the adapter re-configurable.
    if (!check(
            running.on_configure(lifecycle_state) == CallbackReturn::SUCCESS,
            "reconfigure after cleanup from Active failed") ||
        !check(
            running.on_activate(lifecycle_state) == CallbackReturn::SUCCESS,
            "reactivate after cleanup from Active failed") ||
        !check(
            running.write(now, period) == return_type::OK &&
                running.read(now, period) == return_type::OK,
            "control cycle after cleanup from Active failed") ||
        !check(
            running.on_deactivate(lifecycle_state) == CallbackReturn::SUCCESS,
            "deactivate after cleanup from Active failed") ||
        !check(
            running.on_cleanup(lifecycle_state) == CallbackReturn::SUCCESS,
            "cleanup after reconfigure failed"))
        return 1;
    rclcpp::shutdown();
    return 0;
}
