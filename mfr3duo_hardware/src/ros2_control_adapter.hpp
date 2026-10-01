#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "mfr3duo_hardware/robot_hardware.hpp"

namespace mfr3duo_hardware {

class Ros2SensorAdapter;

/**
 * @brief Adapts RobotHardware to ros2_control.
 *
 * The SystemInterface inheritance exists only so controller_manager and
 * pluginlib can load the adapter. It is not a second hardware API: every robot
 * access goes through RobotHardware, and the adapter never touches the MuJoCo
 * backend directly.
 */
class Ros2ControlAdapter final : public hardware_interface::SystemInterface {
public:
    Ros2ControlAdapter();
    ~Ros2ControlAdapter() override;

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start, const std::vector<std::string>& stop) override;
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start, const std::vector<std::string>& stop) override;

    hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    static constexpr std::size_t kArmCount = kArmJointCount;

    struct JointValues {
        std::array<double, 3> state{};
        std::array<double, 3> command{};
    };

    struct GripperValues {
        std::array<double, 4> state{};
        std::array<double, 3> command{};
    };

    bool valid_info() const;
    void copy_state(const RobotState& state);
    void copy_imu_state(const ImuState& state);
    void initialize_commands();
    hardware_interface::CallbackReturn activate_failed();

    RobotHardware robot_;
    std::unique_ptr<Ros2SensorAdapter> sensor_adapter_;

    RobotState snapshot_;
    ImuState imu_snapshot_;

    std::array<JointValues, 14> arms_{};
    JointValues spine_{};
    std::array<JointValues, 4> tmr_{};
    std::array<GripperValues, 2> grippers_{};
    std::array<double, 10> imu_{};

    std::array<JointControlMode, 2> arm_modes_{
        JointControlMode::Position, JointControlMode::Position};
    std::array<JointControlMode, 2> pending_modes_ = arm_modes_;
    std::chrono::nanoseconds control_period_{std::chrono::milliseconds(2)};
};

}  // namespace mfr3duo_hardware
