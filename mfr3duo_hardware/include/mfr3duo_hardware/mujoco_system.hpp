#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "mfr3duo_mujoco/simulation.hpp"

namespace mfr3duo_hardware {

class MujocoSensorBridge;

/** ROS 2 control interface for the complete MuJoCo robot. */
class MujocoSystem final : public hardware_interface::SystemInterface {
public:
  MujocoSystem();
  ~MujocoSystem() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
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
  static constexpr std::size_t kArmCount = 7;
  struct JointValues {
    std::array<double, 3> state{};
    std::array<double, 3> command{};
  };
  struct GripperValues {
    std::array<double, 4> state{};
    std::array<double, 3> command{};
  };
  bool valid_info() const;
  void copy_state(const mfr3duo_mujoco::RobotState& state);
  void copy_imu_state(const mfr3duo_mujoco::ImuState& state);
  void initialize_commands();
  bool safe_command();
  mfr3duo_mujoco::Simulation simulation_;
  std::unique_ptr<MujocoSensorBridge> sensor_bridge_;
  mfr3duo_mujoco::RobotState snapshot_;
  mfr3duo_mujoco::ImuState imu_snapshot_;
  mfr3duo_mujoco::RobotCommand command_;
  std::array<JointValues, 14> arms_{};
  JointValues spine_{};
  std::array<JointValues, 4> tmr_{};
  std::array<GripperValues, 2> grippers_{};
  std::array<double, 10> imu_{};
  std::array<mfr3duo_mujoco::JointControlMode, 2> arm_modes_{
      mfr3duo_mujoco::JointControlMode::Position,
      mfr3duo_mujoco::JointControlMode::Position};
  std::array<mfr3duo_mujoco::JointControlMode, 2> pending_modes_ = arm_modes_;
  std::size_t steps_per_cycle_{1};
};

}  // namespace mfr3duo_hardware
