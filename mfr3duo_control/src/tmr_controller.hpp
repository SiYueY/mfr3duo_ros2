#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "mfr3duo_control/tmr_kinematics.hpp"
#include "mfr3duo_control/tmr_odometry.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace mfr3duo_control {

class TmrController final : public controller_interface::ControllerInterface {
public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    struct Command {
        double vx{0.0};
        double vy{0.0};
        double wz{0.0};
        std::int64_t received_ns{0};
    };
    void hold() noexcept;
    void update_odometry(const rclcpp::Time& time) noexcept;
    TmrOdometry odometry_;
    OdometryStatus pending_diagnostic_{OdometryStatus::Initialized};
    double odom_confidence_{1.0};
    std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> tf_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
        diagnostic_publisher_;
    std::int64_t last_odom_ns_{0}, last_diagnostic_ns_{0}, odom_period_ns_{20000000},
        diagnostic_period_ns_{1000000000};
    std::array<ModuleGeometry, 2> modules_{};
    std::array<std::string, 2> steering_names_{};
    std::array<std::string, 2> drive_names_{};
    std::array<std::size_t, 2> steering_command_{};
    std::array<std::size_t, 2> drive_command_{};
    std::array<std::size_t, 2> steering_state_{};
    std::array<std::size_t, 2> drive_state_{};
    std::array<std::size_t, 2> drive_position_state_{};
    std::array<double, 2> last_steering_{};
    std::array<double, 3> last_velocity_{};
    realtime_tools::RealtimeBuffer<Command> command_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::atomic<bool> active_{false};
    std::int64_t activated_ns_{0};
    bool holding_{true};
    double radius_{0.05};
    double timeout_{0.5};
    double slow_{0.1};
    double stop_{0.6};
    double steering_rate_{1.5};
    double max_wheel_velocity_{8.0};
    double max_linear_velocity_{0.3};
    double max_angular_velocity_{0.5};
    double linear_acceleration_{0.5};
    double angular_acceleration_{0.5};
};

}  // namespace mfr3duo_control
