#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "mfr3duo_msgs/action/move.hpp"
#include "mfr3duo_msgs/action/grasp.hpp"
#include <functional>
#include "rclcpp_action/rclcpp_action.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace mfr3duo_control {

class Mfr3DuoGripperController final : public controller_interface::ControllerInterface {
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
    using Action = control_msgs::action::GripperCommand;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    struct Operation {
        enum class Mode { Legacy, Move, Grasp };
        Mode mode{Mode::Legacy};
        const void* identity{nullptr};
        std::function<void(Operation&, int)> report;
        double speed{0.05};
        double expected_width{0.0}, inner{0.005}, outer{0.005};
        std::int64_t started{0};
        double width{0.0};
        double effort{0.0};
        std::atomic<bool> cancel{false};
        // 0 active; 1 success; 2 abort; 3 canceled. Result fields precede release store.
        std::atomic<int> terminal{0};
        std::atomic<double> position{0.0};
        std::atomic<double> measured_effort{0.0};
        std::atomic<bool> reached{false};
        std::atomic<bool> stalled{false};
        std::int64_t stall_since{0};
        double stall_effort{0.0};
        bool hold_written{false};
    };
    template <class A>
    typename rclcpp_action::Server<A>::SharedPtr create_action(const std::string& name);
    void monitor();
    void hold(double width);
    std::string gpio_;
    double velocity_{0.05};
    double effort_{20.0};
    double hold_effort_{10.0};
    double minimum_{0.0};
    double maximum_{0.04};
    double tolerance_{0.001};
    double stall_velocity_{0.001};
    double stall_timeout_{0.5};
    bool allow_stalling_{true};
    double goal_timeout_{10.0};
    std::array<std::size_t, 3> commands_{};
    std::array<std::size_t, 4> states_{};
    bool mapped_{false};
    std::atomic<bool> active_{false};
    std::atomic<bool> busy_{false};
    realtime_tools::RealtimeBuffer<std::shared_ptr<Operation>> operation_;
    std::mutex monitor_mutex_;
    std::shared_ptr<Operation> monitored_;
    rclcpp_action::Server<Action>::SharedPtr server_;
    rclcpp_action::Server<mfr3duo_msgs::action::Move>::SharedPtr move_server_;
    rclcpp_action::Server<mfr3duo_msgs::action::Grasp>::SharedPtr grasp_server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mfr3duo_control
