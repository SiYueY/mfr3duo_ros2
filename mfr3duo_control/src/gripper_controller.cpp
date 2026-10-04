#include "gripper_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <type_traits>

#include "pluginlib/class_list_macros.hpp"

namespace mfr3duo_control {
namespace {
std::int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
using CallbackReturn = controller_interface::CallbackReturn;
}  // namespace

CallbackReturn Mfr3DuoGripperController::on_init() {
    try {
        auto_declare<std::string>("gpio", "left_gripper");
        auto_declare<double>("default_velocity", 0.05);
        auto_declare<double>("default_effort", 20.0);
        auto_declare<double>("cancel_hold_effort", 10.0);
        auto_declare<double>("min_position", 0.0);
        auto_declare<double>("max_position", 0.04);
        auto_declare<double>("goal_tolerance", 0.001);
        auto_declare<double>("stall_velocity_threshold", 0.001);
        auto_declare<double>("stall_timeout", 0.5);
        auto_declare<bool>("allow_stalling", true);
        auto_declare<double>("goal_timeout", 10.0);
    } catch (const std::exception&) {
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn Mfr3DuoGripperController::on_configure(const rclcpp_lifecycle::State&) {
    const auto node = get_node();
    gpio_ = node->get_parameter("gpio").as_string();
    velocity_ = node->get_parameter("default_velocity").as_double();
    effort_ = node->get_parameter("default_effort").as_double();
    hold_effort_ = node->get_parameter("cancel_hold_effort").as_double();
    minimum_ = node->get_parameter("min_position").as_double();
    maximum_ = node->get_parameter("max_position").as_double();
    tolerance_ = node->get_parameter("goal_tolerance").as_double();
    stall_velocity_ = node->get_parameter("stall_velocity_threshold").as_double();
    stall_timeout_ = node->get_parameter("stall_timeout").as_double();
    allow_stalling_ = node->get_parameter("allow_stalling").as_bool();
    goal_timeout_ = node->get_parameter("goal_timeout").as_double();
    for (const double value :
         {velocity_, effort_, hold_effort_, tolerance_, stall_velocity_, stall_timeout_,
          goal_timeout_})
        if (!std::isfinite(value) || value <= 0.0) return CallbackReturn::ERROR;
    if ((gpio_ != "left_gripper" && gpio_ != "right_gripper") || velocity_ > 0.20 ||
        effort_ > 100.0 || hold_effort_ > 100.0 || !std::isfinite(minimum_) ||
        !std::isfinite(maximum_) || minimum_ < 0.0 || maximum_ > 0.04 || minimum_ >= maximum_)
        return CallbackReturn::ERROR;
    server_ = create_action<Action>("~/gripper_cmd");
    move_server_ = create_action<mfr3duo_msgs::action::Move>("~/move");
    grasp_server_ = create_action<mfr3duo_msgs::action::Grasp>("~/grasp");
    timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this] { monitor(); });
    return CallbackReturn::SUCCESS;
}

template <class A>
typename rclcpp_action::Server<A>::SharedPtr Mfr3DuoGripperController::create_action(
    const std::string& name) {
    using Handle = rclcpp_action::ServerGoalHandle<A>;
    return rclcpp_action::create_server<A>(
        get_node(), name,
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename A::Goal> goal) {
            double width, speed = velocity_, force = effort_;
            if constexpr (std::is_same_v<A, Action>) {
                width = 2 * goal->command.position;
                force = goal->command.max_effort;
            } else {
                width = goal->width;
                speed = goal->speed;
                if constexpr (std::is_same_v<A, mfr3duo_msgs::action::Grasp>) {
                    force = goal->force;
                    if (!std::isfinite(goal->epsilon.inner) || goal->epsilon.inner < 0 ||
                        !std::isfinite(goal->epsilon.outer) || goal->epsilon.outer < 0 ||
                        !std::isfinite(force) || force <= 0)
                        return rclcpp_action::GoalResponse::REJECT;
                }
            }
            if (!active_.load(std::memory_order_acquire) || !std::isfinite(width) ||
                width < 2 * minimum_ || width > 2 * maximum_ || !std::isfinite(speed) ||
                speed <= 0 || speed > .20 || !std::isfinite(force) || force < 0 || force > 100)
                return rclcpp_action::GoalResponse::REJECT;
            bool expected = false;
            if (!busy_.compare_exchange_strong(expected, true))
                return rclcpp_action::GoalResponse::REJECT;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<Handle> goal) {
            std::lock_guard<std::mutex> guard(monitor_mutex_);
            if (!monitored_ || monitored_->identity != goal.get() ||
                monitored_->terminal.load() != 0)
                return rclcpp_action::CancelResponse::REJECT;
            monitored_->cancel.store(true, std::memory_order_release);
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Handle> goal) {
            auto op = std::make_shared<Operation>();
            op->identity = goal.get();
            op->speed = velocity_;
            op->effort = effort_;
            if constexpr (std::is_same_v<A, Action>) {
                op->width = 2 * goal->get_goal()->command.position;
                const auto requested = goal->get_goal()->command.max_effort;
                op->effort = requested == 0 ? effort_ : requested;
            } else {
                op->expected_width = goal->get_goal()->width;
                op->speed = goal->get_goal()->speed;
                op->width = op->expected_width;
                op->mode = Operation::Mode::Move;
                if constexpr (std::is_same_v<A, mfr3duo_msgs::action::Grasp>) {
                    op->mode = Operation::Mode::Grasp;
                    op->effort = goal->get_goal()->force;
                    op->inner = goal->get_goal()->epsilon.inner;
                    op->outer = goal->get_goal()->epsilon.outer;
                    // Closing preload continues until blocked; expected width is the object size.
                    op->width = 2 * minimum_;
                }
            }
            op->report = [goal](Operation& state, int terminal) {
                if (terminal == 0) {
                    auto f = std::make_shared<typename A::Feedback>();
                    if constexpr (std::is_same_v<A, Action>) {
                        f->position = state.position.load();
                        f->effort = state.measured_effort.load();
                        f->stalled = state.stalled.load();
                        f->reached_goal = state.reached.load();
                    } else
                        f->current_width = 2 * state.position.load();
                    if (goal->is_executing()) goal->publish_feedback(f);
                    return;
                }
                auto r = std::make_shared<typename A::Result>();
                if constexpr (std::is_same_v<A, Action>) {
                    r->position = state.position.load();
                    r->effort = state.measured_effort.load();
                    r->stalled = state.stalled.load();
                    r->reached_goal = state.reached.load();
                } else {
                    r->success = terminal == 1 && !goal->is_canceling();
                    if (!r->success)
                        r->error = terminal == 3
                                       ? "canceled"
                                       : "target not achieved, timeout or device inactive";
                }
                if (terminal == 3)
                    goal->canceled(r);
                else if (terminal == 1 && !goal->is_canceling())
                    goal->succeed(r);
                else
                    goal->abort(r);
            };
            std::lock_guard<std::mutex> guard(monitor_mutex_);
            monitored_ = op;
            operation_.writeFromNonRT(op);
            if (!active_.load(std::memory_order_acquire)) op->terminal.store(2);
        });
}

controller_interface::InterfaceConfiguration
Mfr3DuoGripperController::command_interface_configuration() const {
    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        {gpio_ + "/width", gpio_ + "/velocity", gpio_ + "/effort"}};
}
controller_interface::InterfaceConfiguration
Mfr3DuoGripperController::state_interface_configuration() const {
    return {
        controller_interface::interface_configuration_type::INDIVIDUAL,
        {gpio_ + "/width", gpio_ + "/velocity", gpio_ + "/effort", gpio_ + "/stalled"}};
}

CallbackReturn Mfr3DuoGripperController::on_activate(const rclcpp_lifecycle::State&) {
    mapped_ = false;
    if (command_interfaces_.size() != 3 || state_interfaces_.size() != 4)
        return CallbackReturn::ERROR;
    const auto locate = [](const auto& interfaces, const std::string& name, std::size_t& index) {
        for (std::size_t i = 0; i < interfaces.size(); ++i)
            if (interfaces[i].get_name() == name) {
                index = i;
                return true;
            }
        return false;
    };
    const std::array<const char*, 4> names{"width", "velocity", "effort", "stalled"};
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (!locate(state_interfaces_, gpio_ + "/" + names[i], states_[i]))
            return CallbackReturn::ERROR;
        if (i < 3 && !locate(command_interfaces_, gpio_ + "/" + names[i], commands_[i]))
            return CallbackReturn::ERROR;
    }
    const double width = state_interfaces_[states_[0]].get_value();
    if (!std::isfinite(width) || width < 0.0 || width > 0.08) return CallbackReturn::ERROR;
    mapped_ = true;
    hold(width);
    active_.store(true, std::memory_order_release);
    return CallbackReturn::SUCCESS;
}
void Mfr3DuoGripperController::hold(double width) {
    command_interfaces_[commands_[0]].set_value(width);
    command_interfaces_[commands_[1]].set_value(velocity_);
    command_interfaces_[commands_[2]].set_value(hold_effort_);
}
CallbackReturn Mfr3DuoGripperController::on_deactivate(const rclcpp_lifecycle::State&) {
    active_.store(false, std::memory_order_release);
    if (mapped_ && command_interfaces_.size() == 3 && state_interfaces_.size() == 4) {
        const double width = state_interfaces_[states_[0]].get_value();
        if (std::isfinite(width)) hold(width);
    }
    {
        std::lock_guard<std::mutex> guard(monitor_mutex_);
        if (monitored_) {
            int expected = 0;
            monitored_->terminal.compare_exchange_strong(expected, 2, std::memory_order_acq_rel);
        }
    }
    monitor();
    return CallbackReturn::SUCCESS;
}
CallbackReturn Mfr3DuoGripperController::on_cleanup(const rclcpp_lifecycle::State& state) {
    on_deactivate(state);
    timer_.reset();
    server_.reset();
    move_server_.reset();
    grasp_server_.reset();
    return CallbackReturn::SUCCESS;
}
CallbackReturn Mfr3DuoGripperController::on_error(const rclcpp_lifecycle::State& state) {
    return on_cleanup(state);
}

controller_interface::return_type Mfr3DuoGripperController::update(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!active_.load(std::memory_order_acquire)) return controller_interface::return_type::OK;
    const auto operation = *operation_.readFromRT();
    const double width = state_interfaces_[states_[0]].get_value();
    const double velocity = state_interfaces_[states_[1]].get_value();
    const double effort = state_interfaces_[states_[2]].get_value();
    if (!std::isfinite(width) || !std::isfinite(velocity) || !std::isfinite(effort) ||
        width < -0.002 || width > 0.082) {
        if (operation) operation->terminal.store(2, std::memory_order_release);
        return controller_interface::return_type::ERROR;
    }
    if (!operation || operation->terminal.load(std::memory_order_acquire) != 0)
        return controller_interface::return_type::OK;
    operation->position.store(width * 0.5, std::memory_order_relaxed);
    operation->measured_effort.store(effort, std::memory_order_relaxed);
    if (operation->cancel.load(std::memory_order_acquire)) {
        if (!operation->hold_written) {
            hold(width);
            operation->hold_written = true;
        } else {
            // The previous update's hold crossed hardware.write before this read.
            operation->terminal.store(3, std::memory_order_release);
        }
        return controller_interface::return_type::OK;
    }
    command_interfaces_[commands_[0]].set_value(operation->width);
    command_interfaces_[commands_[1]].set_value(operation->speed);
    command_interfaces_[commands_[2]].set_value(operation->effort);
    const auto now = steady_ns();
    if (operation->started == 0) operation->started = now;
    if (static_cast<double>(now - operation->started) * 1e-9 >= goal_timeout_) {
        hold(width);
        operation->terminal.store(2, std::memory_order_release);
        return controller_interface::return_type::OK;
    }
    const bool reached = std::abs(width * 0.5 - operation->width * 0.5) <= tolerance_;
    operation->reached.store(reached, std::memory_order_relaxed);
    if (reached) {
        if (operation->mode == Operation::Mode::Grasp) hold(width);
        operation->terminal.store(
            operation->mode == Operation::Mode::Grasp ? 2 : 1, std::memory_order_release);
        return controller_interface::return_type::OK;
    }
    // max_effort is a cap, not a force target. A compliant position controller
    // can stop on an object below that cap; requiring saturation leaves such a
    // closing action pending forever. Contact identity is verified by the task.
    const bool blocking = std::abs(velocity * 0.5) <= stall_velocity_ && effort > 1e-3;
    if (!blocking)
        operation->stall_since = 0;
    else if (
        operation->stall_since == 0 ||
        std::abs(effort - operation->stall_effort) > std::max(.05, .05 * effort)) {
        // A stationary finger while its closing preload is still increasing is
        // not a settled stall. Start the dwell again until measured force settles.
        operation->stall_since = now;
        operation->stall_effort = effort;
    } else if (static_cast<double>(now - operation->stall_since) * 1e-9 >= stall_timeout_) {
        operation->stalled.store(true, std::memory_order_relaxed);
        bool success = operation->mode == Operation::Mode::Legacy && allow_stalling_;
        if (operation->mode == Operation::Mode::Grasp)
            success = width > operation->expected_width - operation->inner &&
                      width < operation->expected_width + operation->outer;
        if (!success) hold(width);
        operation->terminal.store(success ? 1 : 2, std::memory_order_release);
    }
    return controller_interface::return_type::OK;
}

void Mfr3DuoGripperController::monitor() {
    // Action APIs, allocations and DDS stay on the executor, outside update().
    std::lock_guard<std::mutex> guard(monitor_mutex_);
    if (!monitored_) return;
    auto& operation = *monitored_;
    const int terminal = operation.terminal.load(std::memory_order_acquire);
    operation.report(operation, terminal);
    if (terminal == 0) return;
    monitored_.reset();
    busy_.store(false, std::memory_order_release);
}

}  // namespace mfr3duo_control
PLUGINLIB_EXPORT_CLASS(
    mfr3duo_control::Mfr3DuoGripperController, controller_interface::ControllerInterface)
