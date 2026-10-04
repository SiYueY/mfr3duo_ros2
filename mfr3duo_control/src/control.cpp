#include "mfr3duo_control/control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "mfr3duo_msgs/action/move.hpp"
#include "mfr3duo_msgs/action/grasp.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"

namespace mfr3duo_control {
namespace {
using Clock = std::chrono::steady_clock;
using TrajectoryAction = control_msgs::action::FollowJointTrajectory;
using GripperAction = control_msgs::action::GripperCommand;
using MoveAction = mfr3duo_msgs::action::Move;
using GraspAction = mfr3duo_msgs::action::Grasp;
using Controllers = controller_manager_msgs::srv::ListControllers;
Result error(ErrorCode code, const char* message) { return {code, message}; }
bool valid(Arm arm) { return arm == Arm::Left || arm == Arm::Right; }
bool valid(Gripper gripper) { return gripper == Gripper::Left || gripper == Gripper::Right; }
std::vector<std::string> arm_names(Arm arm) {
    std::vector<std::string> names;
    for (int i = 1; i <= 7; ++i)
        names.push_back(
            std::string(arm == Arm::Left ? "left" : "right") + "_fr3v2_1_joint" +
            std::to_string(i));
    return names;
}
constexpr std::array<const char*, 8> kControllers{
    "joint_state_broadcaster",  "imu_broadcaster",  "left_arm_controller",
    "right_arm_controller",     "spine_controller", "left_gripper_controller",
    "right_gripper_controller", "tmr_controller"};
struct Operation {
    std::mutex mutex;
    std::condition_variable changed;
    OperationState state{OperationState::WaitingForGoalResponse};
    bool response{false};
    bool terminal{false};
    bool unknown{false};
    bool cancel_sent{false};
    bool cancel_response{false};
    int reason{0};  // 1 local deadline, 2 explicit cancel
    Result result{ErrorCode::InternalError, "operation incomplete"};
    std::function<void()> send_cancel;
};
// Does not hold the operation mutex while calling ROS; goal identity stays with its record.
void request_cancel(const std::shared_ptr<Operation>& operation, int reason) {
    std::function<void()> send;
    {
        std::lock_guard<std::mutex> lock(operation->mutex);
        if (operation->terminal) return;
        if (operation->reason == 0) operation->reason = reason;
        if (operation->send_cancel && !operation->cancel_sent) {
            operation->cancel_sent = true;
            operation->state = OperationState::Canceling;
            send = operation->send_cancel;
        }
    }
    operation->changed.notify_all();
    if (send) send();
}
Result await_termination(const std::shared_ptr<Operation>& operation, Clock::time_point deadline) {
    std::unique_lock<std::mutex> lock(operation->mutex);
    if (!operation->changed.wait_until(lock, deadline, [&] { return operation->terminal; })) {
        operation->unknown = true;
        operation->state = OperationState::TerminationUnknown;
        operation->result = error(ErrorCode::CancelFailed, "terminal action result not confirmed");
    }
    return operation->result;
}
void terminal(const std::shared_ptr<Operation>& operation, Result result) {
    {
        std::lock_guard<std::mutex> lock(operation->mutex);
        if (operation->reason == 1 && result.code != ErrorCode::GoalRejected)
            result = error(
                ErrorCode::Timeout,
                "local execution deadline expired; remote termination confirmed");
        else if (operation->reason == 1 && result.code == ErrorCode::GoalRejected)
            result = error(ErrorCode::Timeout, "late goal rejection confirmed");
        operation->terminal = true;
        operation->unknown = false;
        operation->state = OperationState::Idle;
        operation->result = std::move(result);
    }
    operation->changed.notify_all();
}
}  // namespace

struct Control::Impl {
    struct State {
        std::mutex mutex;
        std::condition_variable changed;
        std::map<std::string, double> positions;
        std::map<std::string, urdf::JointLimits> limits;
        rclcpp::Time stamp;
        Clock::time_point received{};
        Clock::time_point controllers_received{};
        bool controllers_active{false};
        bool checking{false};
        Clock::time_point checking_since{};
        std::int64_t pending_request{-1};
        std::uint64_t generation{0};
    };
    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callbacks;
    std::array<rclcpp_action::Client<TrajectoryAction>::SharedPtr, 3> trajectories;
    std::array<rclcpp_action::Client<GripperAction>::SharedPtr, 2> grippers;
    std::array<rclcpp_action::Client<MoveAction>::SharedPtr, 2> gripper_moves;
    std::array<rclcpp_action::Client<GraspAction>::SharedPtr, 2> gripper_grasps;
    rclcpp::Client<Controllers>::SharedPtr controllers;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description;
    rclcpp::TimerBase::SharedPtr readiness_timer;
    std::shared_ptr<State> state{std::make_shared<State>()};
    mutable std::mutex mutex;
    std::array<std::shared_ptr<Operation>, 5> operations;
    bool initialized{false};
    bool busy{false};
    bool settings_valid{true};
    double response_timeout{2.0}, margin{2.0}, cancel_timeout{1.0}, terminal_timeout{2.0},
        gripper_timeout{10.0}, state_timeout{1.0};
    double max_linear{0.3}, max_angular{0.5};

    explicit Impl(rclcpp::Node::SharedPtr value) : node(std::move(value)) {
        if (!node) throw std::invalid_argument("Control requires a node");
        const auto parameter = [&](const char* name, double fallback) {
            if (!node->has_parameter(name)) node->declare_parameter<double>(name, fallback);
            const double value = node->get_parameter(name).as_double();
            if (!std::isfinite(value) || value <= 0.0) settings_valid = false;
            return value;
        };
        response_timeout = parameter("execution.goal_response_timeout", 2.0);
        margin = parameter("execution.timeout_margin", 2.0);
        cancel_timeout = parameter("execution.cancel_timeout", 1.0);
        terminal_timeout = parameter("execution.terminal_timeout", 2.0);
        gripper_timeout = parameter("execution.gripper_timeout", 10.0);
        state_timeout = parameter("control.state_timeout", 1.0);
        max_linear = parameter("control.max_linear_velocity", 0.3);
        max_angular = parameter("control.max_angular_velocity", 0.5);
        callbacks = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // Humble keeps raw callback-group guard pointers in the executor wait
        // set (rclcpp #2664). Retain only the group shell through Context shutdown;
        // clients, subscriptions and facade state still release normally here.
        // The application stops its executor before shutting down the Context.
        node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
            [group = callbacks]() mutable { group.reset(); });
        for (std::size_t i = 0; i < 3; ++i)
            trajectories[i] = rclcpp_action::create_client<TrajectoryAction>(
                node, std::string("/") + kControllers[i + 2] + "/follow_joint_trajectory",
                callbacks);
        for (std::size_t i = 0; i < 2; ++i)
            grippers[i] = rclcpp_action::create_client<GripperAction>(
                node, std::string("/") + kControllers[i + 5] + "/gripper_cmd", callbacks);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto prefix = std::string("/") + kControllers[i + 5];
            gripper_moves[i] =
                rclcpp_action::create_client<MoveAction>(node, prefix + "/move", callbacks);
            gripper_grasps[i] =
                rclcpp_action::create_client<GraspAction>(node, prefix + "/grasp", callbacks);
        }
        controllers = node->create_client<Controllers>(
            "/controller_manager/list_controllers", rmw_qos_profile_services_default, callbacks);
        velocity = node->create_publisher<geometry_msgs::msg::Twist>("/tmr_controller/cmd_vel", 1);
        rclcpp::SubscriptionOptions options;
        options.callback_group = callbacks;
        const std::weak_ptr<State> weak = state;
        auto controlled_names = arm_names(Arm::Left);
        const auto right_names = arm_names(Arm::Right);
        controlled_names.insert(controlled_names.end(), right_names.begin(), right_names.end());
        controlled_names.insert(
            controlled_names.end(), {"franka_spine_vertical_joint", "left_fr3v2_1_finger_joint1",
                                     "right_fr3v2_1_finger_joint1"});
        joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [weak, controlled_names](const sensor_msgs::msg::JointState& message) {
                auto state = weak.lock();
                if (!state || message.name.size() != message.position.size()) return;
                std::map<std::string, double> positions;
                for (std::size_t i = 0; i < message.name.size(); ++i) {
                    if (std::find(
                            controlled_names.begin(), controlled_names.end(), message.name[i]) ==
                        controlled_names.end())
                        continue;
                    if (!std::isfinite(message.position[i]) ||
                        !positions.emplace(message.name[i], message.position[i]).second)
                        return;
                }
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (positions.empty())
                        return;  // A passive-joint-only packet is not a command-device state
                                 // update.
                    state->positions = std::move(positions);
                    state->stamp = message.header.stamp;
                    state->received = Clock::now();
                }
                state->changed.notify_all();
            },
            options);
        description = node->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            [weak](const std_msgs::msg::String& message) {
                auto state = weak.lock();
                if (!state) return;
                urdf::Model model;
                if (!model.initString(message.data)) return;
                std::map<std::string, urdf::JointLimits> limits;
                for (const auto& item : model.joints_)
                    if (item.second->limits) limits.emplace(item.first, *item.second->limits);
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    state->limits = std::move(limits);
                }
                state->changed.notify_all();
            },
            options);
        const std::weak_ptr<rclcpp::Client<Controllers>> weak_client = controllers;
        readiness_timer = node->create_wall_timer(
            std::chrono::milliseconds(200),
            [weak, weak_client] {
                auto state = weak.lock();
                auto client = weak_client.lock();
                if (!state || !client || !client->service_is_ready()) return;
                std::int64_t expired = -1;
                std::uint64_t generation = 0;
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (state->checking &&
                        Clock::now() - state->checking_since < std::chrono::seconds(1))
                        return;
                    if (state->checking) expired = state->pending_request;
                    state->checking = true;
                    state->checking_since = Clock::now();
                    generation = ++state->generation;
                }
                if (expired >= 0) client->remove_pending_request(expired);
                try {
                    const auto request = client->async_send_request(
                        std::make_shared<Controllers::Request>(),
                        [weak, generation](rclcpp::Client<Controllers>::SharedFuture future) {
                            auto state = weak.lock();
                            if (!state) return;
                            bool active = true;
                            try {
                                const auto result = future.get();
                                for (const auto* name : kControllers)
                                    active &= std::any_of(
                                        result->controller.begin(), result->controller.end(),
                                        [&](const auto& controller) {
                                            return controller.name == name &&
                                                   controller.state == "active";
                                        });
                            } catch (...) {
                                active = false;
                            }
                            {
                                std::lock_guard<std::mutex> lock(state->mutex);
                                if (state->generation != generation) return;
                                state->checking = false;
                                state->controllers_active = active;
                                state->controllers_received = Clock::now();
                            }
                            state->changed.notify_all();
                        });
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (state->generation == generation)
                        state->pending_request = request.request_id;
                } catch (...) {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    state->checking = false;
                }
            },
            callbacks);
    }
    Clock::time_point after(double seconds) const {
        return Clock::now() +
               std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    }
    Result begin(std::size_t resource, const std::shared_ptr<Operation>& operation) {
        std::lock_guard<std::mutex> lock(mutex);
        if (operations[resource]) {
            std::lock_guard<std::mutex> operation_lock(operations[resource]->mutex);
            if (operations[resource]->unknown)
                return error(
                    ErrorCode::PreviousOperationNotTerminated, "previous goal termination unknown");
        }
        if (busy) return error(ErrorCode::Busy, "Control already executing an operation");
        if (!initialized) return error(ErrorCode::NotInitialized, "Control not initialized");
        busy = true;
        operations[resource] = operation;
        return {};
    }
    void release() {
        std::lock_guard<std::mutex> lock(mutex);
        busy = false;
    }
    bool dependencies() const {
        if (!settings_valid || !controllers->service_is_ready() ||
            velocity->get_subscription_count() == 0)
            return false;
        for (const auto& client : trajectories)
            if (!client->action_server_is_ready()) return false;
        for (const auto& client : grippers)
            if (!client->action_server_is_ready()) return false;
        for (const auto& client : gripper_moves)
            if (!client->action_server_is_ready()) return false;
        for (const auto& client : gripper_grasps)
            if (!client->action_server_is_ready()) return false;
        std::lock_guard<std::mutex> lock(state->mutex);
        if (!state->controllers_active ||
            Clock::now() - state->controllers_received > std::chrono::seconds(1) ||
            state->received == Clock::time_point{} ||
            Clock::now() - state->received > std::chrono::duration<double>(state_timeout))
            return false;
        for (const auto arm : {Arm::Left, Arm::Right})
            for (const auto& name : arm_names(arm))
                if (!state->positions.count(name) || !state->limits.count(name)) return false;
        for (const auto* name :
             {"franka_spine_vertical_joint", "left_fr3v2_1_finger_joint1",
              "right_fr3v2_1_finger_joint1"})
            if (!state->positions.count(name) || !state->limits.count(name)) return false;
        return true;
    }
    template <typename T>
    StateResult<T> positions(const std::vector<std::string>& names) const {
        StateResult<T> result;
        std::lock_guard<std::mutex> lock(state->mutex);
        result.stamp = state->stamp;
        for (const auto& name : names)
            if (!state->positions.count(name)) {
                result.result = error(ErrorCode::StateUnavailable, "joint state not received");
                return result;
            }
        if (Clock::now() - state->received > std::chrono::duration<double>(state_timeout)) {
            result.result = error(ErrorCode::StateStale, "joint state expired");
            return result;
        }
        if constexpr (std::is_same_v<T, double>)
            result.value = state->positions.at(names.front());
        else
            for (const auto& name : names) result.value.push_back(state->positions.at(name));
        return result;
    }
    Result validate(
        const trajectory_msgs::msg::JointTrajectory& trajectory,
        const std::vector<std::string>& names, double& duration) const {
        if (trajectory.joint_names != names || trajectory.points.empty())
            return error(ErrorCode::InvalidArgument, "trajectory joint names or points invalid");
        std::lock_guard<std::mutex> lock(state->mutex);
        double previous = 0.0;
        for (const auto& point : trajectory.points) {
            const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            if (point.time_from_start.nanosec >= 1000000000U || time <= previous ||
                !std::isfinite(time) || point.positions.size() != names.size() ||
                (!point.velocities.empty() && point.velocities.size() != names.size()) ||
                (!point.accelerations.empty() && point.accelerations.size() != names.size()) ||
                (!point.effort.empty() && point.effort.size() != names.size()))
                return error(ErrorCode::InvalidArgument, "invalid trajectory point");
            for (std::size_t i = 0; i < names.size(); ++i) {
                auto found = state->limits.find(names[i]);
                if (found == state->limits.end())
                    return error(ErrorCode::StateUnavailable, "URDF limits not received");
                if (!std::isfinite(point.positions[i]) ||
                    point.positions[i] < found->second.lower ||
                    point.positions[i] > found->second.upper)
                    return error(ErrorCode::InvalidArgument, "joint position outside URDF limit");
                if (!point.velocities.empty() &&
                    (!std::isfinite(point.velocities[i]) ||
                     std::abs(point.velocities[i]) > found->second.velocity))
                    return error(ErrorCode::InvalidArgument, "joint velocity outside URDF limit");
                if (!point.accelerations.empty() && !std::isfinite(point.accelerations[i]))
                    return error(ErrorCode::InvalidArgument, "nonfinite acceleration");
                if (!point.effort.empty() && (!std::isfinite(point.effort[i]) ||
                                              std::abs(point.effort[i]) > found->second.effort))
                    return error(ErrorCode::InvalidArgument, "joint effort outside URDF limit");
            }
            previous = time;
        }
        duration = previous;
        return {};
    }
    template <typename Action>
    Result execute(
        std::size_t resource, const std::shared_ptr<rclcpp_action::Client<Action>>& client,
        const typename Action::Goal& goal, double timeout) {
        auto operation = std::make_shared<Operation>();
        auto started = begin(resource, operation);
        if (!started) return started;
        struct Release {
            Impl& owner;
            ~Release() { owner.release(); }
        } release{*this};
        if (!dependencies() || !client->action_server_is_ready()) {
            const auto result =
                error(ErrorCode::NotReady, "Control dependencies unavailable or stale");
            terminal(operation, result);
            return result;
        }
        const std::weak_ptr<rclcpp_action::Client<Action>> weak_client = client;
        typename rclcpp_action::Client<Action>::SendGoalOptions options;
        options.goal_response_callback =
            [operation,
             weak_client](typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr handle) {
                if (!handle) {
                    terminal(operation, error(ErrorCode::GoalRejected, "action goal rejected"));
                    return;
                }
                const std::weak_ptr<Operation> weak_operation = operation;
                const std::weak_ptr<rclcpp_action::ClientGoalHandle<Action>> weak_handle = handle;
                const auto cancel = [weak_operation, weak_client, weak_handle] {
                    auto client = weak_client.lock();
                    auto operation = weak_operation.lock();
                    auto handle = weak_handle.lock();
                    if (!client || !operation || !handle) return;
                    try {
                        client->async_cancel_goal(handle, [operation](auto) {
                            {
                                std::lock_guard<std::mutex> lock(operation->mutex);
                                operation->cancel_response = true;
                            }
                            operation->changed.notify_all();
                        });
                    } catch (...) {
                        operation->changed.notify_all();
                    }
                };
                int reason = 0;
                {
                    std::lock_guard<std::mutex> lock(operation->mutex);
                    operation->response = true;
                    operation->send_cancel = cancel;
                    if (!operation->unknown) operation->state = OperationState::Active;
                    reason = operation->reason;
                }
                operation->changed.notify_all();
                if (reason != 0) request_cancel(operation, reason);
            };
        options.result_callback =
            [operation](
                const typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult& response) {
                Result result;
                if (response.code == rclcpp_action::ResultCode::CANCELED)
                    result = error(ErrorCode::Canceled, "action canceled");
                else if (response.code != rclcpp_action::ResultCode::SUCCEEDED || !response.result)
                    result = error(ErrorCode::ExecutionFailed, "action failed");
                else if constexpr (std::is_same_v<Action, TrajectoryAction>) {
                    if (response.result->error_code != TrajectoryAction::Result::SUCCESSFUL)
                        result = {ErrorCode::ExecutionFailed, response.result->error_string};
                }
                if constexpr (
                    std::is_same_v<Action, MoveAction> || std::is_same_v<Action, GraspAction>) {
                    if (response.result && response.code != rclcpp_action::ResultCode::CANCELED &&
                        !response.result->success)
                        result = {ErrorCode::ExecutionFailed, response.result->error};
                }
                terminal(operation, std::move(result));
            };
        const auto response_deadline = after(response_timeout);
        const auto execution_deadline = after(timeout);
        try {
            client->async_send_goal(goal, options);
        } catch (const std::exception& exception) {
            terminal(operation, {ErrorCode::ControllerUnavailable, exception.what()});
        }
        {
            std::unique_lock<std::mutex> lock(operation->mutex);
            if (!operation->changed.wait_until(
                    lock, std::min(response_deadline, execution_deadline), [&] {
                        return operation->response || operation->terminal || operation->reason != 0;
                    })) {
                lock.unlock();
                request_cancel(operation, 1);
                lock.lock();
            }
            if (operation->terminal) return operation->result;
            if (operation->reason == 0 &&
                !operation->changed.wait_until(lock, execution_deadline, [&] {
                    return operation->terminal || operation->reason != 0;
                })) {
                lock.unlock();
                request_cancel(operation, 1);
                lock.lock();
            }
            if (operation->terminal) return operation->result;
        }
        return await_canceled(operation);
    }
    Result await_canceled(const std::shared_ptr<Operation>& operation) {
        const auto response_deadline = after(cancel_timeout);
        {
            std::unique_lock<std::mutex> lock(operation->mutex);
            operation->changed.wait_until(lock, response_deadline, [&] {
                return operation->terminal || operation->cancel_response;
            });
            if (operation->terminal) return operation->result;
        }
        return await_termination(operation, after(terminal_timeout));
    }
    Result stop(std::size_t resource) {
        std::shared_ptr<Operation> operation;
        {
            std::lock_guard<std::mutex> lock(mutex);
            operation = operations[resource];
        }
        if (!operation) return {};
        request_cancel(operation, 2);
        return await_canceled(operation);
    }
};

Control::Control(const rclcpp::Node::SharedPtr& node) : impl_(std::make_unique<Impl>(node)) {}
Control::~Control() {
    try {
        const auto deadline = impl_->after(impl_->cancel_timeout + impl_->terminal_timeout);
        for (const auto& operation : impl_->operations)
            if (operation) request_cancel(operation, 2);
        for (const auto& operation : impl_->operations)
            if (operation) await_termination(operation, deadline);
    } catch (...) {
    }
}
Result Control::initialize(std::chrono::milliseconds timeout) {
    if (timeout.count() <= 0 || !impl_->settings_valid)
        return error(ErrorCode::InvalidArgument, "invalid timeout configuration");
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->busy) return error(ErrorCode::Busy, "Control executing");
        for (const auto& operation : impl_->operations)
            if (operation) {
                std::lock_guard<std::mutex> lock(operation->mutex);
                if (operation->unknown)
                    return error(
                        ErrorCode::PreviousOperationNotTerminated,
                        "previous goal termination unknown");
            }
        impl_->busy = true;
    }
    struct Release {
        Impl& owner;
        ~Release() { owner.release(); }
    } release{*impl_};
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        if (!impl_->node->get_node_base_interface()->get_context()->is_valid())
            return error(
                ErrorCode::NotReady, "ROS context shut down during Control initialization");
        if (impl_->dependencies()) {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->initialized = true;
            return {};
        }
        std::unique_lock<std::mutex> lock(impl_->state->mutex);
        impl_->state->changed.wait_until(
            lock, std::min(deadline, Clock::now() + std::chrono::milliseconds(50)));
    }
    return error(ErrorCode::Timeout, "Control initialization deadline expired");
}
bool Control::is_ready() const {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->initialized) return false;
        for (const auto& operation : impl_->operations)
            if (operation) {
                std::lock_guard<std::mutex> lock(operation->mutex);
                if (operation->unknown) return false;
            }
    }
    return impl_->dependencies();
}
Result Control::command_arm_joint_position(
    Arm arm, const std::vector<double>& positions, std::chrono::milliseconds duration) {
    if (!valid(arm) || duration.count() <= 0 || duration.count() / 1000 > INT32_MAX)
        return error(ErrorCode::InvalidArgument, "invalid arm or duration");
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = arm_names(arm);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start.sec = static_cast<int32_t>(duration.count() / 1000);
    point.time_from_start.nanosec = static_cast<uint32_t>(duration.count() % 1000 * 1000000);
    trajectory.points.push_back(point);
    return execute_arm_trajectory(arm, trajectory);
}
Result Control::execute_arm_trajectory(
    Arm arm, const trajectory_msgs::msg::JointTrajectory& trajectory) {
    if (!valid(arm)) return error(ErrorCode::InvalidArgument, "invalid arm");
    double duration = 0.0;
    const auto validation = impl_->validate(trajectory, arm_names(arm), duration);
    if (!validation) return validation;
    TrajectoryAction::Goal goal;
    goal.trajectory = trajectory;
    const auto index = arm == Arm::Left ? 0U : 1U;
    return impl_->execute(index, impl_->trajectories[index], goal, duration + impl_->margin);
}
Result Control::stop_arm(Arm arm) {
    return valid(arm) ? impl_->stop(arm == Arm::Left ? 0 : 1)
                      : error(ErrorCode::InvalidArgument, "invalid arm");
}
Result Control::command_spine_position(double position, std::chrono::milliseconds duration) {
    if (duration.count() <= 0 || duration.count() / 1000 > INT32_MAX)
        return error(ErrorCode::InvalidArgument, "invalid duration");
    TrajectoryAction::Goal goal;
    goal.trajectory.joint_names = {"franka_spine_vertical_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position};
    point.time_from_start.sec = static_cast<int32_t>(duration.count() / 1000);
    point.time_from_start.nanosec = static_cast<uint32_t>(duration.count() % 1000 * 1000000);
    goal.trajectory.points.push_back(point);
    double time = 0.0;
    const auto validation = impl_->validate(goal.trajectory, goal.trajectory.joint_names, time);
    if (!validation) return validation;
    return impl_->execute(2, impl_->trajectories[2], goal, time + impl_->margin);
}
Result Control::stop_spine() { return impl_->stop(2); }
Result Control::command_gripper(Gripper gripper, double position, std::optional<double> effort) {
    if (!valid(gripper) || !std::isfinite(position) || position < 0.0 || position > 0.04 ||
        (effort && (!std::isfinite(*effort) || *effort <= 0.0 || *effort > 100.0)))
        return error(ErrorCode::InvalidArgument, "invalid finger position or effort");
    GripperAction::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = effort.value_or(0.0);
    const auto index = gripper == Gripper::Left ? 0U : 1U;
    return impl_->execute(3 + index, impl_->grippers[index], goal, impl_->gripper_timeout);
}
Result Control::move_gripper(Gripper gripper, double width, double speed) {
    if (!valid(gripper) || !std::isfinite(width) || width < 0 || width > .08 ||
        !std::isfinite(speed) || speed <= 0 || speed > .20)
        return error(ErrorCode::InvalidArgument, "invalid opening width or speed");
    MoveAction::Goal goal;
    goal.width = width;
    goal.speed = speed;
    const auto index = gripper == Gripper::Left ? 0U : 1U;
    return impl_->execute(3 + index, impl_->gripper_moves[index], goal, impl_->gripper_timeout);
}
Result Control::grasp_gripper(
    Gripper gripper, double width, double speed, double force, double inner, double outer) {
    if (!valid(gripper) || !std::isfinite(width) || width < 0 || width > .08 ||
        !std::isfinite(speed) || speed <= 0 || speed > .20 || !std::isfinite(force) || force <= 0 ||
        force > 100 || !std::isfinite(inner) || inner < 0 || !std::isfinite(outer) || outer < 0)
        return error(ErrorCode::InvalidArgument, "invalid grasp parameters");
    GraspAction::Goal goal;
    goal.width = width;
    goal.speed = speed;
    goal.force = force;
    goal.epsilon.inner = inner;
    goal.epsilon.outer = outer;
    const auto index = gripper == Gripper::Left ? 0U : 1U;
    return impl_->execute(3 + index, impl_->gripper_grasps[index], goal, impl_->gripper_timeout);
}
StateResult<double> Control::get_gripper_width(Gripper gripper) const {
    auto result = get_gripper_position(gripper);
    result.value *= 2;
    return result;
}
Result Control::stop_gripper(Gripper gripper) {
    return valid(gripper) ? impl_->stop(gripper == Gripper::Left ? 3 : 4)
                          : error(ErrorCode::InvalidArgument, "invalid gripper");
}
Result Control::command_base_velocity(const BaseVelocity& input) {
    if (!std::isfinite(input.linear_x) || !std::isfinite(input.linear_y) ||
        !std::isfinite(input.angular_z) ||
        std::hypot(input.linear_x, input.linear_y) > impl_->max_linear ||
        std::abs(input.angular_z) > impl_->max_angular)
        return error(ErrorCode::InvalidArgument, "invalid base velocity");
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->busy) return error(ErrorCode::Busy, "Control executing");
        if (!impl_->initialized) return error(ErrorCode::NotInitialized, "Control not initialized");
        impl_->busy = true;
    }
    struct Release {
        Impl& owner;
        ~Release() { owner.release(); }
    } release{*impl_};
    if (!impl_->dependencies())
        return error(ErrorCode::NotReady, "Control dependencies unavailable or stale");
    geometry_msgs::msg::Twist message;
    message.linear.x = input.linear_x;
    message.linear.y = input.linear_y;
    message.angular.z = input.angular_z;
    try {
        impl_->velocity->publish(message);
    } catch (const std::exception& exception) {
        return {ErrorCode::InternalError, exception.what()};
    }
    return {};
}
Result Control::stop_base() {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->initialized) return error(ErrorCode::NotInitialized, "Control not initialized");
    }
    // stop is allowed during another owned action; it does not cancel that action.
    try {
        impl_->velocity->publish(geometry_msgs::msg::Twist{});
    } catch (const std::exception& exception) {
        return {ErrorCode::InternalError, exception.what()};
    }
    return {};
}
StateResult<std::vector<double>> Control::get_arm_joint_positions(Arm arm) const {
    if (!valid(arm)) return {error(ErrorCode::InvalidArgument, "invalid arm"), {}, rclcpp::Time{}};
    return impl_->positions<std::vector<double>>(arm_names(arm));
}
StateResult<double> Control::get_spine_position() const {
    return impl_->positions<double>({"franka_spine_vertical_joint"});
}
StateResult<double> Control::get_gripper_position(Gripper gripper) const {
    if (!valid(gripper))
        return {error(ErrorCode::InvalidArgument, "invalid gripper"), {}, rclcpp::Time{}};
    return impl_->positions<double>(
        {gripper == Gripper::Left ? "left_fr3v2_1_finger_joint1" : "right_fr3v2_1_finger_joint1"});
}
}  // namespace mfr3duo_control
