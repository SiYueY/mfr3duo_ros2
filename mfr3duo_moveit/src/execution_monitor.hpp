#pragma once

#include <action_msgs/msg/goal_status_array.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <vector>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <set>

// Private application-side execution fence. Uses only standard ROS interfaces;
// it neither subclasses nor copies a MoveIt capability/controller plugin.
namespace mfr3duo_moveit {
namespace {
using Clock = std::chrono::steady_clock;
using Action = moveit_msgs::action::ExecuteTrajectory;
using Client = rclcpp_action::Client<Action>;
using Fjt = control_msgs::action::FollowJointTrajectory;
using GetChildResult = Fjt::Impl::GetResultService;
using UUID = rclcpp_action::GoalUUID;
struct JointSample {
    double position{0}, velocity{0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    Clock::time_point received{};
    bool velocity_known{false};
};
struct MeasuredState {
    std::mutex mutex;
    std::set<std::string> required;
    std::map<std::string, JointSample> joints;
    bool controllers_ready{false}, checking{false};
    Clock::time_point controllers_checked{}, checking_since{};
    std::int64_t pending{-1};
    unsigned generation{0};
};
struct ExecutionMonitor;
struct Operation {
    std::mutex mutex;
    std::condition_variable changed;
    OperationState state{OperationState::WaitingForGoalResponse};
    bool response{false}, terminal{false}, unknown{false}, cancel_sent{false};
    bool parent_done{false}, sent{false};
    int reason{0};
    Result result{ErrorCode::InternalError, "operation incomplete"};
    Result parent_result;
    std::function<void()> send_cancel;
    Client::SharedPtr retained_client;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr retained_states;
    std::shared_ptr<MeasuredState> measured;
    std::shared_ptr<ExecutionMonitor> monitor;
    std::array<bool, 3> expected{};
    std::array<std::set<UUID>, 3> baseline;
    std::vector<std::string> joints;
    std::int64_t dispatch_stamp{0}, stop_stamp{0};
    rclcpp::Clock::SharedPtr ros_clock;
    Clock::time_point stop_requested{}, stopped_since{}, last_stop{};
};
void request_cancel(const std::shared_ptr<Operation>& op, int reason) {
    std::function<void()> send;
    {
        std::lock_guard<std::mutex> lock(op->mutex);
        if (op->terminal) return;
        if (!op->reason) {
            op->reason = reason;
            op->stop_requested = Clock::now();
            op->stop_stamp = op->ros_clock->now().nanoseconds();
        }
        if (op->send_cancel && !op->cancel_sent) {
            op->cancel_sent = true;
            op->state = OperationState::Canceling;
            send = op->send_cancel;
        }
    }
    op->changed.notify_all();
    if (send) send();
}
bool terminal(const std::shared_ptr<Operation>& op, Result result, int expected_reason = -1) {
    Client::SharedPtr client;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr states;
    std::shared_ptr<ExecutionMonitor> monitor;
    {
        std::lock_guard<std::mutex> lock(op->mutex);
        if (op->terminal) return true;
        if (expected_reason >= 0 && op->reason != expected_reason) return false;
        if (op->reason == 1)
            result = {
                ErrorCode::Timeout, "local deadline expired; execution termination confirmed"};
        client.swap(op->retained_client);
        states.swap(op->retained_states);
        monitor.swap(op->monitor);
        op->result = std::move(result);
        op->terminal = true;
        op->unknown = false;
        op->state = OperationState::Idle;
        op->send_cancel = {};
    }
    op->changed.notify_all();
    return true;
}
Result await_termination(const std::shared_ptr<Operation>& op, Clock::time_point deadline) {
    std::unique_lock<std::mutex> lock(op->mutex);
    if (!op->changed.wait_until(lock, deadline, [&] { return op->terminal; })) {
        op->unknown = true;
        op->state = OperationState::TerminationUnknown;
        op->result = {
            ErrorCode::CancelFailed,
            "parent/child terminal results or measured stop not confirmed"};
    }
    return op->result;
}
struct ExecutionMonitor : std::enable_shared_from_this<ExecutionMonitor> {
    struct Child {
        std::int8_t status{0};
        std::int64_t stamp{0};
        bool requested{false}, done{false};
        std::int32_t error_code{0};
    };
    std::mutex mutex;
    std::array<std::map<UUID, Child>, 3> children;
    std::shared_ptr<Operation> active;
    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callbacks;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stop;
    std::array<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr, 3> statuses;
    std::array<rclcpp::Client<GetChildResult>::SharedPtr, 3> results;
    rclcpp::TimerBase::SharedPtr timer;
    explicit ExecutionMonitor(rclcpp::Node::SharedPtr n) : node(std::move(n)) {}
    void init() {
        callbacks = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
            [group = callbacks]() mutable { group.reset(); });
        stop = node->create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 10);
        const std::weak_ptr<ExecutionMonitor> weak = shared_from_this();
        const std::array<const char*, 3> names{
            "left_arm_controller", "right_arm_controller", "spine_controller"};
        rclcpp::SubscriptionOptions options;
        options.callback_group = callbacks;
        for (std::size_t i = 0; i < 3; ++i) {
            const auto base = std::string("/") + names[i] + "/follow_joint_trajectory/_action/";
            results[i] = node->create_client<GetChildResult>(
                base + "get_result", rmw_qos_profile_services_default, callbacks);
            statuses[i] = node->create_subscription<action_msgs::msg::GoalStatusArray>(
                base + "status", rclcpp::QoS(10).reliable().transient_local(),
                [weak, i](const action_msgs::msg::GoalStatusArray& message) {
                    if (auto owner = weak.lock()) {
                        std::lock_guard<std::mutex> lock(owner->mutex);
                        for (const auto& s : message.status_list) {
                            auto& c = owner->children[i][s.goal_info.goal_id.uuid];
                            c.status = s.status;
                            c.stamp =
                                static_cast<std::int64_t>(s.goal_info.stamp.sec) * 1000000000LL +
                                s.goal_info.stamp.nanosec;
                        }
                        // Keep all current status entries and a bounded recent result cache.
                        if (!owner->active && owner->children[i].size() > 512) {
                            for (auto it = owner->children[i].begin();
                                 it != owner->children[i].end() &&
                                 owner->children[i].size() > 256;) {
                                if (it->second.status >= 4)
                                    it = owner->children[i].erase(it);
                                else
                                    ++it;
                            }
                        }
                    }
                },
                options);
        }
        timer = node->create_wall_timer(
            std::chrono::milliseconds(20),
            [weak] {
                if (auto owner = weak.lock()) owner->poll();
            },
            callbacks);
    }
    bool available() {
        std::lock_guard<std::mutex> lock(mutex);
        if (active) {
            std::lock_guard<std::mutex> guard(active->mutex);
            if (!active->terminal) return false;
        }
        for (const auto& controller : children)
            for (const auto& entry : controller)
                if (entry.second.status > 0 && entry.second.status < 4) return false;
        return true;
    }
    Result claim(const std::shared_ptr<Operation>& op) {
        std::lock_guard<std::mutex> lock(mutex);
        if (auto previous = active) {
            std::lock_guard<std::mutex> guard(previous->mutex);
            if (!previous->terminal)
                return {
                    previous->unknown ? ErrorCode::PreviousOperationNotTerminated : ErrorCode::Busy,
                    "shared MoveGroup execution channel occupied"};
        }
        for (const auto& controller : children)
            for (const auto& entry : controller)
                if (entry.second.status > 0 && entry.second.status < 4)
                    return {
                        ErrorCode::Busy,
                        "controller already active; exclusive execution channel required"};
        for (std::size_t i = 0; i < 3; ++i)
            for (const auto& entry : children[i]) op->baseline[i].insert(entry.first);
        op->monitor = shared_from_this();
        active = op;
        return {};
    }
    void poll() {
        std::shared_ptr<Operation> op;
        {
            std::lock_guard<std::mutex> lock(mutex);
            op = active;
        }
        if (!op) return;
        bool parent_done, sent;
        int reason;
        std::array<bool, 3> expected;
        std::int64_t dispatch;
        Result result;
        {
            std::lock_guard<std::mutex> lock(op->mutex);
            if (op->terminal) return;
            parent_done = op->parent_done;
            reason = op->reason;
            sent = op->sent;
            expected = op->expected;
            dispatch = op->dispatch_stamp;
            result = op->parent_result;
        }
        if (!sent) return;
        bool all = true;
        struct Query {
            std::size_t index;
            UUID uuid;
        };
        std::vector<Query> query;
        {
            std::lock_guard<std::mutex> lock(mutex);
            for (std::size_t i = 0; i < 3; ++i)
                if (expected[i]) {
                    bool observed = false;
                    for (auto& entry : children[i]) {
                        if (op->baseline[i].count(entry.first) || entry.second.stamp < dispatch)
                            continue;
                        observed = true;
                        auto& child = entry.second;
                        if (!child.requested && results[i]->service_is_ready()) {
                            child.requested = true;
                            query.push_back({i, entry.first});
                        }
                        all = all && child.done;
                        if (child.done && reason == 0 &&
                            (child.error_code != Fjt::Result::SUCCESSFUL || child.status != 4))
                            result = {ErrorCode::ExecutionFailed, "controller reported failure"};
                    }
                    all = all && observed;
                }
        }
        for (const auto& q : query) {
            auto request = std::make_shared<GetChildResult::Request>();
            request->goal_id.uuid = q.uuid;
            const std::weak_ptr<ExecutionMonitor> weak = shared_from_this();
            try {
                results[q.index]->async_send_request(
                    request, [weak, q](rclcpp::Client<GetChildResult>::SharedFuture future) {
                        if (auto owner = weak.lock()) {
                            const auto response = future.get();
                            std::lock_guard<std::mutex> lock(owner->mutex);
                            auto& child = owner->children[q.index][q.uuid];
                            child.done = response->status >= 4 && response->status <= 6;
                            child.status = response->status;
                            child.error_code = response->result.error_code;
                        }
                    });
            } catch (const std::exception&) {
                std::lock_guard<std::mutex> lock(mutex);
                children[q.index][q.uuid].requested = false;
            }
        }
        // Stop is manager-wide. Repeat until the execution is fenced: a stop
        // published before a late Accepted/child dispatch must not be lost.
        if (reason && (!parent_done || !all)) {
            bool publish = false;
            {
                std::lock_guard<std::mutex> lock(op->mutex);
                if (!op->terminal &&
                    Clock::now() - op->last_stop >= std::chrono::milliseconds(50)) {
                    op->last_stop = Clock::now();
                    publish = true;
                }
            }
            if (publish) {
                std_msgs::msg::String event;
                event.data = "stop";
                stop->publish(event);
            }
        }
        if (!parent_done || !all) return;
        if (reason) {
            bool stationary = true;
            const auto now = Clock::now();
            {
                std::lock_guard<std::mutex> lock(op->measured->mutex);
                for (const auto& name : op->joints) {
                    const auto found = op->measured->joints.find(name);
                    if (found == op->measured->joints.end() || !found->second.velocity_known ||
                        found->second.received <= op->stop_requested ||
                        found->second.stamp.nanoseconds() < op->stop_stamp ||
                        op->ros_clock->now().nanoseconds() - found->second.stamp.nanoseconds() >
                            300000000LL ||
                        now - found->second.received > std::chrono::milliseconds(300) ||
                        std::abs(found->second.velocity) >
                            (name == "franka_spine_vertical_joint" ? .003 : .02))
                        stationary = false;
                }
            }
            std::lock_guard<std::mutex> lock(op->mutex);
            if (!stationary)
                op->stopped_since = {};
            else if (op->stopped_since == Clock::time_point{})
                op->stopped_since = now;
            if (!stationary || now - op->stopped_since < std::chrono::milliseconds(100)) return;
            if (reason == 2)
                result = {
                    ErrorCode::Canceled,
                    "official stop; all controller results and measured stop confirmed"};
        }
        if (terminal(op, std::move(result), reason)) {
            std::lock_guard<std::mutex> lock(mutex);
            if (active == op) active.reset();
        }
    }
};
std::shared_ptr<ExecutionMonitor> execution_monitor(const rclcpp::Node::SharedPtr& node) {
    static std::mutex registry_mutex;
    static std::map<rclcpp::Context*, std::weak_ptr<ExecutionMonitor>> registry;
    std::lock_guard<std::mutex> lock(registry_mutex);
    const auto context = node->get_node_base_interface()->get_context();
    auto& weak = registry[context.get()];
    if (auto monitor = weak.lock()) return monitor;
    auto monitor = std::make_shared<ExecutionMonitor>(node);
    monitor->init();
    weak = monitor;
    return monitor;
}
}  // namespace
}  // namespace mfr3duo_moveit
