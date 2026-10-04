#include "mfr3duo_nav/navigator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace mfr3duo_nav {
namespace {
using Clock = std::chrono::steady_clock;
using To = nav2_msgs::action::NavigateToPose;
using Through = nav2_msgs::action::NavigateThroughPoses;
using GetState = lifecycle_msgs::srv::GetState;
Clock::time_point after(double seconds) {
    return Clock::now() +
           std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}
bool valid_pose(const geometry_msgs::msg::PoseStamped& pose) {
    const auto& p = pose.pose.position;
    const auto& q = pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return !pose.header.frame_id.empty() && pose.header.stamp.sec >= 0 &&
           pose.header.stamp.nanosec < 1000000000U && std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.z) && std::isfinite(norm) && std::abs(norm - 1.) < 1e-6;
}
struct Operation {
    std::mutex mutex;
    std::condition_variable changed;
    NavigationState state{NavigationState::Running};
    bool response{false}, terminal{false}, unknown{false}, cancel_sent{false};
    int cancel_reason{0};
    Result result{ErrorCode::InternalError, "operation incomplete"};
    Clock::time_point response_deadline, deadline, terminal_deadline;
    double terminal_timeout{4.};
    std::function<void()> send_cancel;
    std::shared_ptr<void> retained_client;
};
void finish(const std::shared_ptr<Operation>& op, Result result) {
    std::shared_ptr<void> release;
    {
        std::lock_guard<std::mutex> lock(op->mutex);
        if (op->terminal) return;
        if (op->cancel_reason == 2)
            result = {ErrorCode::Timeout, "local deadline expired; terminal result confirmed"};
        op->result = std::move(result);
        op->terminal = true;
        op->unknown = false;
        op->state = op->result.code == ErrorCode::Success
                        ? NavigationState::Succeeded
                        : (op->result.code == ErrorCode::Canceled ? NavigationState::Canceled
                                                                  : NavigationState::Failed);
        release.swap(op->retained_client);
    }
    op->changed.notify_all();
}
void request_cancel(const std::shared_ptr<Operation>& op, int reason) {
    std::function<void()> send;
    {
        std::lock_guard<std::mutex> lock(op->mutex);
        if (op->terminal) return;
        if (!op->cancel_reason) {
            op->cancel_reason = reason;
            op->terminal_deadline = after(op->terminal_timeout);
        }
        if (!op->unknown) op->state = NavigationState::Canceling;
        if (op->send_cancel && !op->cancel_sent) {
            op->cancel_sent = true;
            send = op->send_cancel;
        }
    }
    op->changed.notify_all();
    if (send) send();
}
void progress(const std::shared_ptr<Operation>& op) {
    bool expired;
    {
        std::lock_guard<std::mutex> lock(op->mutex);
        if (op->terminal) return;
        const auto now = Clock::now();
        expired = !op->cancel_reason &&
                  (now >= op->deadline || (!op->response && now >= op->response_deadline));
        if (op->cancel_reason && now >= op->terminal_deadline) {
            op->unknown = true;
            op->state = NavigationState::TerminationUnknown;
            op->result = {ErrorCode::CancelFailed, "owned goal terminal result not confirmed"};
            op->changed.notify_all();
        }
    }
    if (expired) request_cancel(op, 2);
}
Result await(const std::shared_ptr<Operation>& op) {
    std::unique_lock<std::mutex> lock(op->mutex);
    while (!op->terminal && !op->unknown) {
        op->changed.wait_for(lock, std::chrono::milliseconds(10));
        lock.unlock();
        progress(op);
        lock.lock();
    }
    return op->result;
}
struct Readiness {
    std::mutex mutex;
    bool map{false};
    Clock::time_point odom_received{};
    rclcpp::Time odom_stamp{0, 0, RCL_ROS_TIME};
    std::array<bool, 8> active{}, pending{};
    std::array<Clock::time_point, 8> received{}, requested{};
    std::array<std::uint64_t, 8> generation{};
};
constexpr std::array<const char*, 8> kLifecycle{
    "map_server",      "amcl",         "controller_server", "planner_server", "smoother_server",
    "behavior_server", "bt_navigator", "velocity_smoother"};
}  // namespace

struct NavigationHandle::Impl {
    std::shared_ptr<Operation> operation;
};
struct Navigator::Impl {
    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callbacks;
    rclcpp_action::Client<To>::SharedPtr to;
    rclcpp_action::Client<Through>::SharedPtr through;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<Readiness> readiness{std::make_shared<Readiness>()};
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf, static_tf;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom;
    std::array<rclcpp::Client<GetState>::SharedPtr, 8> lifecycle;
    rclcpp::TimerBase::SharedPtr operation_timer;
    rclcpp::TimerBase::SharedPtr readiness_timer;
    mutable std::mutex mutex;
    bool initialized{false};
    std::shared_ptr<Operation> operation;
    std::string map_frame, base_frame;
    double response_timeout, navigation_timeout, terminal_timeout, freshness;

    explicit Impl(rclcpp::Node::SharedPtr supplied) : node(std::move(supplied)) {
        if (!node) throw std::invalid_argument("Navigator requires an external node");
        auto text = [&](const char* name, const char* fallback) {
            if (!node->has_parameter(name)) node->declare_parameter(name, fallback);
            return node->get_parameter(name).as_string();
        };
        auto seconds = [&](const char* name, double fallback) {
            if (!node->has_parameter(name)) node->declare_parameter(name, fallback);
            const double value = node->get_parameter(name).as_double();
            if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name);
            return value;
        };
        map_frame = text("navigator.map_frame", "map");
        base_frame = text("navigator.base_frame", "base_link");
        if (map_frame.empty() || base_frame.empty() || map_frame == base_frame)
            throw std::invalid_argument("invalid navigation frames");
        response_timeout = seconds("navigator.goal_response_timeout", 2.);
        navigation_timeout = seconds("navigator.navigation_timeout", 120.);
        terminal_timeout = seconds("navigator.terminal_timeout", 4.);
        freshness = seconds("navigator.state_timeout", 1.);
        callbacks = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
            [group = callbacks]() mutable { group.reset(); });
        to = rclcpp_action::create_client<To>(
            node, text("navigator.navigate_to_action", "navigate_to_pose"), callbacks);
        through = rclcpp_action::create_client<Through>(
            node, text("navigator.navigate_through_action", "navigate_through_poses"), callbacks);
        initial = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            text("navigator.initial_pose_topic", "initialpose"), 10);
        buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        rclcpp::SubscriptionOptions options;
        options.callback_group = callbacks;
        const std::weak_ptr<tf2_ros::Buffer> weak_buffer = buffer;
        auto receive = [weak_buffer](const tf2_msgs::msg::TFMessage& message, bool fixed) {
            if (auto owned = weak_buffer.lock())
                for (const auto& transform : message.transforms) try {
                        owned->setTransform(transform, "mfr3duo_nav", fixed);
                    } catch (const tf2::TransformException&) {
                    }
        };
        tf = node->create_subscription<tf2_msgs::msg::TFMessage>(
            "tf", 100, [receive](const tf2_msgs::msg::TFMessage& msg) { receive(msg, false); },
            options);
        static_tf = node->create_subscription<tf2_msgs::msg::TFMessage>(
            "tf_static", rclcpp::QoS(100).transient_local(),
            [receive](const tf2_msgs::msg::TFMessage& msg) { receive(msg, true); }, options);
        const std::weak_ptr<Readiness> weak = readiness;
        map = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            text("navigator.map_topic", "map"), rclcpp::QoS(1).transient_local(),
            [weak, frame = map_frame](const nav_msgs::msg::OccupancyGrid& msg) {
                if (auto cache = weak.lock()) {
                    geometry_msgs::msg::PoseStamped origin;
                    origin.header = msg.header;
                    origin.pose = msg.info.origin;
                    std::lock_guard<std::mutex> lock(cache->mutex);
                    cache->map = msg.header.frame_id == frame && msg.info.width &&
                                 msg.info.height && std::isfinite(msg.info.resolution) &&
                                 msg.info.resolution > 0 && valid_pose(origin) &&
                                 msg.data.size() ==
                                     static_cast<std::size_t>(msg.info.width) * msg.info.height;
                }
            },
            options);
        odom = node->create_subscription<nav_msgs::msg::Odometry>(
            text("navigator.odom_topic", "tmr_controller/odom"), 10,
            [weak, frame = base_frame](const nav_msgs::msg::Odometry& msg) {
                if (auto cache = weak.lock()) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = msg.header;
                    pose.pose = msg.pose.pose;
                    if (msg.child_frame_id != frame || !valid_pose(pose) ||
                        !std::isfinite(msg.twist.twist.linear.x) ||
                        !std::isfinite(msg.twist.twist.linear.y) ||
                        !std::isfinite(msg.twist.twist.angular.z) ||
                        !std::isfinite(msg.pose.covariance[0]) ||
                        !std::isfinite(msg.pose.covariance[7]) ||
                        !std::isfinite(msg.pose.covariance[35]) || msg.pose.covariance[0] < 0 ||
                        msg.pose.covariance[7] < 0 || msg.pose.covariance[35] < 0 ||
                        msg.pose.covariance[0] >= 1e6 || msg.pose.covariance[7] >= 1e6 ||
                        msg.pose.covariance[35] >= 1e6)
                        return;
                    std::lock_guard<std::mutex> lock(cache->mutex);
                    cache->odom_stamp = rclcpp::Time(msg.header.stamp, RCL_ROS_TIME);
                    cache->odom_received = Clock::now();
                }
            },
            options);
        for (std::size_t i = 0; i < lifecycle.size(); ++i)
            lifecycle[i] = node->create_client<GetState>(
                std::string(kLifecycle[i]) + "/get_state", rmw_qos_profile_services_default,
                callbacks);
        readiness_timer = node->create_wall_timer(
            std::chrono::milliseconds(200),
            [weak, clients = lifecycle] {
                const auto cache = weak.lock();
                if (!cache) return;
                const auto now = Clock::now();
                std::lock_guard<std::mutex> lock(cache->mutex);
                for (std::size_t i = 0; i < clients.size(); ++i) {
                    if (cache->pending[i] && now - cache->requested[i] > std::chrono::seconds(2)) {
                        clients[i]->prune_pending_requests();
                        cache->pending[i] = false;
                        ++cache->generation[i];
                    }
                    if (cache->pending[i] || !clients[i]->service_is_ready() ||
                        now - cache->requested[i] < std::chrono::milliseconds(250))
                        continue;
                    cache->pending[i] = true;
                    cache->requested[i] = now;
                    const auto generation = ++cache->generation[i];
                    clients[i]->async_send_request(
                        std::make_shared<GetState::Request>(),
                        [weak, i, generation](rclcpp::Client<GetState>::SharedFuture future) {
                            if (const auto held = weak.lock()) {
                                std::lock_guard<std::mutex> lock(held->mutex);
                                if (held->generation[i] != generation) return;
                                held->active[i] = future.get()->current_state.id == 3;
                                held->pending[i] = false;
                                held->received[i] = Clock::now();
                            }
                        });
                }
            },
            callbacks);
    }
    bool ready() const {
        const auto now = Clock::now();
        bool active = true, measured = false;
        {
            std::lock_guard<std::mutex> lock(readiness->mutex);
            measured = readiness->map &&
                       std::chrono::duration<double>(now - readiness->odom_received).count() <=
                           freshness &&
                       std::abs((node->now() - readiness->odom_stamp).seconds()) <= freshness;
            for (std::size_t i = 0; i < lifecycle.size(); ++i) {
                active = active && readiness->active[i] &&
                         std::chrono::duration<double>(now - readiness->received[i]).count() <=
                             freshness;
            }
        }
        if (!measured || !active || !to->action_server_is_ready() ||
            !through->action_server_is_ready())
            return false;
        try {
            const auto transform =
                buffer->lookupTransform(map_frame, base_frame, tf2::TimePointZero);
            return std::abs((node->now() - rclcpp::Time(transform.header.stamp, RCL_ROS_TIME))
                                .seconds()) <= freshness;
        } catch (const tf2::TransformException&) {
            return false;
        }
    }
    std::shared_ptr<Operation> reserve(
        const std::vector<geometry_msgs::msg::PoseStamped>& targets) {
        auto op = std::make_shared<Operation>();
        op->response_deadline = after(response_timeout);
        op->deadline = after(navigation_timeout);
        op->terminal_timeout = terminal_timeout;
        std::lock_guard<std::mutex> lock(mutex);
        Result error;
        if (operation) {
            std::lock_guard<std::mutex> held(operation->mutex);
            if (!operation->terminal)
                error = {
                    operation->unknown ? ErrorCode::PreviousOperationNotTerminated
                                       : ErrorCode::Busy,
                    "previous owned navigation has not terminated"};
        }
        if (error && !initialized)
            error = {ErrorCode::NotInitialized, "initialize Navigator first"};
        if (error && (targets.empty() || !std::all_of(targets.begin(), targets.end(), valid_pose)))
            error = {ErrorCode::InvalidGoal, "finite pose, valid quaternion and frame required"};
        if (error && !ready())
            error = {ErrorCode::NotReady, "map, localization, fresh odom and active Nav2 required"};
        if (error) {
            for (const auto& target : targets) {
                if (target.header.frame_id == map_frame) continue;
                try {
                    buffer->lookupTransform(
                        map_frame, target.header.frame_id,
                        rclcpp::Time(target.header.stamp, RCL_ROS_TIME));
                } catch (const tf2::TransformException&) {
                    error = {ErrorCode::InvalidGoal, "goal frame transform unavailable"};
                    break;
                }
            }
        }
        if (!error) {
            finish(op, error);
            return op;
        }
        operation = op;
        const std::weak_ptr<Operation> weak = op;
        operation_timer = node->create_wall_timer(
            std::chrono::milliseconds(20),
            [weak] {
                if (const auto held = weak.lock()) progress(held);
            },
            callbacks);
        return op;
    }
    template <class Action>
    void send(
        const std::shared_ptr<Operation>& op, const typename Action::Goal& goal,
        const typename rclcpp_action::Client<Action>::SharedPtr& client) {
        {
            std::lock_guard<std::mutex> lock(op->mutex);
            if (op->terminal) return;
        }
        typename rclcpp_action::Client<Action>::SendGoalOptions options;
        std::weak_ptr<rclcpp_action::Client<Action>> weak_client = client;
        options.goal_response_callback = [op, weak_client](auto handle) {
            if (!handle) {
                finish(op, {ErrorCode::GoalRejected, "Nav2 rejected goal"});
                return;
            }
            bool canceled;
            {
                std::lock_guard<std::mutex> lock(op->mutex);
                op->response = true;
                std::weak_ptr<rclcpp_action::ClientGoalHandle<Action>> weak_handle = handle;
                op->send_cancel = [weak_client, weak_handle] {
                    if (auto owned = weak_client.lock())
                        if (auto identity = weak_handle.lock()) try {
                                owned->async_cancel_goal(identity);
                            } catch (const std::exception&) {
                            }
                };
                canceled = op->cancel_reason != 0;
            }
            op->changed.notify_all();
            if (canceled) request_cancel(op, 1);
        };
        options.result_callback = [op](const auto& result) {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    finish(op, {});
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    finish(op, {ErrorCode::Canceled, "owned navigation canceled"});
                    break;
                default:
                    finish(op, {ErrorCode::NavigationFailed, "Nav2 terminal failure"});
                    break;
            }
        };
        try {
            client->async_send_goal(goal, options);
        } catch (const std::exception& exception) {
            finish(op, {ErrorCode::InternalError, exception.what()});
        }
    }
};

bool NavigationHandle::valid() const noexcept { return impl_ && impl_->operation; }
NavigationState NavigationHandle::state() const {
    if (!valid()) return NavigationState::Idle;
    progress(impl_->operation);
    std::lock_guard<std::mutex> lock(impl_->operation->mutex);
    return impl_->operation->state;
}
Result NavigationHandle::wait() {
    return valid() ? await(impl_->operation)
                   : Result{ErrorCode::InvalidGoal, "invalid navigation handle"};
}
Result NavigationHandle::cancel() {
    if (!valid()) return {ErrorCode::InvalidGoal, "invalid navigation handle"};
    request_cancel(impl_->operation, 1);
    auto result = await(impl_->operation);
    return result.code == ErrorCode::Canceled ? Result{} : result;
}
std::optional<Result> NavigationHandle::result() const {
    if (!valid()) return std::nullopt;
    progress(impl_->operation);
    std::lock_guard<std::mutex> lock(impl_->operation->mutex);
    return impl_->operation->terminal || impl_->operation->unknown
               ? std::optional<Result>(impl_->operation->result)
               : std::nullopt;
}
Navigator::Navigator(const rclcpp::Node::SharedPtr& node) : impl_(std::make_unique<Impl>(node)) {}
Navigator::~Navigator() {
    cancel();
    if (const auto op = impl_->operation) {
        std::lock_guard<std::mutex> lock(op->mutex);
        if (!op->terminal) {
            // Preserve communication for a late Accepted/Result, not the owner.
            // Both action kinds must remain available while identity is unresolved.
            struct Clients {
                decltype(impl_->to) to;
                decltype(impl_->through) through;
            };
            op->retained_client = std::make_shared<Clients>(Clients{impl_->to, impl_->through});
            impl_->node->get_node_base_interface()->get_context()->add_on_shutdown_callback([op] {
                std::lock_guard<std::mutex> held(op->mutex);
                op->retained_client.reset();
            });
        }
    }
}
Result Navigator::initialize(std::chrono::milliseconds timeout) {
    if (timeout.count() <= 0)
        return {ErrorCode::Timeout, "positive initialization deadline required"};
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        if (impl_->ready()) {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->initialized = true;
            return {};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return {ErrorCode::Timeout, "navigation readiness deadline expired"};
}
bool Navigator::is_ready() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->initialized && impl_->ready();
}
Result Navigator::set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
    geometry_msgs::msg::PoseStamped plain;
    plain.header = pose.header;
    plain.pose = pose.pose.pose;
    if (!valid_pose(plain) || pose.header.frame_id != impl_->map_frame ||
        !std::all_of(
            pose.pose.covariance.begin(), pose.pose.covariance.end(),
            [](double v) { return std::isfinite(v); }) ||
        pose.pose.covariance[0] < 0 || pose.pose.covariance[7] < 0 || pose.pose.covariance[35] < 0)
        return {ErrorCode::InvalidGoal, "invalid initial localization pose/covariance"};
    impl_->initial->publish(pose);
    return {};
}
Result Navigator::get_current_pose(geometry_msgs::msg::PoseStamped& pose) const {
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (!impl_->initialized) return {ErrorCode::NotInitialized, "initialize Navigator first"};
    }
    {
        std::lock_guard<std::mutex> lock(impl_->readiness->mutex);
        if (std::chrono::duration<double>(Clock::now() - impl_->readiness->odom_received).count() >
                impl_->freshness ||
            std::abs((impl_->node->now() - impl_->readiness->odom_stamp).seconds()) >
                impl_->freshness)
            return {ErrorCode::StateStale, "measured odometry stale"};
    }
    try {
        const auto tf =
            impl_->buffer->lookupTransform(impl_->map_frame, impl_->base_frame, tf2::TimePointZero);
        if (std::abs((impl_->node->now() - rclcpp::Time(tf.header.stamp, RCL_ROS_TIME)).seconds()) >
            impl_->freshness)
            return {ErrorCode::StateStale, "localization transform stale"};
        geometry_msgs::msg::PoseStamped result;
        result.header = tf.header;
        result.pose.position.x = tf.transform.translation.x;
        result.pose.position.y = tf.transform.translation.y;
        result.pose.position.z = tf.transform.translation.z;
        result.pose.orientation = tf.transform.rotation;
        if (!valid_pose(result))
            return {ErrorCode::StateUnavailable, "invalid localization transform"};
        pose = result;
        return {};
    } catch (const tf2::TransformException& exception) {
        return {ErrorCode::StateUnavailable, exception.what()};
    }
}
NavigationHandle Navigator::start_navigate_to(const geometry_msgs::msg::PoseStamped& target) {
    NavigationHandle handle;
    handle.impl_ = std::make_shared<NavigationHandle::Impl>();
    handle.impl_->operation = impl_->reserve({target});
    To::Goal goal;
    goal.pose = target;
    impl_->send<To>(handle.impl_->operation, goal, impl_->to);
    return handle;
}
Result Navigator::navigate_to(const geometry_msgs::msg::PoseStamped& target) {
    return start_navigate_to(target).wait();
}
Result Navigator::navigate_through(const std::vector<geometry_msgs::msg::PoseStamped>& targets) {
    const auto op = impl_->reserve(targets);
    Through::Goal goal;
    goal.poses = targets;
    impl_->send<Through>(op, goal, impl_->through);
    return await(op);
}
Result Navigator::cancel() {
    std::shared_ptr<Operation> op;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        op = impl_->operation;
    }
    if (!op) return {};
    request_cancel(op, 1);
    const auto result = await(op);
    return result.code == ErrorCode::Canceled ? Result{} : result;
}
NavigationState Navigator::state() const {
    std::shared_ptr<Operation> op;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        op = impl_->operation;
    }
    if (!op) return NavigationState::Idle;
    progress(op);
    std::lock_guard<std::mutex> lock(op->mutex);
    return op->state;
}
}  // namespace mfr3duo_nav
