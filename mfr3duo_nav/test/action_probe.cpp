#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "mfr3duo_nav/navigator.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
namespace {
using To = nav2_msgs::action::NavigateToPose;
using Through = nav2_msgs::action::NavigateThroughPoses;
using Handle = rclcpp_action::ServerGoalHandle<To>;
enum class Mode { Success, Abort, Hold, LateAccept, LateReject, Reject };
struct Fixture {
    std::atomic<Mode> mode{Mode::Success};
    std::atomic<bool> release{false};
    std::atomic<unsigned> cancellations{0};
    std::mutex mutex;
    std::vector<std::shared_ptr<Handle>> held;
};
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigator_action_probe");
    auto server = rclcpp::Node::make_shared("navigator_fixture");
    auto group = server->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto fixture = std::make_shared<Fixture>();
    std::vector<rclcpp::Service<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle;
    for (const char* name :
         {"map_server", "amcl", "controller_server", "planner_server", "smoother_server",
          "behavior_server", "bt_navigator", "velocity_smoother"})
        lifecycle.push_back(server->create_service<lifecycle_msgs::srv::GetState>(
            std::string(name) + "/get_state",
            [](const lifecycle_msgs::srv::GetState::Request::SharedPtr,
               lifecycle_msgs::srv::GetState::Response::SharedPtr response) {
                response->current_state.id = 3;
            },
            rmw_qos_profile_services_default, group));
    auto map = server->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map", rclcpp::QoS(1).transient_local());
    auto odom = server->create_publisher<nav_msgs::msg::Odometry>("tmr_controller/odom", 10);
    auto tf = server->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.width = grid.info.height = 10;
    grid.info.resolution = .1;
    grid.info.origin.orientation.w = 1.;
    grid.data.resize(100, 0);
    map->publish(grid);
    auto timer = server->create_wall_timer(
        10ms,
        [server, odom, tf, fixture] {
            nav_msgs::msg::Odometry measured;
            measured.header.frame_id = "odom";
            measured.child_frame_id = "base_link";
            measured.header.stamp = server->now();
            measured.pose.pose.orientation.w = 1.;
            odom->publish(measured);
            tf2_msgs::msg::TFMessage transforms;
            geometry_msgs::msg::TransformStamped world, base;
            world.header.stamp = measured.header.stamp;
            world.header.frame_id = "map";
            world.child_frame_id = "odom";
            world.transform.rotation.w = 1.;
            base = world;
            base.header.frame_id = "odom";
            base.child_frame_id = "base_link";
            transforms.transforms = {world, base};
            tf->publish(transforms);
            if (!fixture->release.load()) return;
            std::lock_guard<std::mutex> lock(fixture->mutex);
            for (auto& handle : fixture->held) {
                if (handle->is_canceling()) handle->canceled(std::make_shared<To::Result>());
            }
            fixture->held.erase(
                std::remove_if(
                    fixture->held.begin(), fixture->held.end(),
                    [](const auto& handle) { return !handle->is_active(); }),
                fixture->held.end());
        },
        group);
    auto action = rclcpp_action::create_server<To>(
        server, "navigate_to_pose",
        [fixture](const auto&, const auto&) {
            const auto mode = fixture->mode.load();
            if (mode == Mode::LateAccept || mode == Mode::LateReject)
                std::this_thread::sleep_for(800ms);
            return mode == Mode::Reject || mode == Mode::LateReject
                       ? rclcpp_action::GoalResponse::REJECT
                       : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [fixture](const auto&) {
            ++fixture->cancellations;
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [fixture](const auto& handle) {
            const auto mode = fixture->mode.load();
            if (mode == Mode::Success)
                handle->succeed(std::make_shared<To::Result>());
            else if (mode == Mode::Abort)
                handle->abort(std::make_shared<To::Result>());
            else {
                std::lock_guard<std::mutex> lock(fixture->mutex);
                fixture->held.push_back(handle);
            }
        },
        rcl_action_server_get_default_options(), group);
    auto through = rclcpp_action::create_server<Through>(
        server, "navigate_through_poses",
        [](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [](const auto& handle) { handle->succeed(std::make_shared<Through::Result>()); },
        rcl_action_server_get_default_options(), group);
    node->declare_parameter("navigator.goal_response_timeout", .2);
    node->declare_parameter("navigator.navigation_timeout", 2.);
    node->declare_parameter("navigator.terminal_timeout", .3);
    node->declare_parameter("navigator.state_timeout", 1.);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
    executor.add_node(node);
    executor.add_node(server);
    std::thread spin([&] { executor.spin(); });
    int failed = 0;
    try {
        using mfr3duo_nav::ErrorCode;
        using mfr3duo_nav::NavigationState;
        mfr3duo_nav::Navigator nav(node);
        require(static_cast<bool>(nav.initialize(8s)), "fake readiness");
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "map";
        target.pose.orientation.w = 1.;
        require(static_cast<bool>(nav.navigate_to(target)), "terminal Success");
        require(
            static_cast<bool>(nav.navigate_through({target, target})),
            "NavigateThroughPoses endpoint");
        fixture->mode = Mode::Abort;
        require(
            nav.navigate_to(target).code == ErrorCode::NavigationFailed, "terminal Abort mapping");
        fixture->mode = Mode::Reject;
        require(nav.navigate_to(target).code == ErrorCode::GoalRejected, "GoalRejected mapping");
        fixture->mode = Mode::Hold;
        auto foreign_client = rclcpp_action::create_client<To>(node, "navigate_to_pose");
        To::Goal foreign_goal;
        foreign_goal.pose = target;
        auto foreign_response = foreign_client->async_send_goal(foreign_goal);
        require(
            foreign_response.wait_for(2s) == std::future_status::ready, "foreign goal accepted");
        const auto foreign_handle = foreign_response.get();
        require(static_cast<bool>(foreign_handle), "foreign goal identity");
        auto foreign_result = foreign_client->async_get_result(foreign_handle);
        const auto cancel_baseline = fixture->cancellations.load();
        auto owned = nav.start_navigate_to(target);
        std::this_thread::sleep_for(100ms);
        require(
            nav.start_navigate_to(target).wait().code == ErrorCode::Busy, "one active operation");
        require(owned.cancel().code == ErrorCode::CancelFailed, "cancel ACK is not terminal");
        require(owned.state() == NavigationState::TerminationUnknown, "unknown state");
        require(
            fixture->cancellations.load() == cancel_baseline + 1, "cancel only exact owned UUID");
        require(
            foreign_result.wait_for(0s) != std::future_status::ready,
            "foreign goal not terminated");
        require(
            nav.start_navigate_to(target).wait().code == ErrorCode::PreviousOperationNotTerminated,
            "unknown blocks next goal");
        fixture->release = true;
        auto deadline = std::chrono::steady_clock::now() + 2s;
        while (owned.state() == NavigationState::TerminationUnknown &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(10ms);
        require(owned.wait().code == ErrorCode::Canceled, "late actual terminal converges");
        require(
            foreign_result.wait_for(0s) != std::future_status::ready,
            "foreign goal remains active after owned terminal");
        foreign_client->async_cancel_goal(foreign_handle);
        require(
            foreign_result.wait_for(2s) == std::future_status::ready,
            "fixture foreign goal cleanup");
        require(
            foreign_result.get().code == rclcpp_action::ResultCode::CANCELED,
            "foreign fixture cleanup terminal");
        fixture->release = false;
        fixture->mode = Mode::LateAccept;
        const auto before = fixture->cancellations.load();
        auto delayed = nav.start_navigate_to(target);
        require(
            delayed.wait().code == ErrorCode::CancelFailed,
            "response timeout identity stays unknown");
        require(
            nav.start_navigate_to(target).wait().code == ErrorCode::PreviousOperationNotTerminated,
            "pending identity guard");
        deadline = std::chrono::steady_clock::now() + 2s;
        while (fixture->cancellations.load() == before &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(10ms);
        require(fixture->cancellations.load() == before + 1, "late Accepted exact cancellation");
        fixture->release = true;
        deadline = std::chrono::steady_clock::now() + 2s;
        while (delayed.state() == NavigationState::TerminationUnknown &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(10ms);
        require(
            delayed.wait().code == ErrorCode::Timeout, "local timeout only after remote terminal");
        require(static_cast<bool>(nav.cancel()), "confirmed timeout releases navigation ownership");
        fixture->mode = Mode::LateReject;
        auto rejected = nav.start_navigate_to(target);
        require(
            rejected.wait().code == ErrorCode::CancelFailed, "late rejection initially unknown");
        deadline = std::chrono::steady_clock::now() + 2s;
        while (rejected.state() == NavigationState::TerminationUnknown &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(10ms);
        require(
            rejected.wait().code == ErrorCode::Timeout, "late Rejected confirms no remote goal");
        fixture->mode = Mode::Success;
        require(static_cast<bool>(nav.navigate_to(target)), "next navigation after convergence");
        fixture->mode = Mode::LateAccept;
        fixture->release = false;
        const auto destructor_baseline = fixture->cancellations.load();
        const auto started = std::chrono::steady_clock::now();
        {
            mfr3duo_nav::Navigator temporary(node);
            require(static_cast<bool>(temporary.initialize(3s)), "temporary readiness");
            auto pending = temporary.start_navigate_to(target);
        }
        require(std::chrono::steady_clock::now() - started < 2s, "finite pending-goal destruction");
        deadline = std::chrono::steady_clock::now() + 2s;
        while (fixture->cancellations.load() == destructor_baseline &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(10ms);
        require(
            fixture->cancellations.load() == destructor_baseline + 1,
            "destructor late Accepted canceled by retained identity lease");
        fixture->release = true;
        std::this_thread::sleep_for(100ms);
        std::cout << "NAVIGATOR_ACTION_PASS\n";
    } catch (const std::exception& exception) {
        std::cerr << "NAVIGATOR_ACTION_FAIL " << exception.what() << '\n';
        failed = 1;
    }
    executor.cancel();
    spin.join();
    executor.remove_node(node);
    executor.remove_node(server);
    timer.reset();
    action.reset();
    through.reset();
    lifecycle.clear();
    node.reset();
    server.reset();
    rclcpp::shutdown();
    return failed;
}
