#include <chrono>
#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <future>
#include <atomic>
#include <stdexcept>
#include <thread>

#include "mfr3duo_moveit/move_group.hpp"
#include "mfr3duo_nav/navigator.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
}  // namespace
double yaw(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
bool overlaps_obstacle(const geometry_msgs::msg::Pose& pose) {
    const double angle = yaw(pose.orientation), c = std::cos(angle), s = std::sin(angle);
    const std::array<std::array<double, 2>, 4> outline{
        {{-.47, -1.14}, {.78, -1.14}, {.78, 1.09}, {-.47, 1.09}}};
    const std::array<std::array<double, 2>, 4> axes{{{1, 0}, {0, 1}, {c, s}, {-s, c}}};
    for (const auto& axis : axes) {
        double low = 1e9, high = -1e9;
        for (const auto& point : outline) {
            const double x = pose.position.x + c * point[0] - s * point[1];
            const double y = pose.position.y + s * point[0] + c * point[1];
            const double projection = x * axis[0] + y * axis[1];
            low = std::min(low, projection);
            high = std::max(high, projection);
        }
        const double center = 1.8 * axis[0] + 1.6 * axis[1];
        const double radius = .3 * std::abs(axis[0]) + .45 * std::abs(axis[1]);
        if (high < center - radius || low > center + radius) return false;
    }
    return true;
}
struct Samples {
    std::mutex mutex;
    geometry_msgs::msg::PoseStamped truth;
    bool available{false};
    double maximum_y{0};
    double measured_speed{1};
    double measured_yaw_rate{1};
    std::chrono::steady_clock::time_point odom_received{};
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigator_runtime_probe");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    std::thread spin([&] { executor.spin(); });
    int exit_code = 0;
    auto samples = std::make_shared<Samples>();
    auto truth_subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "sensors/simulation/base_pose", 10,
        [samples](const geometry_msgs::msg::PoseStamped& message) {
            std::lock_guard<std::mutex> lock(samples->mutex);
            samples->truth = message;
            samples->available = true;
        });
    auto velocity_subscription = node->create_subscription<geometry_msgs::msg::Twist>(
        "tmr_controller/cmd_vel", 10, [samples](const geometry_msgs::msg::Twist& message) {
            std::lock_guard<std::mutex> lock(samples->mutex);
            samples->maximum_y = std::max(samples->maximum_y, std::abs(message.linear.y));
        });
    auto odometry_subscription = node->create_subscription<nav_msgs::msg::Odometry>(
        "tmr_controller/odom", 10, [samples](const nav_msgs::msg::Odometry& message) {
            std::lock_guard<std::mutex> lock(samples->mutex);
            samples->measured_speed =
                std::hypot(message.twist.twist.linear.x, message.twist.twist.linear.y);
            samples->measured_yaw_rate = std::abs(message.twist.twist.angular.z);
            samples->odom_received = std::chrono::steady_clock::now();
        });
    try {
        tf2_ros::Buffer truth_buffer(node->get_clock());
        tf2_ros::TransformListener truth_listener(truth_buffer, node, false);
        truth_buffer.setUsingDedicatedThread(true);
        const auto left = node->declare_parameter<std::vector<double>>(
            "navigation_posture.left_arm", std::vector<double>{});
        const auto right = node->declare_parameter<std::vector<double>>(
            "navigation_posture.right_arm", std::vector<double>{});
        const auto height = node->declare_parameter("navigation_posture.spine_height", .05);
        require(left.size() == 7 && right.size() == 7, "load configured navigation posture");
        mfr3duo_moveit::MoveGroup move(node);
        require(static_cast<bool>(move.initialize(60s)), "MoveGroup initialize");
        require(
            static_cast<bool>(move.add_groups(
                {mfr3duo_moveit::RobotGroup::LeftArm, mfr3duo_moveit::RobotGroup::RightArm,
                 mfr3duo_moveit::RobotGroup::Spine})),
            "navigation posture groups");
        require(
            static_cast<bool>(
                move.add_joint_position_target(mfr3duo_moveit::RobotGroup::LeftArm, left)),
            "left transport target");
        require(
            static_cast<bool>(
                move.add_joint_position_target(mfr3duo_moveit::RobotGroup::RightArm, right)),
            "right transport target");
        require(
            static_cast<bool>(
                move.add_joint_position_target(mfr3duo_moveit::RobotGroup::Spine, height)),
            "spine transport target");
        const auto posture = move.move();
        if (!posture)
            std::cerr << "POSTURE " << static_cast<int>(posture.code) << ' ' << posture.message
                      << '\n';
        require(static_cast<bool>(posture), "execute collision-checked fixed navigation posture");
        mfr3duo_nav::Navigator nav(node);
        require(static_cast<bool>(nav.initialize(60s)), "Navigator initialize");
        require(nav.is_ready(), "Navigator readiness");
        geometry_msgs::msg::PoseStamped pose;
        require(static_cast<bool>(nav.get_current_pose(pose)), "current localized pose");
        auto target = pose;
        target.pose.position.y += .35;
        double maximum_position_error = 0, maximum_heading_error = 0;
        const auto execute_to = [&](const geometry_msgs::msg::PoseStamped& goal,
                                    const char* label) {
            auto operation = nav.start_navigate_to(goal);
            geometry_msgs::msg::PoseStamped previous;
            bool first = true;
            auto next_progress = std::chrono::steady_clock::now() + 5s;
            while (!operation.result()) {
                geometry_msgs::msg::PoseStamped measured;
                require(
                    static_cast<bool>(nav.get_current_pose(measured)),
                    "continuous localization available");
                if (!first)
                    require(
                        std::hypot(
                            measured.pose.position.x - previous.pose.position.x,
                            measured.pose.position.y - previous.pose.position.y) <= .15,
                        "localization position discontinuity");
                if (!first)
                    require(
                        std::abs(std::remainder(
                            yaw(measured.pose.orientation) - yaw(previous.pose.orientation),
                            6.283185307179586)) <= .15,
                        "localization heading discontinuity");
                previous = measured;
                first = false;
                if (std::chrono::steady_clock::now() >= next_progress) {
                    std::cout << label << " progress=" << measured.pose.position.x << ','
                              << measured.pose.position.y
                              << " yaw=" << yaw(measured.pose.orientation) << std::endl;
                    next_progress = std::chrono::steady_clock::now() + 5s;
                }
                geometry_msgs::msg::PoseStamped truth;
                {
                    std::lock_guard<std::mutex> lock(samples->mutex);
                    require(samples->available, "same-instance truth available");
                    truth = samples->truth;
                }
                {
                    if (overlaps_obstacle(truth.pose)) {
                        std::cerr << "FOOTPRINT_COLLISION truth=" << truth.pose.position.x << ','
                                  << truth.pose.position.y << " yaw=" << yaw(truth.pose.orientation)
                                  << " localized=" << measured.pose.position.x << ','
                                  << measured.pose.position.y
                                  << " yaw=" << yaw(measured.pose.orientation) << std::endl;
                        throw std::runtime_error("fixed footprint intersects physical obstacle");
                    }
                    const auto truth_stamp = rclcpp::Time(truth.header.stamp);
                    require(
                        (node->now() - truth_stamp).seconds() <= .3, "fresh physical truth sample");
                    const auto localized = truth_buffer.lookupTransform(
                        "map", "base_link", truth_stamp, rclcpp::Duration::from_seconds(.3));
                    const double heading_error = std::abs(std::remainder(
                        yaw(localized.transform.rotation) - yaw(truth.pose.orientation),
                        6.283185307179586));
                    const double position_error = std::hypot(
                        localized.transform.translation.x - truth.pose.position.x,
                        localized.transform.translation.y - truth.pose.position.y);
                    maximum_position_error = std::max(maximum_position_error, position_error);
                    maximum_heading_error = std::max(maximum_heading_error, heading_error);
                    if (heading_error > .15 || position_error > .1)
                        std::cerr << "LOCALIZATION_ERROR position=" << position_error
                                  << " heading=" << heading_error
                                  << " truth=" << truth.pose.position.x << ','
                                  << truth.pose.position.y
                                  << " localized=" << localized.transform.translation.x << ','
                                  << localized.transform.translation.y
                                  << " stamp=" << truth_stamp.seconds() << std::endl;
                    require(heading_error <= .15, "AMCL/world truth heading error");
                    require(position_error <= .1, "AMCL/world truth position error");
                }
                std::this_thread::sleep_for(100ms);
            }
            const auto finished = operation.wait();
            std::cout << label << " pose=" << previous.pose.position.x << ','
                      << previous.pose.position.y << " yaw=" << yaw(previous.pose.orientation)
                      << " result=" << static_cast<int>(finished.code) << ' ' << finished.message
                      << std::endl;
            return finished;
        };
        const auto result = execute_to(target, "lateral");
        if (!result)
            std::cerr << "NAVIGATION " << static_cast<int>(result.code) << ' ' << result.message
                      << '\n';
        require(static_cast<bool>(result), "facade lateral navigation");
        {
            std::lock_guard<std::mutex> lock(samples->mutex);
            require(samples->maximum_y > .005, "DWB actual nonzero linear.y output");
            std::cout << "maximum linear.y=" << samples->maximum_y << std::endl;
        }
        require(static_cast<bool>(nav.get_current_pose(pose)), "forward start");
        target = pose;
        target.pose.position.x += .15;
        require(static_cast<bool>(execute_to(target, "forward")), "facade forward navigation");
        require(static_cast<bool>(nav.get_current_pose(pose)), "diagonal start");
        target = pose;
        target.pose.position.x += .15;
        target.pose.position.y -= .15;
        require(static_cast<bool>(execute_to(target, "diagonal")), "facade diagonal navigation");
        require(static_cast<bool>(nav.get_current_pose(pose)), "rotation start");
        target = pose;
        const double heading = std::atan2(
                                   2 * pose.pose.orientation.w * pose.pose.orientation.z,
                                   1 - 2 * pose.pose.orientation.z * pose.pose.orientation.z) +
                               .3;
        target.pose.orientation.x = target.pose.orientation.y = 0;
        target.pose.orientation.z = std::sin(heading / 2);
        target.pose.orientation.w = std::cos(heading / 2);
        require(static_cast<bool>(execute_to(target, "rotation")), "facade rotation navigation");
        require(static_cast<bool>(nav.get_current_pose(pose)), "through-poses start");
        auto second_waypoint = pose;
        second_waypoint.pose.position.x -= .15;
        const auto through_result = nav.navigate_through({pose, second_waypoint});
        require(static_cast<bool>(through_result), "real NavigateThroughPoses result");
        geometry_msgs::msg::PoseStamped through_end;
        require(static_cast<bool>(nav.get_current_pose(through_end)), "through-poses end");
        require(
            through_end.pose.position.x < pose.pose.position.x - .04,
            "through-poses actual displacement");
        std::cout << "through-poses result=" << static_cast<int>(through_result.code) << std::endl;
        target.header.frame_id = "map";
        target.header.stamp = node->now();
        target.pose.position.x = 2.85;
        target.pose.position.y = .4;
        target.pose.position.z = 0;
        target.pose.orientation.x = target.pose.orientation.y = target.pose.orientation.z = 0;
        target.pose.orientation.w = 1.;
        require(
            static_cast<bool>(execute_to(target, "obstacle detour")),
            "navigate around physical obstacle");
        auto invalid = target;
        invalid.header.frame_id = "missing_navigation_frame";
        require(
            nav.start_navigate_to(invalid).wait().code == mfr3duo_nav::ErrorCode::InvalidGoal,
            "unknown goal frame");
        require(static_cast<bool>(nav.get_current_pose(pose)), "post-navigation localization");
        target = pose;
        target.pose.position.y -= .8;
        auto handle = nav.start_navigate_to(target);
        require(handle.valid(), "async handle identity");
        require(
            nav.start_navigate_to(target).wait().code == mfr3duo_nav::ErrorCode::Busy,
            "navigation Busy");
        std::this_thread::sleep_for(2s);
        require(static_cast<bool>(handle.cancel()), "exact goal cancel and terminal confirmation");
        require(handle.wait().code == mfr3duo_nav::ErrorCode::Canceled, "retained canceled result");
        const auto stop_deadline = std::chrono::steady_clock::now() + 4s;
        bool stopped = false;
        while (std::chrono::steady_clock::now() < stop_deadline) {
            {
                std::lock_guard<std::mutex> lock(samples->mutex);
                stopped = std::chrono::steady_clock::now() - samples->odom_received < 300ms &&
                          samples->measured_speed < .005 && samples->measured_yaw_rate < .01;
            }
            if (stopped) break;
            std::this_thread::sleep_for(20ms);
        }
        require(stopped, "measured odometry stopped after confirmed navigation cancellation");
        std::cout << "LOCALIZATION_MAX_ERROR position=" << maximum_position_error
                  << " heading=" << maximum_heading_error << std::endl;
        std::cout << "NAVIGATOR_RUNTIME_PASS\n";
    } catch (const std::exception& e) {
        std::cerr << "NAVIGATOR_RUNTIME_FAIL " << e.what() << '\n';
        exit_code = 1;
    }
    executor.cancel();
    spin.join();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return exit_code;
}
