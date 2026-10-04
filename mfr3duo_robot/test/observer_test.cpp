#include <atomic>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <tf2_msgs/msg/tf_message.hpp>
#include "grasp_observer.hpp"
using namespace std::chrono_literals;
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("observer_test");
    node->declare_parameter("grasp_observation.timeout", .05);
    node->declare_parameter("grasp_observation.maximum_age", .05);
    node->declare_parameter("perception.objects_topic_prefix", "/observer_fixture/objects");
    node->declare_parameter("perception.tools_topic_prefix", "/observer_fixture/tools");
    using Pose = geometry_msgs::msg::PoseStamped;
    auto object = node->create_publisher<Pose>("/observer_fixture/objects/box/pose", 10);
    auto tool = node->create_publisher<Pose>("/observer_fixture/tools/left/pose", 10);
    auto tf = node->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).transient_local());
    tf2_msgs::msg::TFMessage frames;
    geometry_msgs::msg::TransformStamped frame;
    frame.header.frame_id = "fixture_world";
    frame.child_frame_id = "camera";
    frame.transform.rotation.w = 1;
    frame.transform.translation.x = .1;
    frames.transforms.push_back(frame);
    tf->publish(frames);
    std::atomic<int> mode{0};
    auto timer = node->create_wall_timer(5ms, [&] {
        const int current = mode.load();
        if (current == 1) return;
        Pose pose;
        pose.header.frame_id = "fixture_world";
        pose.pose.orientation.w = 1;
        pose.header.stamp =
            current == 2 ? node->now() - rclcpp::Duration::from_seconds(1) : node->now();
        if (current == 4) pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        object->publish(pose);
        if (current == 5) return;
        auto tcp = pose;
        if (current == 3) tcp.header.frame_id = "missing_frame";
        if (current == 6) tcp.header.stamp = node->now() - rclcpp::Duration::from_seconds(.2);
        if (current == 7) {
            tcp.header.stamp =
                rclcpp::Time(pose.header.stamp) - rclcpp::Duration::from_seconds(.001);
            tcp.pose.position.x = .01;
            tool->publish(tcp);
            tcp.header.stamp =
                rclcpp::Time(pose.header.stamp) + rclcpp::Duration::from_seconds(.001);
            tcp.pose.position.x = .03;
            tool->publish(tcp);
            return;
        }
        if (current == 8) tcp.header.frame_id = "camera";
        tool->publish(tcp);
    });
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    std::thread spin([&] { executor.spin(); });
    int result = 0;
    try {
        mfr3duo_robot::SimulationGraspObserver observer(node);
        const auto query = [&] {
            return observer.observe("box", mfr3duo_robot::Manipulator::Left);
        };
        const auto await = [&](bool valid) {
            const auto deadline = std::chrono::steady_clock::now() + 2s;
            while (std::chrono::steady_clock::now() < deadline) {
                const auto begin = std::chrono::steady_clock::now();
                auto o = query();
                if (std::chrono::steady_clock::now() - begin > 150ms)
                    throw std::runtime_error("unbounded observer timeout");
                if (o.valid == valid) return;
                std::this_thread::sleep_for(10ms);
            }
            throw std::runtime_error("validity did not converge");
        };
        await(true);
        for (int invalid = 1; invalid <= 6; ++invalid) {
            mode = invalid;
            await(false);
            mode = 0;
            await(true);
        }
        mode = 7;
        bool interpolated = false;
        for (int i = 0; i < 100; ++i) {
            auto o = query();
            if (o.valid && std::abs(o.tool_pose.pose.position.x - .02) < 1e-6) {
                interpolated = true;
                break;
            }
            std::this_thread::sleep_for(10ms);
        }
        if (!interpolated) throw std::runtime_error("tool interpolation failed");
        mode = 8;
        bool transformed = false;
        for (int i = 0; i < 100; ++i) {
            auto o = query();
            if (o.valid && std::abs(o.tool_pose.pose.position.x - .1) < 1e-6) {
                transformed = true;
                break;
            }
            std::this_thread::sleep_for(10ms);
        }
        if (!transformed) throw std::runtime_error("sampling-time TF conversion failed");
        if (observer.observe("unknown", mfr3duo_robot::Manipulator::Left).valid ||
            observer.observe("box", mfr3duo_robot::Manipulator::Auto).valid)
            throw std::runtime_error("invalid identity accepted");
        std::cout << "OBSERVER_SOFTWARE_PASS timeout, stale, invalid pose, missing tool, time "
                     "mismatch, interpolation, TF, identity\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        result = 1;
    }
    executor.cancel();
    spin.join();
    rclcpp::shutdown();
    return result;
}
