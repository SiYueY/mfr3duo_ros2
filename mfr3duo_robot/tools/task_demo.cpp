#include <iostream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include "mfr3duo_robot/robot.hpp"
using namespace mfr3duo_robot;
using namespace std::chrono_literals;
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(
        "task_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    std::thread spin([&] { executor.spin(); });
    const auto context = node->get_node_base_interface()->get_context();
    std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups;
    int code = 0;
    try {
        Robot robot(node);
        // Retain group shells until the externally owned executor has stopped, even
        // when SIGINT shuts down the Context before Robot is destroyed (Humble).
        node->get_node_base_interface()->for_each_callback_group(
            [&](const rclcpp::CallbackGroup::SharedPtr& group) {
                callback_groups.push_back(group);
            });
        auto ready = robot.initialize(90s);
        if (!ready) throw std::runtime_error("Robot initialize: " + ready.message);
        geometry_msgs::msg::PoseStamped destination;
        destination.header.frame_id = "map";
        destination.pose.position.x = -.35;
        destination.pose.position.y = .7;
        destination.pose.orientation.w = 1;
        geometry_msgs::msg::PoseStamped resting;
        resting.header.frame_id = "simulation_world";
        resting.pose.position.x = .72;
        resting.pose.position.y = .75;
        resting.pose.position.z = .985;
        resting.pose.orientation.w = 1;
        TaskSequence tasks;
        tasks.add(std::make_unique<NavigateTask>(destination));
        tasks.add(std::make_unique<PickTask>("box"));
        auto place = std::make_unique<PlaceTask>("box");
        place->set_place_pose(resting);
        tasks.add(std::move(place));
        const auto task_started = std::chrono::steady_clock::now();
        auto result = robot.execute(tasks);
        if (!result)
            throw std::runtime_error(
                "Task: error=" + std::to_string(static_cast<int>(result.error)) + " " +
                result.message);
        std::cout << "ROBOT_DEMO_PASS NavigateTask -> physical PickTask -> physical PlaceTask"
                  << " task_seconds="
                  << std::chrono::duration<double>(std::chrono::steady_clock::now() - task_started)
                         .count()
                  << std::endl;
    } catch (const std::exception& e) {
        const bool interrupted = !context->is_valid();
        std::cerr << (interrupted ? "ROBOT_DEMO_INTERRUPTED " : "ROBOT_DEMO_FAIL ") << e.what()
                  << std::endl;
        code = interrupted ? 130 : 1;
    }
    executor.cancel();
    spin.join();
    if (context->is_valid()) executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return code;
}
