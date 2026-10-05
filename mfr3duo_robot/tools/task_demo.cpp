#include <iostream>
#include <cmath>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>
#include "mfr3duo_robot/robot.hpp"
#include "task_server.hpp"
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
    std::unique_ptr<TaskServer> task_server;
    int code = 0;
    try {
        const bool serve_tasks = node->has_parameter("serve_tasks")
                                     ? node->get_parameter("serve_tasks").as_bool()
                                     : node->declare_parameter<bool>("serve_tasks", false);
        if (serve_tasks) {
            task_server = std::make_unique<TaskServer>(node);
            node->get_node_base_interface()->for_each_callback_group(
                [&](const rclcpp::CallbackGroup::SharedPtr& group) {
                    callback_groups.push_back(group);
                });
            while (context->is_valid()) std::this_thread::sleep_for(100ms);
        } else {
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
            const auto navigation =
                node->has_parameter("demo_navigation_pose")
                    ? node->get_parameter("demo_navigation_pose").as_double_array()
                    : std::vector<double>{-.35, .7, 0.};
            if (navigation.size() != 3) throw std::invalid_argument("demo_navigation_pose");
            destination.pose.position.x = navigation[0];
            destination.pose.position.y = navigation[1];
            destination.pose.orientation.z = std::sin(navigation[2] / 2);
            destination.pose.orientation.w = std::cos(navigation[2] / 2);
            geometry_msgs::msg::PoseStamped resting;
            resting.header.frame_id = "simulation_world";
            const auto placement = node->has_parameter("demo_place_pose")
                                       ? node->get_parameter("demo_place_pose").as_double_array()
                                       : std::vector<double>{.72, .75, .985};
            if (placement.size() != 3) throw std::invalid_argument("demo_place_pose");
            resting.pose.position.x = placement[0];
            resting.pose.position.y = placement[1];
            resting.pose.position.z = placement[2];
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
                      << std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - task_started)
                             .count()
                      << std::endl;
        }
    } catch (const std::exception& e) {
        const bool interrupted = !context->is_valid();
        std::cerr << (interrupted ? "ROBOT_DEMO_INTERRUPTED " : "ROBOT_DEMO_FAIL ") << e.what()
                  << std::endl;
        code = interrupted ? 130 : 1;
    }
    executor.cancel();
    spin.join();
    task_server.reset();
    if (context->is_valid()) executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return code;
}
