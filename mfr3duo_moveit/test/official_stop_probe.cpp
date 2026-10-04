// Software fixture: delayed JTC terminal results, real MoveIt/TEM action path.
#include "mfr3duo_moveit/move_group.hpp"
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <future>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

namespace {
using namespace std::chrono_literals;
using Fjt = control_msgs::action::FollowJointTrajectory;
using Parent = moveit_msgs::action::ExecuteTrajectory;
using Child = rclcpp_action::ServerGoalHandle<Fjt>;
void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
struct FixtureState {
    std::mutex mutex;
    std::array<std::shared_ptr<Child>, 3> goals;
    std::array<bool, 3> release{};
    std::array<bool, 3> cancel{};
    std::array<unsigned, 3> generations{};
    bool moving{false};
};
struct Fixture {
    std::shared_ptr<FixtureState> state = std::make_shared<FixtureState>();
    std::array<rclcpp_action::Server<Fjt>::SharedPtr, 3> servers;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::CallbackGroup::SharedPtr group;
    rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers;
    Fixture(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model) {
        controllers = node->create_service<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers",
            [](const std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>,
               std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response) {
                for (const auto* name :
                     {"left_arm_controller", "right_arm_controller", "spine_controller"}) {
                    controller_manager_msgs::msg::ControllerState state;
                    state.name = name;
                    state.state = "active";
                    response->controller.push_back(state);
                }
            });
        group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        const std::array<std::string, 3> names{
            "left_arm_controller", "right_arm_controller", "spine_controller"};
        const auto record = state;
        for (std::size_t i = 0; i < names.size(); ++i)
            servers[i] = rclcpp_action::create_server<Fjt>(
                node->get_node_base_interface(), node->get_node_clock_interface(),
                node->get_node_logging_interface(), node->get_node_waitables_interface(),
                names[i] + "/follow_joint_trajectory",
                [](const auto&, const auto&) {
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                },
                [record, i](const std::shared_ptr<Child>&) {
                    std::lock_guard<std::mutex> lock(record->mutex);
                    record->cancel[i] = true;
                    return rclcpp_action::CancelResponse::ACCEPT;
                },
                [record, i](const std::shared_ptr<Child>& goal) {
                    std::lock_guard<std::mutex> lock(record->mutex);
                    record->goals[i] = goal;
                    record->release[i] = false;
                    record->cancel[i] = false;
                    ++record->generations[i];
                },
                rcl_action_server_get_default_options(), group);
        joints = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        moveit::core::RobotState home(model);
        home.setToDefaultValues();
        home.setToDefaultValues(model->getJointModelGroup("dual_arm_spine"), "home");
        for (const auto* side : {"left", "right"})
            home.setVariablePosition(std::string(side) + "_fr3v2_1_finger_joint1", .035);
        sensor_msgs::msg::JointState message;
        message.name = model->getVariableNames();
        for (const auto& name : message.name)
            message.position.push_back(home.getVariablePosition(name));
        message.velocity.resize(message.name.size(), 0);
        const auto publisher = joints;
        const auto joint_message = std::make_shared<sensor_msgs::msg::JointState>(message);
        timer = node->create_wall_timer(
            5ms,
            [record, node, publisher, joint_message]() {
                joint_message->header.stamp = node->now();
                std::lock_guard<std::mutex> lock(record->mutex);
                std::fill(
                    joint_message->velocity.begin(), joint_message->velocity.end(),
                    record->moving ? .1 : 0);
                publisher->publish(*joint_message);
                for (std::size_t i = 0; i < record->goals.size(); ++i) {
                    const auto goal = record->goals[i];
                    if (!goal || !record->release[i] || !goal->is_active()) continue;
                    auto result = std::make_shared<Fjt::Result>();
                    result->error_code = result->SUCCESSFUL;
                    if (record->cancel[i]) {
                        if (goal->is_canceling()) goal->canceled(result);
                    } else
                        goal->succeed(result);
                }
            },
            group);
    }
};
template <class Predicate>
void wait(Predicate predicate, const std::string& detail) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return;
        std::this_thread::sleep_for(5ms);
    }
    throw std::runtime_error(detail);
}
void run(
    const rclcpp::Node::SharedPtr& node, Fixture& fixture,
    const moveit::core::RobotModelPtr& model) {
    using namespace mfr3duo_moveit;
    auto check = [](Result result) { require(static_cast<bool>(result), result.message); };
    MoveGroup move(node), other(node);
    check(move.initialize(60s));
    check(other.initialize(5s));
    check(move.add_groups({RobotGroup::LeftArm, RobotGroup::RightArm, RobotGroup::Spine}));
    moveit::core::RobotState home(model);
    home.setToDefaultValues();
    home.setToDefaultValues(model->getJointModelGroup("dual_arm_spine"), "home");
    for (const auto& item : std::array<std::pair<RobotGroup, const char*>, 2>{
             {{RobotGroup::LeftArm, "left_arm"}, {RobotGroup::RightArm, "right_arm"}}}) {
        std::vector<double> target;
        home.copyJointGroupPositions(item.second, target);
        target[0] += .05;
        check(move.add_joint_position_target(item.first, target));
    }
    check(move.add_joint_position_target(
        RobotGroup::Spine, home.getVariablePosition("franka_spine_vertical_joint")));
    Plan plan;
    check(move.plan(plan));
    auto running = std::async(std::launch::async, [&] { return move.execute(plan); });
    auto generation = [&](unsigned count) {
        std::lock_guard<std::mutex> lock(fixture.state->mutex);
        return std::all_of(
            fixture.state->generations.begin(), fixture.state->generations.end(),
            [count](auto n) { return n == count; });
    };
    wait([&] { return generation(1); }, "three child goals not received");
    auto stopping = std::async(std::launch::async, [&] { return move.stop(); });
    wait(
        [&] {
            std::lock_guard<std::mutex> lock(fixture.state->mutex);
            return std::all_of(
                fixture.state->cancel.begin(), fixture.state->cancel.end(),
                [](bool v) { return v; });
        },
        "official stop did not cancel all controllers");
    {
        std::lock_guard<std::mutex> lock(fixture.state->mutex);
        fixture.state->release[0] = true;
    }
    require(
        stopping.wait_for(200ms) == std::future_status::timeout,
        "first child alone completed stop");
    require(
        running.wait_for(0ms) == std::future_status::timeout,
        "early parent terminal released facade");
    require(!move.is_ready(), "early parent terminal marked facade ready");
    require(
        other.execute(plan).code == ErrorCode::Busy,
        "shared channel did not reject parallel execution");
    {
        std::lock_guard<std::mutex> lock(fixture.state->mutex);
        fixture.state->moving = true;
        fixture.state->release.fill(true);
    }
    require(
        stopping.wait_for(250ms) == std::future_status::timeout,
        "child results alone released moving robot");
    {
        std::lock_guard<std::mutex> lock(fixture.state->mutex);
        fixture.state->moving = false;
    }
    require(stopping.wait_for(3s) == std::future_status::ready, "measured stop did not converge");
    require(stopping.get().code == ErrorCode::Canceled, "stop result missing cancellation");
    require(running.get().code == ErrorCode::Canceled, "execute result missing cancellation");
    require(move.is_ready(), "fenced stop did not restore readiness");
    auto next = std::async(std::launch::async, [&] { return move.execute(plan); });
    wait([&] { return generation(2); }, "next child goals not received");
    {
        std::lock_guard<std::mutex> lock(fixture.state->mutex);
        fixture.state->release.fill(true);
    }
    require(next.wait_for(5s) == std::future_status::ready, "next execution deadline");
    check(next.get());
    std::cout << "OFFICIAL_STOP_PASS official stop, all three child results, measured stop, "
                 "shared serial channel, next success"
              << std::endl;
}

}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "official_stop_probe",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    std::unique_ptr<Fixture> fixture;
    int status = 0;
    try {
        robot_model_loader::RobotModelLoader loader(node);
        require(loader.getModel() != nullptr, "fixture model missing");
        fixture = std::make_unique<Fixture>(node, loader.getModel());
        run(node, *fixture, loader.getModel());
    } catch (const std::exception& error) {
        std::cerr << "OFFICIAL_STOP_FAIL " << error.what() << std::endl;
        status = 1;
    }
    executor.cancel();
    spinner.join();
    executor.remove_node(node);
    fixture.reset();
    node.reset();
    rclcpp::shutdown();
    return status;
}
