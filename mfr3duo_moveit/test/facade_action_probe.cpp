#include "mfr3duo_moveit/move_group.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

namespace {
using namespace std::chrono_literals;
using namespace mfr3duo_moveit;
using Action = moveit_msgs::action::ExecuteTrajectory;
using Fjt = control_msgs::action::FollowJointTrajectory;
using Child = rclcpp_action::ServerGoalHandle<Fjt>;
using Goal = rclcpp_action::ServerGoalHandle<Action>;
void require(bool value, const std::string& detail) {
    if (!value) throw std::runtime_error(detail);
}
void check(Result r) { require(static_cast<bool>(r), r.message); }
void expect(Result r, ErrorCode code) {
    require(
        r.code == code,
        "unexpected result " + std::to_string(static_cast<int>(r.code)) + " " + r.message);
}
struct State {
    std::mutex mutex;
    std::condition_variable changed;
    enum Mode { Hold, Success, Abort, RemoteTimeout, Reject } mode{Hold};
    bool response{true}, finish{false};
    std::vector<std::shared_ptr<Goal>> goals;
    unsigned cancellations{0};
    bool stopped{false};
    std::vector<std::shared_ptr<Child>> children;
};
struct Fixture {
    std::shared_ptr<State> state = std::make_shared<State>();
    rclcpp::CallbackGroup::SharedPtr actions, publication;
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Server<Fjt>::SharedPtr child_server;
    rclcpp_action::Client<Fjt>::SharedPtr child_client;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stop;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
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
        actions = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        publication = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        const auto record = state;
        child_server = rclcpp_action::create_server<Fjt>(
            node->get_node_base_interface(), node->get_node_clock_interface(),
            node->get_node_logging_interface(), node->get_node_waitables_interface(),
            "/left_arm_controller/follow_joint_trajectory",
            [](const auto&, const auto&) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
            [record](const auto& goal) {
                std::lock_guard<std::mutex> lock(record->mutex);
                record->children.push_back(goal);
            },
            rcl_action_server_get_default_options(), actions);
        child_client = rclcpp_action::create_client<Fjt>(
            node, "/left_arm_controller/follow_joint_trajectory", actions);
        rclcpp::SubscriptionOptions stop_options;
        stop_options.callback_group = actions;
        stop = node->create_subscription<std_msgs::msg::String>(
            "/trajectory_execution_event", 10,
            [record](const std_msgs::msg::String& event) {
                if (event.data != "stop") return;
                std::lock_guard<std::mutex> lock(record->mutex);
                record->stopped = true;
                ++record->cancellations;
                record->changed.notify_all();
            },
            stop_options);
        const auto child_sender = child_client;
        server = rclcpp_action::create_server<Action>(
            node->get_node_base_interface(), node->get_node_clock_interface(),
            node->get_node_logging_interface(), node->get_node_waitables_interface(),
            "/execute_trajectory",
            [record](const auto&, const auto&) {
                std::unique_lock<std::mutex> lock(record->mutex);
                if (!record->changed.wait_for(lock, 5s, [&] { return record->response; }))
                    return rclcpp_action::GoalResponse::REJECT;
                return record->mode == State::Reject
                           ? rclcpp_action::GoalResponse::REJECT
                           : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [record](const auto&) {
                std::lock_guard<std::mutex> lock(record->mutex);
                ++record->cancellations;
                record->changed.notify_all();
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [record, child_sender](const auto& goal) {
                {
                    std::lock_guard<std::mutex> lock(record->mutex);
                    record->goals.push_back(goal);
                    record->changed.notify_all();
                }
                child_sender->async_send_goal(Fjt::Goal{});
            },
            rcl_action_server_get_default_options(), actions);
        moveit::core::RobotState home(model);
        home.setToDefaultValues();
        home.setToDefaultValues(model->getJointModelGroup("dual_arm_spine"), "home");
        home.setVariablePosition("left_fr3v2_1_finger_joint1", .035);
        home.setVariablePosition("right_fr3v2_1_finger_joint1", .035);
        home.update();
        auto message = std::make_shared<sensor_msgs::msg::JointState>();
        message->name = model->getVariableNames();
        for (const auto& name : message->name)
            message->position.push_back(home.getVariablePosition(name));
        message->velocity.resize(message->name.size(), 0);
        publisher = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        const auto output = publisher;
        const auto clock = node->get_clock();
        timer = node->create_wall_timer(
            5ms,
            [record, message, output, clock] {
                message->header.stamp = clock->now();
                output->publish(*message);
                std::lock_guard<std::mutex> lock(record->mutex);
                if (record->children.empty()) return;
                const bool normal = record->mode != State::Hold;
                if (!normal && !(record->finish && record->stopped)) return;
                for (const auto& child : record->children)
                    if (child->is_active()) {
                        auto result = std::make_shared<Fjt::Result>();
                        result->error_code = Fjt::Result::SUCCESSFUL;
                        child->succeed(result);
                    }
                for (const auto& goal : record->goals)
                    if (goal->is_active()) {
                        auto result = std::make_shared<Action::Result>();
                        result->error_code.val =
                            normal ? (record->mode == State::Success ? result->error_code.SUCCESS
                                      : record->mode == State::RemoteTimeout
                                          ? result->error_code.TIMED_OUT
                                          : result->error_code.CONTROL_FAILED)
                                   : result->error_code.PREEMPTED;
                        if (record->mode == State::Success)
                            goal->succeed(result);
                        else
                            goal->abort(result);
                    }
            },
            publication);
    }
    void prepare(State::Mode mode, bool response, bool finish) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->mode = mode;
        state->response = response;
        state->finish = finish;
        state->goals.clear();
        state->cancellations = 0;
        state->stopped = false;
        state->children.clear();
    }
    void release(bool response, bool finish) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->response = response;
        state->finish = finish;
        state->changed.notify_all();
    }
    void accepted(std::size_t count) {
        std::unique_lock<std::mutex> lock(state->mutex);
        require(
            state->changed.wait_for(lock, 5s, [&] { return state->goals.size() >= count; }),
            "owned goal not accepted");
    }
};
void ready(MoveGroup& move) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!move.is_ready() && std::chrono::steady_clock::now() < deadline) {
        std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);
        std::condition_variable pending;
        pending.wait_for(lock, 5ms);
    }
    require(move.is_ready(), "late terminal/rejection did not converge");
}
void run(const rclcpp::Node::SharedPtr& node, Fixture& fixture) {
    MoveGroup move(node);
    check(move.initialize(40s));
    check(move.add_group(RobotGroup::LeftArm));
    check(move.add_joint_position_target(
        RobotGroup::LeftArm,
        std::vector<double>{.0661, -1.2125, -.607, -.9266, .001, .8389, .0281}));
    Plan plan;
    check(move.plan(plan));
    fixture.prepare(State::Success, true, false);
    check(move.execute(plan));
    check(move.stop());
    fixture.prepare(State::Abort, true, false);
    expect(move.execute(plan), ErrorCode::ExecutionFailed);
    fixture.prepare(State::RemoteTimeout, true, false);
    expect(move.execute(plan), ErrorCode::ExecutionFailed);
    fixture.prepare(State::Hold, true, true);
    expect(move.execute(plan), ErrorCode::Timeout);
    check(move.initialize(5s));
    fixture.prepare(State::Hold, false, true);
    expect(move.execute(plan), ErrorCode::CancelFailed);
    require(!move.is_ready(), "pending response marked ready");
    expect(move.execute(plan), ErrorCode::PreviousOperationNotTerminated);
    fixture.release(true, true);
    ready(move);
    {
        std::lock_guard<std::mutex> lock(fixture.state->mutex);
        require(
            fixture.state->cancellations >= 1, "late accepted goal did not receive official stop");
    }
    fixture.prepare(State::Reject, false, false);
    expect(move.execute(plan), ErrorCode::CancelFailed);
    fixture.release(true, false);
    ready(move);
    fixture.prepare(State::Hold, true, false);
    auto running = std::async(std::launch::async, [&] { return move.execute(plan); });
    fixture.accepted(1);
    expect(move.stop(), ErrorCode::CancelFailed);
    require(
        running.wait_for(3s) == std::future_status::ready, "unknown execution wait did not settle");
    expect(running.get(), ErrorCode::CancelFailed);
    require(!move.is_ready(), "parent alone marked execution ready");
    expect(move.execute(plan), ErrorCode::PreviousOperationNotTerminated);
    MoveGroup other(node);
    check(other.initialize(5s));
    expect(other.execute(plan), ErrorCode::PreviousOperationNotTerminated);
    fixture.release(true, true);
    ready(move);
    // Owner destruction during pending response must still cancel late Accepted.
    fixture.prepare(State::Hold, false, true);
    auto owner = std::make_unique<MoveGroup>(node);
    check(owner->initialize(30s));
    expect(owner->execute(plan), ErrorCode::CancelFailed);
    owner.reset();
    fixture.release(true, true);
    {
        std::unique_lock<std::mutex> lock(fixture.state->mutex);
        require(
            fixture.state->changed.wait_for(
                lock, 5s, [&] { return fixture.state->cancellations >= 1; }),
            "destroyed owner lost late goal cancellation");
    }
    ready(move);
    fixture.prepare(State::Success, true, false);
    check(move.execute(plan));
    std::cout << "MOVEIT_ACTION_PASS response/late accept/reject, ACK != terminal, "
                 "unknown/recovery, exclusive shared channel, owner destruction"
              << std::endl;
}
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("moveit_action_probe", options);
    node->set_parameter(rclcpp::Parameter("execution.goal_response_timeout", .05));
    node->set_parameter(rclcpp::Parameter("execution.cancel_timeout", .03));
    node->set_parameter(rclcpp::Parameter("execution.terminal_timeout", .3));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    std::unique_ptr<Fixture> fixture;
    int status = 0;
    try {
        robot_model_loader::RobotModelLoader loader(node);
        require(loader.getModel() != nullptr, "fixture model missing");
        fixture = std::make_unique<Fixture>(node, loader.getModel());
        run(node, *fixture);
    } catch (const std::exception& e) {
        std::cerr << "MOVEIT_ACTION_FAIL " << e.what() << std::endl;
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
