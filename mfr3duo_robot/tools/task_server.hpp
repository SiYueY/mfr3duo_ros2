#pragma once

#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mfr3duo_msgs/action/execute_task.hpp>
#include <mfr3duo_msgs/msg/robot_status.hpp>
#include <mfr3duo_msgs/srv/control_lease.hpp>
#include <mfr3duo_msgs/srv/plan_task.hpp>
#include "mfr3duo_robot/robot.hpp"

namespace mfr3duo_robot {
// ROS transport for the existing SDK, hosted by the existing task application.
// The application owns and spins the four-worker executor throughout teardown.
class TaskServer {
    using Action = mfr3duo_msgs::action::ExecuteTask;
    using Goal = rclcpp_action::ServerGoalHandle<Action>;
    using Lease = mfr3duo_msgs::srv::ControlLease;
    using PlanTask = mfr3duo_msgs::srv::PlanTask;
    using Clock = std::chrono::steady_clock;

public:
    explicit TaskServer(rclcpp::Node::SharedPtr node) : node_(std::move(node)), robot_(node_) {
        velocity_ =
            node_->create_publisher<geometry_msgs::msg::Twist>("/tmr_controller/cmd_vel", 1);
        status_ = node_->create_publisher<mfr3duo_msgs::msg::RobotStatus>("/robot/status", 10);
        lease_ = node_->create_service<Lease>(
            "/robot/control",
            [this](
                const Lease::Request::SharedPtr request,
                const Lease::Response::SharedPtr response) { command(*request, *response); });
        plan_ = node_->create_service<PlanTask>(
            "/robot/plan_task",
            [this](const PlanTask::Request::SharedPtr request,
                   const PlanTask::Response::SharedPtr response) { plan(*request, *response); });
        action_ = rclcpp_action::create_server<Action>(
            node_, "/robot/execute_task",
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> goal) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_ || busy_ || !robot_.is_ready() || authority_.empty() ||
                    goal->authority_id != authority_ || mode_ != "task" ||
                    Clock::now() >= expires_ || goal->steps.empty() || goal->steps.size() > 32 ||
                    !duration_valid(goal->timeout_s))
                    return rclcpp_action::GoalResponse::REJECT;
                for (const auto& step : goal->steps) {
                    if (!duration_valid(step.timeout_s) || step.manipulator > 2 ||
                        (step.kind != "navigate" && step.kind != "pick" && step.kind != "place" &&
                         step.kind != "scene_joint") ||
                        (step.kind != "navigate" && step.object_id.empty()) ||
                        ((step.kind == "navigate" || step.kind == "place") && !step.has_pose) ||
                        (step.kind == "scene_joint" && !std::isfinite(step.position)))
                        return rclcpp_action::GoalResponse::REJECT;
                }
                busy_ = true;  // Reserve before accepting a second goal or lease.
                cancel_requested_ = false;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<Goal> goal) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (active_goal_ != goal) return rclcpp_action::CancelResponse::REJECT;
                cancel_requested_ = true;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<Goal> goal) {
                if (worker_.joinable()) worker_.join();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    active_goal_ = goal;
                }
                worker_ = std::thread([this, goal] { execute(goal); });
            });
        timer_ =
            node_->create_wall_timer(std::chrono::milliseconds(100), [this] { publish_status(); });
        initialize(90.0);
    }

    ~TaskServer() {
        running_ = false;
        cancel_requested_ = true;
        if (worker_.joinable()) worker_.join();
        zero();
    }

private:
    static bool duration_valid(double value) {
        return std::isfinite(value) && value > 0 && value <= 3600;
    }
    static std::chrono::milliseconds duration(double value) {
        return std::chrono::milliseconds(static_cast<std::int64_t>(value * 1000));
    }
    static const char* error_code(TaskError error) {
        switch (error) {
            case TaskError::None:
                return "";
            case TaskError::InvalidTask:
                return "invalid_input";
            case TaskError::RobotBusy:
                return "control_authority_conflict";
            case TaskError::Timeout:
                return "robot_timeout";
            case TaskError::Canceled:
                return "run_cancelled";
            case TaskError::RobotNotReady:
                return "robot_not_ready";
            case TaskError::PreviousOperationNotTerminated:
            case TaskError::CancelFailed:
            case TaskError::RecoveryRequired:
                return "robot_recovery_required";
            default:
                return "robot_execution_failed";
        }
    }
    void zero() noexcept {
        try {
            velocity_->publish(geometry_msgs::msg::Twist{});
        } catch (...) {
        }
    }
    void initialize(double timeout) {
        if (worker_.joinable()) worker_.join();
        busy_ = true;
        worker_ = std::thread([this, timeout] {
            try {
                const auto result = robot_.initialize(duration(timeout));
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostic_ = result.message;
            } catch (const std::exception& error) {
                std::lock_guard<std::mutex> lock(mutex_);
                diagnostic_ = error.what();
            }
            busy_ = false;
        });
    }
    void command(const Lease::Request& request, Lease::Response& response) {
        response.error_code = "control_authority_conflict";
        response.message = "Robot is busy or control is owned by another client";
        std::unique_lock<std::mutex> lock(mutex_);
        const auto now = Clock::now();
        if (request.command == "initialize") {
            if (busy_ || !authority_.empty() || !duration_valid(request.timeout_s)) return;
            lock.unlock();
            initialize(request.timeout_s);
        } else if (request.command == "acquire") {
            if (robot_.state() == RobotState::Error) {
                response.error_code = "robot_recovery_required";
                return;
            }
            if (busy_ || (!authority_.empty() && now < expires_) || request.authority_id.empty() ||
                (request.mode != "task" && request.mode != "teleoperation") ||
                !duration_valid(request.ttl_s))
                return;
            if (request.mode == "task" && !robot_.is_ready()) {
                response.error_code = "robot_not_ready";
                return;
            }
            authority_ = request.authority_id;
            mode_ = request.mode;
            expires_ = now + duration(request.ttl_s);
        } else if (request.command == "renew") {
            if (request.authority_id != authority_ || now >= expires_ ||
                !duration_valid(request.ttl_s))
                return;
            expires_ = now + duration(request.ttl_s);
        } else if (request.command == "release" || request.command == "stop") {
            if (authority_.empty() || request.authority_id != authority_) {
                response.error_code = "control_authority_required";
                return;
            }
            cancel_requested_ = true;
            zero();
            if (request.command == "release") {
                authority_.clear();
                mode_ = "idle";
            }
        } else {
            response.error_code = "invalid_input";
            response.message = "Unknown robot control command";
            return;
        }
        response.success = true;
        response.error_code.clear();
        response.message.clear();
    }
    std::unique_ptr<RobotTask> task(const mfr3duo_msgs::msg::TaskStep& step) {
        std::unique_ptr<RobotTask> result;
        const auto hand = static_cast<Manipulator>(step.manipulator);
        if (step.kind == "navigate")
            result = std::make_unique<NavigateTask>(step.pose);
        else if (step.kind == "scene_joint")
            result = std::make_unique<SceneJointTask>(step.object_id, step.position);
        else if (step.kind == "pick") {
            auto pick = std::make_unique<PickTask>(step.object_id);
            pick->set_manipulator(hand);
            if (step.has_pose) pick->set_grasp_pose(step.pose);
            result = std::move(pick);
        } else {
            auto place = std::make_unique<PlaceTask>(step.object_id);
            place->set_manipulator(hand);
            place->set_place_pose(step.pose);
            result = std::move(place);
        }
        result->set_timeout(duration(step.timeout_s));
        return result;
    }
    void plan(const PlanTask::Request& request, PlanTask::Response& response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_ || !robot_.is_ready()) {
            response.error_code = busy_ ? "control_authority_conflict" : "robot_not_ready";
            response.message = busy_ ? "Robot is busy" : "Robot is not ready";
            return;
        }
        if (request.step.kind != "pick" || request.step.object_id.empty() ||
            request.step.manipulator > 2 || !duration_valid(request.step.timeout_s)) {
            response.error_code = "invalid_input";
            response.message = "PlanTask currently requires one valid pick step";
            return;
        }
        try {
            auto planned = task(request.step);
            const auto result = robot_.preflight(*planned);
            response.feasible = result.feasible;
            response.selected_manipulator = static_cast<std::uint8_t>(result.manipulator);
            response.error_code = result.feasible ? "" : error_code(result.error);
            response.message = result.message;
        } catch (const std::exception& error) {
            response.error_code = "robot_execution_failed";
            response.message = error.what();
        }
    }
    void execute(const std::shared_ptr<Goal>& goal) noexcept {
        auto response = std::make_shared<Action::Result>();
        try {
            auto sequence = std::make_unique<TaskSequence>();
            for (const auto& step : goal->get_goal()->steps) sequence->add(task(step));
            sequence->set_timeout(duration(goal->get_goal()->timeout_s));
            auto handle = robot_.start(std::move(sequence));
            const auto deadline = Clock::now() + duration(goal->get_goal()->timeout_s);
            while (!handle.result()) {
                if (!running_ || cancel_requested_ || Clock::now() >= deadline) {
                    handle.cancel();
                    break;
                }
                if (rclcpp::ok(node_->get_node_base_interface()->get_context())) {
                    auto feedback = std::make_shared<Action::Feedback>();
                    feedback->phase = "executing";
                    feedback->task_state = static_cast<std::uint8_t>(handle.state());
                    goal->publish_feedback(feedback);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            const auto result = handle.wait();
            response->success = static_cast<bool>(result);
            response->error_code = error_code(result.error);
            response->message = result.message;
            response->termination_confirmed = robot_.state() != RobotState::Error;
            if (rclcpp::ok(node_->get_node_base_interface()->get_context())) {
                if (goal->is_canceling())
                    goal->canceled(response);
                else if (response->success)
                    goal->succeed(response);
                else
                    goal->abort(response);
            }
        } catch (const std::exception& error) {
            response->error_code = "robot_recovery_required";
            response->message = error.what();
            response->termination_confirmed = false;
            try {
                if (goal->is_active()) goal->abort(response);
            } catch (...) {
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            diagnostic_ = response->message;
            active_goal_.reset();
            busy_ = false;
        }
    }
    void publish_status() {
        mfr3duo_msgs::msg::RobotStatus message;
        message.header.stamp = node_->now();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!authority_.empty() && Clock::now() >= expires_) {
                cancel_requested_ = true;
                zero();
                authority_.clear();
                mode_ = "idle";
            }
            message.authority_id = authority_;
            message.mode = mode_;
            message.diagnostic = diagnostic_;
        }
        message.ready = robot_.is_ready();
        message.busy = busy_;
        message.state = static_cast<std::uint8_t>(robot_.state());
        status_->publish(message);
    }

    rclcpp::Node::SharedPtr node_;
    Robot robot_;
    std::mutex mutex_;
    std::atomic<bool> running_{true}, busy_{false}, cancel_requested_{false};
    std::thread worker_;
    std::string authority_, mode_{"idle"}, diagnostic_;
    Clock::time_point expires_{};
    std::shared_ptr<Goal> active_goal_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
    rclcpp::Publisher<mfr3duo_msgs::msg::RobotStatus>::SharedPtr status_;
    rclcpp::Service<Lease>::SharedPtr lease_;
    rclcpp::Service<PlanTask>::SharedPtr plan_;
    rclcpp_action::Server<Action>::SharedPtr action_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace mfr3duo_robot
