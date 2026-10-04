// Phase 2B numerical/planning POC; cancellation uses the application execution fence.
#include "mfr3duo_moveit/move_group.hpp"
#include <std_msgs/msg/string.hpp>
#include <future>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <ompl/util/RandomNumbers.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <thread>
#include <atomic>

namespace {
using Clock = std::chrono::steady_clock;
using Execute = moveit_msgs::action::ExecuteTrajectory;
using ExecuteClient = rclcpp_action::Client<Execute>;
constexpr const char* kSpine = "franka_spine_vertical_joint";
void require(bool condition, const std::string& detail) {
    if (!condition) throw std::runtime_error(detail);
}
geometry_msgs::msg::Pose message(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond q(transform.rotation());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}
struct Case {
    std::string id, gate, group;
    double spine{0.0};
    bool high{false};
    std::map<std::string, Eigen::Isometry3d> poses;
};
moveit::core::RobotState home_state(const moveit::core::RobotModelPtr& model) {
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    require(
        state.setToDefaultValues(model->getJointModelGroup("dual_arm_spine"), "home"),
        "home absent");
    for (const auto* side : {"left", "right"})
        state.setVariablePosition(std::string(side) + "_fr3v2_1_finger_joint1", .035);
    state.update();
    return state;
}
std::vector<Case> cases(const YAML::Node& manifest, const moveit::core::RobotState& start) {
    std::vector<Case> result;
    for (const auto& entry : manifest["cases"]) {
        Case item;
        item.id = entry["id"].as<std::string>();
        item.gate = entry["gate"].as<std::string>();
        item.group = entry["group"].as<std::string>();
        item.spine = entry["spine"].as<double>();
        item.high = entry["high"] && entry["high"].as<bool>();
        auto goal = start;
        goal.setVariablePosition(kSpine, item.spine);
        for (const auto* side : {"left", "right"})
            if (entry[side])
                goal.setJointGroupPositions(
                    std::string(side) + "_arm", entry[side].as<std::vector<double>>());
        goal.update();
        require(goal.satisfiesBounds(), "construction bounds " + item.id);
        for (const auto* side : {"left", "right"})
            if (entry[side]) {
                const std::string tip = std::string(side) + "_fr3v2_1_hand_tcp";
                item.poses[tip] = goal.getGlobalLinkTransform(tip);
                if (item.high) {
                    const auto* link = start.getRobotModel()->getLinkModel(tip);
                    const std::string base = std::string(side) + "_fr3v2_1_link0";
                    double reach = 0;
                    while (link && link->getName() != base) {
                        reach += link->getJointOriginTransform().translation().norm();
                        link = link->getParentLinkModel();
                    }
                    require(link != nullptr, "arm chain base");
                    const double distance = (item.poses.at(tip).translation() -
                                             start.getGlobalLinkTransform(base).translation())
                                                .norm();
                    require(
                        distance > reach + .05,
                        "high target not proven outside fixed-arm reach " + item.id);
                    std::cout << "REACH_PROOF " << item.id << " distance=" << distance
                              << " upper_bound=" << reach << " margin=" << distance - reach
                              << std::endl;
                }
            }
        result.push_back(std::move(item));
    }
    return result;
}
moveit_msgs::msg::Constraints pose_constraints(
    const Case& item, const YAML::Node& manifest, const std::string& frame) {
    moveit_msgs::msg::Constraints merged;
    for (const auto& target : item.poses) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame;
        pose.pose = message(target.second);
        const auto constraint = kinematic_constraints::constructGoalConstraints(
            target.first, pose, manifest["position_tolerance"].as<double>(),
            manifest["orientation_tolerance"].as<double>());
        merged.position_constraints.insert(
            merged.position_constraints.end(), constraint.position_constraints.begin(),
            constraint.position_constraints.end());
        merged.orientation_constraints.insert(
            merged.orientation_constraints.end(), constraint.orientation_constraints.begin(),
            constraint.orientation_constraints.end());
    }
    if (item.gate == "C") {
        moveit_msgs::msg::JointConstraint joint;
        joint.joint_name = kSpine;
        joint.position = item.spine;
        joint.weight = 1;
        joint.tolerance_above = joint.tolerance_below = manifest["joint_tolerance"].as<double>();
        merged.joint_constraints.push_back(joint);
    }
    return merged;
}
bool single_ik(
    moveit::core::RobotState& state, const std::string& group, const Eigen::Isometry3d& target,
    std::mt19937& random, bool randomize) {
    const auto* jmg = state.getRobotModel()->getJointModelGroup(group);
    const auto& solver = jmg->getSolverInstance();
    require(solver != nullptr, "missing solver " + group);
    std::vector<double> seed;
    state.copyJointGroupPositions(jmg, seed);
    if (randomize)
        for (std::size_t i = 0; i < seed.size(); ++i) {
            const auto& bound =
                state.getRobotModel()->getVariableBounds(jmg->getVariableNames()[i]);
            seed[i] = std::uniform_real_distribution<double>(
                bound.min_position_, bound.max_position_)(random);
        }
    // KDL getPositionIK is one attempt. This avoids its unseeded retry RNG;
    // Eigen's singularity wiggle uses std::rand, seeded once per trial below.
    const auto local = state.getGlobalLinkTransform(solver->getBaseFrame()).inverse() * target;
    std::vector<double> values;
    moveit_msgs::msg::MoveItErrorCodes error;
    if (!solver->getPositionIK(message(local), seed, values, error) || error.val != error.SUCCESS)
        return false;
    state.setJointGroupPositions(jmg, values);
    state.update();
    return state.satisfiesBounds(jmg);
}
bool sample(
    const Case& item, const YAML::Node& manifest, const planning_scene::PlanningScene& scene,
    const moveit::core::RobotState& start, unsigned seed, moveit::core::RobotState& goal,
    std::string& failure) {
    std::mt19937 random(seed);
    std::srand(seed);
    const auto deadline =
        Clock::now() + std::chrono::duration<double>(manifest["sampling_timeout"].as<double>());
    const auto constraints =
        pose_constraints(item, manifest, start.getRobotModel()->getModelFrame());
    bool solved = false;
    for (unsigned attempt = 0; attempt < 200 && Clock::now() < deadline; ++attempt) {
        auto candidate = start;
        if (item.gate == "C") candidate.setVariablePosition(kSpine, item.spine);
        candidate.update();
        bool valid = true;
        for (const auto& target : item.poses) {
            const std::string arm = target.first.substr(0, target.first.find('_')) + "_arm";
            if (!single_ik(
                    candidate, item.gate == "A" ? item.group : arm, target.second, random,
                    attempt > 0)) {
                valid = false;
                break;
            }
        }
        if (!valid) continue;
        solved = true;
        // Both poses, shared explicit spine, limits and entire self/world scene
        // must pass together before this sample can be used by the planner.
        if (item.gate == "B" &&
            std::abs(candidate.getVariablePosition(kSpine) - start.getVariablePosition(kSpine)) >
                1e-12)
            continue;
        if (!candidate.satisfiesBounds() || !scene.isStateValid(candidate, constraints, ""))
            continue;
        // Redundant IK must leave the existing execution-error envelope inside
        // the physical range, instead of choosing a saturated mechanical stop.
        bool margin = true;
        for (const auto& name :
             start.getRobotModel()->getJointModelGroup(item.group)->getVariableNames()) {
            if (name == kSpine && item.gate != "A") continue;
            const auto& bounds = start.getRobotModel()->getVariableBounds(name);
            const double reserve =
                manifest
                    [name == kSpine ? "execution_spine_joint_error" : "execution_arm_joint_error"]
                        .as<double>();
            const double position = candidate.getVariablePosition(name);
            if (bounds.position_bounded_ && (position < bounds.min_position_ + reserve ||
                                             position > bounds.max_position_ - reserve))
                margin = false;
        }
        if (!margin) continue;
        goal = candidate;
        return true;
    }
    failure = solved ? "GoalSamplingFailed" : "IKFailed";
    return false;
}
planning_interface::MotionPlanRequest request_for(
    const Case& item, const YAML::Node& manifest, const moveit::core::RobotState& start,
    const moveit::core::RobotState& goal) {
    planning_interface::MotionPlanRequest request;
    request.group_name = item.group;
    request.planner_id = "RRTConnectkConfigDefault";
    request.allowed_planning_time = manifest["planning_timeout"].as<double>();
    request.num_planning_attempts = 1;
    request.max_velocity_scaling_factor = .2;
    request.max_acceleration_scaling_factor = .2;
    moveit::core::robotStateToRobotStateMsg(start, request.start_state);
    auto constraint = kinematic_constraints::constructGoalConstraints(
        goal, start.getRobotModel()->getJointModelGroup(item.group),
        manifest["joint_tolerance"].as<double>());
    const auto poses = pose_constraints(item, manifest, start.getRobotModel()->getModelFrame());
    constraint.position_constraints = poses.position_constraints;
    constraint.orientation_constraints = poses.orientation_constraints;
    request.goal_constraints = {constraint};
    return request;
}
void negative_contract(
    const YAML::Node& manifest, const planning_scene::PlanningScenePtr& scene,
    const moveit::core::RobotState& start, const std::vector<Case>& targets) {
    const auto it = std::find_if(
        targets.begin(), targets.end(), [](const auto& item) { return item.id == "B_small"; });
    require(it != targets.end(), "negative fixture target");
    auto candidate = start;
    std::string failure;
    require(
        sample(*it, manifest, *scene, start, 1101, candidate, failure),
        "negative fixture valid baseline");
    const auto constraints =
        pose_constraints(*it, manifest, start.getRobotModel()->getModelFrame());
    require(
        constraints.position_constraints.size() == 2 &&
            constraints.orientation_constraints.size() == 2,
        "dual pose AND dimensions");
    auto wrong = candidate;
    wrong.setVariablePosition(
        "right_fr3v2_1_joint1", wrong.getVariablePosition("right_fr3v2_1_joint1") + .3);
    wrong.update();
    require(
        !scene->isStateConstrained(wrong, constraints), "right target violation accepted by AND");
    auto left_only = constraints;
    left_only.position_constraints.resize(1);
    left_only.orientation_constraints.resize(1);
    require(scene->isStateConstrained(wrong, left_only), "left-only fixture must remain satisfied");
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.id = "blocked_goal";
    obstacle.header.frame_id = start.getRobotModel()->getModelFrame();
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {.2, .2, .2};
    obstacle.primitives.push_back(box);
    geometry_msgs::msg::Pose pose;
    pose.position = message(it->poses.begin()->second).position;
    pose.orientation.w = 1;
    obstacle.primitive_poses.push_back(pose);
    obstacle.operation = obstacle.ADD;
    require(scene->processCollisionObjectMsg(obstacle), "negative world insertion");
    require(scene->isStateColliding(candidate), "blocked full state accepted");
    require(
        !sample(*it, manifest, *scene, start, 1101, candidate, failure) &&
            failure == "GoalSamplingFailed",
        "blocked target not diagnosed as GoalSamplingFailed");
    obstacle.operation = obstacle.REMOVE;
    require(scene->processCollisionObjectMsg(obstacle), "negative world removal");
    auto unreachable = *it;
    unreachable.poses.begin()->second.translation().z() += 5;
    require(
        !sample(unreachable, manifest, *scene, start, 1101, candidate, failure) &&
            failure == "IKFailed",
        "finite failed search must report IKFailed");
    std::cout << "NEGATIVE_CONTRACT_PASS left-only != AND, blocked world rejected, "
                 "IKFailed/GoalSamplingFailed distinguished"
              << std::endl;
}
void validate_path(
    const planning_scene::PlanningScene& scene, const moveit::core::RobotState& start,
    const Case& item, const YAML::Node& manifest, const robot_trajectory::RobotTrajectory& path) {
    const auto* group = start.getRobotModel()->getJointModelGroup(item.group);
    require(
        path.getGroupName() == item.group && path.getWayPointCount() >= 2,
        "exact timed path group");
    double previous = -1;
    for (std::size_t i = 0; i < path.getWayPointCount(); ++i) {
        const auto& state = path.getWayPoint(i);
        require(
            state.satisfiesBounds() && scene.isStateValid(state),
            "full trajectory collision/bounds");
        const double time = path.getWayPointDurationFromStart(i);
        require(std::isfinite(time) && time > previous, "path time monotonic");
        previous = time;
        for (const auto& name : group->getVariableNames())
            require(
                std::isfinite(state.getVariableVelocity(name)) &&
                    std::abs(state.getVariableVelocity(name)) <=
                        start.getRobotModel()->getVariableBounds(name).max_velocity_ + 1e-5,
                "path velocity bounds");
        for (const auto& name :
             start.getRobotModel()->getJointModelGroup("dual_arm_spine")->getVariableNames())
            if (!group->hasJointModel(name))
                require(
                    std::abs(state.getVariablePosition(name) - start.getVariablePosition(name)) <
                        1e-12,
                    "inactive path variable");
    }
    require(
        scene.isStateConstrained(
            path.getLastWayPoint(),
            pose_constraints(item, manifest, start.getRobotModel()->getModelFrame())),
        "endpoint AND constraint violation");
}
void execute(
    const rclcpp::Node::SharedPtr& node, const ExecuteClient::SharedPtr& client,
    const robot_trajectory::RobotTrajectory& path, const moveit::core::RobotState& start,
    planning_scene_monitor::CurrentStateMonitor& monitor,
    const planning_scene::PlanningScene& scene, const YAML::Node& manifest,
    bool cancel_probe = false) {
    if (cancel_probe) {
        using namespace mfr3duo_moveit;
        auto check = [](Result result) { require(static_cast<bool>(result), result.message); };
        MoveGroup move(node);
        check(move.initialize(std::chrono::seconds(30)));
        for (const auto& item : std::array<std::pair<RobotGroup, const char*>, 3>{
                 {{RobotGroup::LeftArm, "left_arm"},
                  {RobotGroup::RightArm, "right_arm"},
                  {RobotGroup::Spine, "spine"}}}) {
            const auto* group = start.getRobotModel()->getJointModelGroup(item.second);
            if (!path.getGroup()->hasJointModel(group->getActiveJointModelNames().front()))
                continue;
            check(move.add_group(item.first));
            std::vector<double> values;
            path.getLastWayPoint().copyJointGroupPositions(group, values);
            if (item.first == RobotGroup::Spine)
                check(move.add_joint_position_target(item.first, values.front()));
            else
                check(move.add_joint_position_target(item.first, values));
        }
        Plan plan;
        check(move.plan(plan));
        auto running = std::async(std::launch::async, [&] { return move.execute(plan); });
        require(
            running.wait_for(std::chrono::milliseconds(400)) == std::future_status::timeout,
            "cancellation motion ended early");
        auto stopped = move.stop();
        require(stopped.code == ErrorCode::Canceled, "official stop failed: " + stopped.message);
        require(
            running.get().code == ErrorCode::Canceled && move.is_ready(),
            "application termination fence failed");
        std::cout << "TEM_REAL_CANCEL_PASS official stop, child results and measured stop confirmed"
                  << std::endl;
        return;
    }
    const auto stop =
        node->create_publisher<std_msgs::msg::String>("/trajectory_execution_event", 10);
    auto send_stop = [stop] {
        std_msgs::msg::String event;
        event.data = "stop";
        stop->publish(event);
    };
    require(client->wait_for_action_server(std::chrono::seconds(10)), "TEM action not available");
    auto stopping = std::make_shared<std::atomic<bool>>(false);
    ExecuteClient::SendGoalOptions options;
    options.goal_response_callback = [send_stop, stopping](auto handle) {
        if (handle && stopping->load()) send_stop();
    };
    Execute::Goal goal;
    path.getRobotTrajectoryMsg(goal.trajectory);
    auto accepted = client->async_send_goal(goal, options);
    if (accepted.wait_for(std::chrono::duration<double>(
            manifest["execution_goal_response_timeout"].as<double>())) !=
        std::future_status::ready) {
        stopping->store(true);
        throw std::runtime_error(
            "ExecutionFailed goal response timeout; late accepted goal will be canceled; test "
            "stops hardware");
    }
    const auto handle = accepted.get();
    require(handle != nullptr, "TEM goal rejected");
    auto result = client->async_get_result(handle);
    const auto deadline = Clock::now() + std::chrono::duration<double>(2 * path.getDuration() + 5);
    std::exception_ptr validation_failure;
    while (result.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready &&
           Clock::now() < deadline) {
        try {
            require(
                monitor.haveCompleteState(rclcpp::Duration::from_seconds(1)),
                "execution state stale");
            const auto actual = monitor.getCurrentState();
            actual->update();
            if (!actual->satisfiesBounds()) {
                for (const auto& name : actual->getRobotModel()->getVariableNames()) {
                    const auto& bounds = actual->getRobotModel()->getVariableBounds(name);
                    const double value = actual->getVariablePosition(name);
                    if (bounds.position_bounded_ &&
                        (value < bounds.min_position_ || value > bounds.max_position_))
                        std::cerr << "BOUND_VIOLATION " << name << " actual=" << value
                                  << " min=" << bounds.min_position_
                                  << " max=" << bounds.max_position_ << std::endl;
                }
                throw std::runtime_error("actual execution bounds");
            }
            require(scene.isStateValid(*actual), "actual execution collision");
            for (const auto& name :
                 start.getRobotModel()->getJointModelGroup("dual_arm_spine")->getVariableNames())
                if (!path.getGroup()->hasJointModel(name))
                    require(
                        std::abs(
                            actual->getVariablePosition(name) - start.getVariablePosition(name)) <
                            manifest["inactive_joint_max_error"].as<double>(),
                        "actual inactive variable moved " + name);
        } catch (const std::exception& error) {
            std::cerr << "RUNTIME_VALIDATION_FAIL " << error.what() << std::endl;
            validation_failure = std::current_exception();
            break;
        }
    }
    if (validation_failure ||
        result.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        stopping->store(true);
        send_stop();
        // Only terminal result confirms termination. Cancellation acknowledgement
        // cannot allow the next test/trajectory to start.
        const bool terminal = result.wait_for(std::chrono::duration<double>(
                                  manifest["execution_terminal_cancel_timeout"].as<double>())) ==
                              std::future_status::ready;
        if (!terminal)
            throw std::runtime_error(
                "ExecutionFailed CancelFailed/TerminationUnknown; test stops hardware");
        if (validation_failure) std::rethrow_exception(validation_failure);
        throw std::runtime_error("ExecutionFailed deadline; terminal cancellation confirmed");
    }
    const auto terminal = result.get();
    require(
        terminal.code == rclcpp_action::ResultCode::SUCCEEDED && terminal.result &&
            terminal.result->error_code.val == terminal.result->error_code.SUCCESS,
        "ExecutionFailed TEM terminal code=" +
            std::to_string(terminal.result ? terminal.result->error_code.val : 0));
    std::cout << "TEM_TERMINAL_SUCCESS duration=" << path.getDuration() << std::endl;
}
void runtime(
    const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model,
    const YAML::Node& manifest, const ExecuteClient::SharedPtr& client) {
    planning_scene_monitor::CurrentStateMonitor monitor(node, model, {}, false);
    monitor.enableCopyDynamics(true);
    monitor.startStateMonitor();
    auto service = node->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers");
    const auto deadline = Clock::now() + std::chrono::seconds(40);
    bool ready = false;
    while (Clock::now() < deadline) {
        if (!service->wait_for_service(std::chrono::milliseconds(100))) continue;
        auto response = service->async_send_request(
            std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
        if (response.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            service->remove_pending_request(response);
            continue;
        }
        const auto message = response.get();
        ready = std::count_if(
                    message->controller.begin(), message->controller.end(),
                    [](const auto& controller) { return controller.state == "active"; }) == 8;
        if (ready) break;
    }
    require(ready && monitor.waitForCompleteState(10), "StateUnavailable complete/active runtime");
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    planning_pipeline::PlanningPipeline pipeline(model, node, "ompl");
    const auto nominal = home_state(model);
    const auto targets = cases(manifest, nominal);
    const auto selected = manifest["runtime_cases"].as<std::vector<std::string>>();
    auto reset_home = [&] {
        auto before = monitor.getCurrentState();
        before->update();
        auto goal = *before;
        std::vector<double> values;
        nominal.copyJointGroupPositions("dual_arm_spine", values);
        goal.setJointGroupPositions("dual_arm_spine", values);
        goal.update();
        scene->setCurrentState(*before);
        std::vector<double> actual;
        before->copyJointGroupPositions("dual_arm_spine", actual);
        bool already_home = actual.size() == values.size();
        for (std::size_t i = 0; i < actual.size() && already_home; ++i)
            already_home = std::abs(actual[i] - values[i]) <= .001;
        if (already_home) {
            require(
                before->satisfiesBounds() && scene->isStateValid(*before),
                "already-home state remains bounded and collision free");
            std::cout << "RESET_HOME_ALREADY_AT_TARGET" << std::endl;
            return;
        }
        Case item;
        item.id = "runtime_home";
        item.group = "dual_arm_spine";
        item.gate = "home";
        const auto request = request_for(item, manifest, *before, goal);
        planning_interface::MotionPlanResponse response;
        require(
            pipeline.generatePlan(scene, request, response) &&
                response.error_code_.val == response.error_code_.SUCCESS && response.trajectory_,
            "runtime home planning");
        validate_path(*scene, *before, item, manifest, *response.trajectory_);
        execute(node, client, *response.trajectory_, *before, monitor, *scene, manifest);
    };
    reset_home();
    {
        auto before = monitor.getCurrentState();
        before->update();
        scene->setCurrentState(*before);
        auto goal = *before;
        std::string failure;
        const auto& item = targets.front();
        require(
            sample(item, manifest, *scene, *before, 1101, goal, failure),
            "real cancellation sample " + failure);
        planning_interface::MotionPlanResponse response;
        require(
            pipeline.generatePlan(scene, request_for(item, manifest, *before, goal), response) &&
                response.error_code_.val == response.error_code_.SUCCESS && response.trajectory_,
            "real cancellation planning");
        validate_path(*scene, *before, item, manifest, *response.trajectory_);
        execute(node, client, *response.trajectory_, *before, monitor, *scene, manifest, true);
        reset_home();
    }
    for (const auto& id : selected) {
        const auto it = std::find_if(
            targets.begin(), targets.end(), [&](const auto& item) { return item.id == id; });
        require(it != targets.end(), "runtime case unknown");
        require(
            monitor.haveCompleteState(rclcpp::Duration::from_seconds(1)), "runtime start stale");
        const auto before = monitor.getCurrentState();
        before->update();
        scene->setCurrentState(*before);
        auto goal = *before;
        std::string failure;
        require(
            sample(*it, manifest, *scene, *before, 1101, goal, failure),
            "runtime " + id + " " + failure);
        const auto request = request_for(*it, manifest, *before, goal);
        planning_interface::MotionPlanResponse response;
        require(
            pipeline.generatePlan(scene, request, response) &&
                response.error_code_.val == response.error_code_.SUCCESS && response.trajectory_,
            "runtime path " + id);
        validate_path(*scene, *before, *it, manifest, *response.trajectory_);
        execute(node, client, *response.trajectory_, *before, monitor, *scene, manifest);
        const auto actual = monitor.getCurrentState();
        actual->update();
        double position_error = 0, angle_error = 0;
        for (const auto& target : it->poses) {
            const auto& pose = actual->getGlobalLinkTransform(target.first);
            position_error =
                std::max(position_error, (pose.translation() - target.second.translation()).norm());
            angle_error = std::max(
                angle_error,
                Eigen::AngleAxisd(target.second.rotation().transpose() * pose.rotation()).angle());
        }
        require(
            position_error <= manifest["execution_position_tolerance"].as<double>() &&
                angle_error <= manifest["execution_orientation_tolerance"].as<double>(),
            "actual TCP error " + id);
        const double spine_delta =
            actual->getVariablePosition(kSpine) - before->getVariablePosition(kSpine);
        if (it->high) require(spine_delta > .1, "actual high target spine did not move");
        if (it->gate == "B")
            require(
                std::abs(spine_delta) < manifest["inactive_joint_max_error"].as<double>(),
                "Gate B actual spine moved");
        if (it->gate == "C")
            require(
                std::abs(actual->getVariablePosition(kSpine) - it->spine) <
                    manifest["execution_spine_joint_error"].as<double>(),
                "Gate C actual spine target error");
        const auto* group = model->getJointModelGroup(it->group);
        for (const auto& name : group->getVariableNames())
            require(
                std::abs(actual->getVariablePosition(name) - goal.getVariablePosition(name)) <
                    manifest
                        [name == kSpine ? "execution_spine_joint_error"
                                        : "execution_arm_joint_error"]
                            .as<double>(),
                "actual joint target error " + name);
        std::cout << "GATE_RUNTIME_PASS " << id << " TCP_error=" << position_error
                  << " angle_error=" << angle_error << " spine_delta=" << spine_delta << std::endl;
        reset_home();
    }
    monitor.stopStateMonitor();
}

void run(
    const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model,
    const YAML::Node& manifest, bool explore) {
    auto start = home_state(model);
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->setCurrentState(start);
    require(start.satisfiesBounds() && scene->isStateValid(start), "initial full state invalid");
    const auto targets = cases(manifest, start);
    negative_contract(manifest, scene, start, targets);
    std::unique_ptr<planning_pipeline::PlanningPipeline> pipeline;
    if (!explore)
        pipeline = std::make_unique<planning_pipeline::PlanningPipeline>(model, node, "ompl");
    const auto seeds = manifest["ik_trial_seeds"].as<std::vector<unsigned>>();
    bool all_passed = true;
    for (const auto& item : targets) {
        unsigned passed = 0;
        std::vector<double> times;
        for (unsigned trial = 0; trial < (explore ? 1u : seeds.size()); ++trial) {
            auto goal = start;
            std::string failure;
            const auto begin = Clock::now();
            if (!sample(item, manifest, *scene, start, seeds[trial], goal, failure)) {
                std::cout << "TRIAL_FAIL " << item.id << " seed=" << seeds[trial] << " " << failure
                          << std::endl;
                continue;
            }
            if (item.high)
                require(
                    goal.getVariablePosition(kSpine) - start.getVariablePosition(kSpine) > .1,
                    "high sample did not use spine");
            if (!explore) {
                const auto request = request_for(item, manifest, start, goal);
                const auto poses = pose_constraints(item, manifest, model->getModelFrame());
                planning_interface::MotionPlanResponse response;
                const auto plan_begin = Clock::now();
                const bool success = pipeline->generatePlan(scene, request, response);
                const double elapsed =
                    std::chrono::duration<double>(Clock::now() - plan_begin).count();
                times.push_back(elapsed);
                if (!success || response.error_code_.val != response.error_code_.SUCCESS ||
                    !response.trajectory_) {
                    std::cout << "TRIAL_FAIL " << item.id << " seed=" << seeds[trial] << " "
                              << (response.error_code_.val == response.error_code_.TIMED_OUT
                                      ? "PlanningTimeout"
                                      : "PathPlanningFailed")
                              << " code=" << response.error_code_.val << std::endl;
                    continue;
                }
                const auto* group = model->getJointModelGroup(item.group);
                require(response.trajectory_->getGroupName() == item.group, "superset trajectory");
                for (std::size_t i = 0; i < response.trajectory_->getWayPointCount(); ++i) {
                    const auto& point = response.trajectory_->getWayPoint(i);
                    require(
                        point.satisfiesBounds() && scene->isStateValid(point),
                        "planned full-state limits/collision");
                    for (const auto& name :
                         model->getJointModelGroup("dual_arm_spine")->getVariableNames())
                        if (!group->hasJointModel(name))
                            require(
                                std::abs(
                                    point.getVariablePosition(name) -
                                    start.getVariablePosition(name)) < 1e-12,
                                "inactive trajectory joint " + name);
                }
                require(
                    scene->isStateConstrained(response.trajectory_->getLastWayPoint(), poses),
                    "endpoint AND constraints");
                std::cout << "TRIAL_PLAN " << item.id << " seed=" << seeds[trial]
                          << " plan_wall=" << elapsed << std::endl;
            }
            ++passed;
            std::cout << "TRIAL_SAMPLE " << item.id << " seed=" << seeds[trial]
                      << " spine=" << goal.getVariablePosition(kSpine) << " total_wall="
                      << std::chrono::duration<double>(Clock::now() - begin).count() << std::endl;
        }
        const unsigned count = explore ? 1 : seeds.size();
        std::sort(times.begin(), times.end());
        const double p95 =
            times.empty() ? 0 : times[static_cast<std::size_t>(std::ceil(times.size() * .95)) - 1];
        const bool pass =
            static_cast<double>(passed) / count >= manifest["minimum_success_rate"].as<double>() &&
            p95 <= manifest["p95_planning_wall_time_limit"].as<double>();
        all_passed = all_passed && pass;
        std::cout << "CASE_RESULT " << item.id << " success=" << passed << "/" << count
                  << " p95_plan=" << p95 << std::endl;
    }
    require(
        all_passed,
        explore ? "exploration sampler construction failed" : "Gate 2 manifest criteria failed");
    std::cout << (explore ? "GATE2_EXPLORE_COMPLETE" : "GATE2_PLANNING_PASS") << std::endl;
}
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "phase2b_joint_probe",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    auto execute_client = rclcpp_action::create_client<Execute>(node, "/execute_trajectory");
    int status = 0;
    try {
        const auto manifest = YAML::LoadFile(node->get_parameter("gate2_manifest").as_string());
        ompl::RNG::setSeed(manifest["ompl_process_seed"].as<unsigned>());
        robot_model_loader::RobotModelLoader loader(node);
        require(loader.getModel() != nullptr, "model unavailable");
        const auto mode = node->get_parameter("joint_probe_mode").as_string();
        run(node, loader.getModel(), manifest, mode == "explore");
        if (mode == "acceptance") {
            runtime(node, loader.getModel(), manifest, execute_client);
            std::cout << "GATE2_PASS" << std::endl;
        }
    } catch (const std::exception& error) {
        std::cerr << "GATE2_PROBE_FAIL " << error.what() << std::endl;
        status = 1;
    }
    executor.cancel();
    spinner.join();
    executor.remove_node(node);
    execute_client.reset();
    node.reset();
    rclcpp::shutdown();
    return status;
}
