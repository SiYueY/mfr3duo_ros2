#include "mfr3duo_moveit/move_group.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <future>
#include <iostream>
#include <limits>
#include <thread>

namespace {
using namespace mfr3duo_moveit;
using namespace std::chrono_literals;
void require(bool good, const std::string& detail) {
    if (!good) throw std::runtime_error(detail);
}
void check(Result result) {
    require(
        static_cast<bool>(result),
        std::to_string(static_cast<int>(result.code)) + " " + result.message);
}
void expect(Result result, ErrorCode code) {
    require(
        result.code == code,
        "unexpected error " + std::to_string(static_cast<int>(result.code)) + " " + result.message);
}
moveit::core::RobotState measured(
    const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model) {
    auto service = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    require(service->wait_for_service(5s), "scene service unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 1023;
    auto future = service->async_send_request(request);
    require(future.wait_for(3s) == std::future_status::ready, "scene response deadline");
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(future.get()->scene.robot_state, state);
    state.update();
    return state;
}
geometry_msgs::msg::PoseStamped pose(
    const moveit::core::RobotState& state, const std::string& tip) {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = state.getRobotModel()->getModelFrame();
    const auto& t = state.getGlobalLinkTransform(tip);
    const Eigen::Quaterniond q(t.rotation());
    result.pose.position.x = t.translation().x();
    result.pose.position.y = t.translation().y();
    result.pose.position.z = t.translation().z();
    result.pose.orientation.x = q.x();
    result.pose.orientation.y = q.y();
    result.pose.orientation.z = q.z();
    result.pose.orientation.w = q.w();
    return result;
}
void obstacle(
    const rclcpp::Node::SharedPtr& node, const geometry_msgs::msg::PoseStamped& p, bool add) {
    auto service =
        node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    require(service->wait_for_service(5s), "apply scene unavailable");
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene.is_diff = true;
    moveit_msgs::msg::CollisionObject object;
    object.id = "phase3_tcp_negative";
    object.header = p.header;
    object.operation = add ? object.ADD : object.REMOVE;
    if (add) {
        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {.08, .08, .08};
        object.primitives.push_back(box);
        object.primitive_poses.push_back(p.pose);
    }
    request->scene.world.collision_objects.push_back(object);
    auto future = service->async_send_request(request);
    require(
        future.wait_for(3s) == std::future_status::ready && future.get()->success,
        "apply collision object failed");
}
void check_fresh_attachment(
    const rclcpp::Node::SharedPtr& node, MoveGroup& move, const Plan& plan,
    const moveit::core::RobotState& before) {
    auto future_state = before;
    const auto& trajectory = plan.trajectory().joint_trajectory;
    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
        future_state.setVariablePosition(
            trajectory.joint_names[i], trajectory.points.back().positions[i]);
    future_state.update();
    const std::string tip = "left_fr3v2_1_hand_tcp";
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tip;
    attached.touch_links = {tip};
    attached.object.id = "phase3_fresh_attachment";
    attached.object.header.frame_id = tip;
    attached.object.operation = attached.object.ADD;
    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = sphere.SPHERE;
    sphere.dimensions = {.03};
    attached.object.primitives = {sphere};
    moveit_msgs::msg::CollisionObject wall;
    wall.id = "phase3_attachment_destination";
    wall.header.frame_id = before.getRobotModel()->getModelFrame();
    wall.operation = wall.ADD;
    wall.primitives = {sphere};
    bool fixture = false;
    for (int axis = 0; axis < 3 && !fixture; ++axis) {
        for (double sign : {-1., 1.}) {
            Eigen::Vector3d offset = Eigen::Vector3d::Zero();
            offset[axis] = sign;
            geometry_msgs::msg::Pose local;
            local.position.x = offset.x();
            local.position.y = offset.y();
            local.position.z = offset.z();
            local.orientation.w = 1;
            attached.object.primitive_poses = {local};
            auto destination = pose(future_state, tip).pose;
            const auto position = future_state.getGlobalLinkTransform(tip) * offset;
            destination.position.x = position.x();
            destination.position.y = position.y();
            destination.position.z = position.z();
            wall.primitive_poses = {destination};
            planning_scene::PlanningScene scene(before.getRobotModel());
            scene.setCurrentState(before);
            require(scene.processCollisionObjectMsg(wall), "attachment obstacle fixture");
            if (!scene.isStateValid(before) || !scene.isStateValid(future_state)) continue;
            require(scene.processAttachedCollisionObjectMsg(attached), "attachment fixture");
            auto loaded_start = scene.getCurrentState();
            auto loaded_end = loaded_start;
            for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
                loaded_end.setVariablePosition(
                    trajectory.joint_names[i], trajectory.points.back().positions[i]);
            loaded_end.update();
            fixture = scene.isStateValid(loaded_start) && !scene.isStateValid(loaded_end);
            if (fixture) break;
        }
    }
    require(fixture, "no isolated fresh-attachment collision fixture");
    auto service =
        node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    require(service->wait_for_service(5s), "attachment apply unavailable");
    const auto apply = [&](bool add) {
        auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
        request->scene.is_diff = true;
        request->scene.robot_state.is_diff = true;
        if (!add) {
            attached.object.operation = attached.object.REMOVE;
            wall.operation = wall.REMOVE;
        }
        request->scene.robot_state.attached_collision_objects = {attached};
        request->scene.world.collision_objects = {wall};
        auto response = service->async_send_request(request);
        require(
            response.wait_for(3s) == std::future_status::ready && response.get()->success,
            "fresh attachment apply failed");
    };
    apply(true);
    expect(move.execute(plan), ErrorCode::InvalidPlan);
    apply(false);
    // Detach restores a world object; remove that representation separately.
    auto cleanup = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    cleanup->scene.is_diff = true;
    cleanup->scene.world.collision_objects = {attached.object};
    auto response = service->async_send_request(cleanup);
    require(
        response.wait_for(3s) == std::future_status::ready && response.get()->success,
        "attachment fixture cleanup failed");
}
void run(const rclcpp::Node::SharedPtr& node) {
    const auto left = RobotGroup::LeftArm, right = RobotGroup::RightArm, spine = RobotGroup::Spine;
    MoveGroup move(node);
    check(move.initialize(40s));
    require(move.is_ready(), "facade not ready");
    robot_model_loader::RobotModelLoader loader(node);
    const auto model = loader.getModel();
    require(model != nullptr, "model not copied from MoveGroup");
    Plan plan;
    expect(move.plan(plan), ErrorCode::NoTarget);
    require(!plan.valid(), "failure left valid plan");
    expect(move.execute(plan), ErrorCode::InvalidPlan);

    // Public sampler covers both free-spine chains and position/orientation-only targets.
    auto nominal = measured(node, model);
    for (const auto side : {left, right}) {
        check(move.add_groups({side, spine}));
        auto high = nominal;
        high.setVariablePosition("franka_spine_vertical_joint", .8);
        const std::vector<double> high_values =
            side == left
                ? std::vector<double>{-1.8261453686056142, .6639092553505206,  1.8546028842190814,
                                      -.17731476957725523, -.4405972164831544, 2.7035012965265453,
                                      .27624529664852604}
                : std::vector<double>{-1.6965583548428984, -.654967663272759,   .11084848371254985,
                                      -.376918723315828,   -2.1258522482367894, 2.6933640022487375,
                                      .9303804520386185};
        high.setJointGroupPositions(side == left ? "left_arm" : "right_arm", high_values);
        high.update();
        check(move.add_pose_target(
            side, pose(high, side == left ? "left_fr3v2_1_hand_tcp" : "right_fr3v2_1_hand_tcp")));
        check(move.plan(plan));
        require(
            plan.trajectory().joint_trajectory.joint_names.size() == 8,
            "free spine public plan not exact 8 DOF");
        check(move.clear_targets());
        check(move.clear_groups());
    }
    check(move.add_group(left));
    const auto current_pose = pose(nominal, "left_fr3v2_1_hand_tcp");
    geometry_msgs::msg::PointStamped position;
    position.header = current_pose.header;
    position.point = current_pose.pose.position;
    check(move.add_position_target(left, position));
    check(move.plan(plan));
    check(move.remove_target(left));
    geometry_msgs::msg::QuaternionStamped orientation;
    orientation.header = current_pose.header;
    orientation.quaternion = current_pose.pose.orientation;
    check(move.add_orientation_target(left, orientation));
    check(move.plan(plan));
    check(move.clear_targets());
    check(move.clear_groups());
    check(move.add_groups({left, right, spine}));
    auto before = measured(node, model);
    check(move.add_pose_target(left, pose(before, "left_fr3v2_1_hand_tcp")));
    check(move.add_pose_target(right, pose(before, "right_fr3v2_1_hand_tcp")));
    expect(move.plan(plan), ErrorCode::UnsupportedCombination);
    require(!plan.valid(), "Gate D produced plan");
    check(move.add_joint_position_target(
        spine, before.getVariablePosition("franka_spine_vertical_joint") + .02));
    check(move.plan(plan));
    require(
        plan.groups().size() == 3 && plan.trajectory().joint_trajectory.joint_names.size() == 15,
        "exact joint group/AND merge failed");
    auto bad_start = plan.start_state();
    bad_start.joint_state.name.push_back(bad_start.joint_state.name.front());
    bad_start.joint_state.position.push_back(0);
    expect(move.set_start_state(bad_start), ErrorCode::InvalidStartState);
    check(move.clear_targets());
    check(move.clear_groups());
    check(move.add_group(spine));
    check(move.add_joint_position_target(
        spine, before.getVariablePosition("franka_spine_vertical_joint") + .03));
    check(move.plan(plan));
    auto saved = plan;
    // Deliberately breach the const view in test copies to exercise execution's
    // defensive checks; the supported API exposes no trajectory mutator.
    for (int mutation = 0; mutation < 9; ++mutation) {
        auto damaged = saved;
        auto& trajectory =
            const_cast<moveit_msgs::msg::RobotTrajectory&>(damaged.trajectory()).joint_trajectory;
        if (mutation == 0) trajectory.joint_names[0] = "unknown_joint";
        if (mutation == 1) trajectory.joint_names.push_back(trajectory.joint_names.front());
        if (mutation == 2) trajectory.points.front().positions.clear();
        if (mutation == 3)
            trajectory.points.back().positions[0] = std::numeric_limits<double>::quiet_NaN();
        if (mutation == 4) trajectory.points.back().positions[0] = 10;
        if (mutation == 5)
            trajectory.points.back().time_from_start = trajectory.points.front().time_from_start;
        if (mutation == 6)
            trajectory.points.front().velocities = {std::numeric_limits<double>::infinity()};
        if (mutation == 7) const_cast<std::vector<RobotGroup>&>(damaged.groups()).push_back(spine);
        if (mutation == 8)
            const_cast<moveit_msgs::msg::RobotState&>(damaged.start_state())
                .joint_state.position.front() = std::numeric_limits<double>::quiet_NaN();
        expect(move.execute(damaged), ErrorCode::InvalidPlan);
    }

    check(move.clear_targets());
    check(move.clear_groups());
    check(move.add_group(left));
    check(move.set_max_velocity_scaling_factor(.2));
    check(move.execute(saved));  // Uses Plan groups even though builder is now LeftArm.
    expect(move.execute(saved), ErrorCode::ExecutionStartStateMismatch);
    before = measured(node, model);
    std::vector<double> arm;
    before.copyJointGroupPositions("left_arm", arm);
    arm[0] += .03;
    check(move.add_joint_position_target(left, arm));
    check(move.move());
    check(move.clear_targets());
    expect(
        move.add_joint_position_target(
            left, std::vector<double>(7, std::numeric_limits<double>::quiet_NaN())),
        ErrorCode::InvalidTarget);
    expect(move.add_named_target(left, "does_not_exist"), ErrorCode::InvalidTarget);
    check(move.add_named_target(left, "home"));
    check(move.remove_target(left));
    check(move.add_group(spine));
    before = measured(node, model);
    moveit_msgs::msg::Constraints constraint;
    moveit_msgs::msg::JointConstraint fixed;
    fixed.joint_name = "franka_spine_vertical_joint";
    fixed.position = before.getVariablePosition(fixed.joint_name);
    fixed.tolerance_above = fixed.tolerance_below = .001;
    fixed.weight = 1;
    constraint.joint_constraints.push_back(fixed);
    check(move.add_path_constraint(constraint));
    check(move.add_path_constraint(constraint));
    auto conflict = constraint;
    conflict.joint_constraints[0].position += .1;
    expect(move.add_path_constraint(conflict), ErrorCode::InvalidConstraint);
    auto start_pose = pose(before, "left_fr3v2_1_hand_tcp");
    auto end_pose = start_pose;
    end_pose.pose.position.z += .01;
    CartesianPath full;
    auto mixed = end_pose;
    mixed.header.frame_id = "unknown_frame";
    expect(
        move.compute_cartesian_path(left, {start_pose, mixed}, .001, full),
        ErrorCode::InvalidTarget);
    check(move.compute_cartesian_path(left, {end_pose}, .001, full));
    require(
        full.fraction >= 1 - 1e-6 && full.plan.groups() == std::vector<RobotGroup>{left},
        "full Cartesian/fixed spine failed");
    // A deliberately tiny absolute jump bound must reject/truncate the path.
    node->set_parameter(
        rclcpp::Parameter("move_group.cartesian.absolute_revolute_threshold", 1e-7));
    {
        MoveGroup limited(node);
        check(limited.initialize(30s));
        check(limited.add_group(left));
        CartesianPath rejected;
        const auto jump_result = limited.compute_cartesian_path(left, {end_pose}, .002, rejected);
        require(!jump_result || rejected.fraction < 1 - 1e-6, "absolute joint jump check disabled");
    }
    node->set_parameter(rclcpp::Parameter("move_group.cartesian.absolute_revolute_threshold", .3));
    check(move.execute(full.plan));
    check(move.clear_path_constraints());
    before = measured(node, model);
    start_pose = pose(before, "left_fr3v2_1_hand_tcp");
    end_pose = start_pose;
    end_pose.pose.position.z += .005;
    auto far = end_pose;
    far.pose.position.z += 3;
    CartesianPath partial;
    check(move.compute_cartesian_path(left, {end_pose, far}, .002, partial));
    require(
        partial.fraction < 1 - 1e-6 && partial.plan.valid(),
        "partial Cartesian diagnostic missing");
    partial.fraction = 1;
    expect(move.execute(partial.plan), ErrorCode::IncompleteCartesianPath);
    obstacle(node, start_pose, true);
    expect(move.compute_cartesian_path(left, {end_pose}, .002, full), ErrorCode::InvalidStartState);
    obstacle(node, start_pose, false);
    check(move.remove_group(spine));
    before = measured(node, model);
    before.copyJointGroupPositions("left_arm", arm);
    arm[0] += .4;
    check(move.add_joint_position_target(left, arm));
    check(move.plan(plan));
    const double origin = before.getVariablePosition("left_fr3v2_1_joint1");
    check_fresh_attachment(node, move, plan, before);
    auto running = std::async(std::launch::async, [&] { return move.execute(plan); });
    const auto deadline = std::chrono::steady_clock::now() + 8s;
    bool moving = false;
    while (std::chrono::steady_clock::now() < deadline) {
        if (std::abs(measured(node, model).getVariablePosition("left_fr3v2_1_joint1") - origin) >
            .005) {
            moving = true;
            break;
        }
        require(
            running.wait_for(20ms) != std::future_status::ready,
            "execution ended before cancel test");
    }
    require(moving, "real execution did not start");
    expect(move.clear_targets(), ErrorCode::Busy);
    expect(move.set_max_velocity_scaling_factor(.2), ErrorCode::Busy);
    const auto stopped = move.stop();
    require(
        stopped.code == ErrorCode::Canceled || stopped.code == ErrorCode::Success,
        "stop lacked terminal result: " + stopped.message);
    require(
        running.wait_for(5s) == std::future_status::ready, "synchronous execute failed to settle");
    expect(running.get(), ErrorCode::Canceled);
    check(move.stop());
    check(move.clear_targets());
    check(move.clear_groups());
    for (int i = 0; i < 30; ++i) {
        MoveGroup temporary(node);
    }
    std::cout << "MOVEIT_FACADE_PASS AND/exact groups, immutable Plan/groups/start, full/partial "
                 "Cartesian, collision/path/jump/time, actual execution/cancel/busy/destruction"
              << std::endl;
}
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // Deliberately no copied robot parameters: initialize retrieves the running SDK's config.
    auto node = std::make_shared<rclcpp::Node>("moveit_facade_probe");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    int status = 0;
    try {
        run(node);
    } catch (const std::exception& e) {
        std::cerr << "MOVEIT_FACADE_FAIL " << e.what() << std::endl;
        status = 1;
    }
    executor.cancel();
    spinner.join();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return status;
}
