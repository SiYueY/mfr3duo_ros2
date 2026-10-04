#include "mfr3duo_moveit/move_group.hpp"
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <iostream>

using namespace mfr3duo_moveit;
static_assert(
    std::is_same_v<
        decltype(std::declval<Plan>().trajectory()), const moveit_msgs::msg::RobotTrajectory&>);
static_assert(std::is_same_v<
              decltype(std::declval<Plan>().start_state()), const moveit_msgs::msg::RobotState&>);
static_assert(!std::is_copy_constructible_v<MoveGroup> && !std::is_move_constructible_v<MoveGroup>);
void expect(Result r, ErrorCode code) {
    if (r.code != code)
        throw std::runtime_error(
            "unexpected result " + std::to_string(static_cast<int>(r.code)) + " " + r.message);
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_builder_test");
    int result = 0;
    try {
        MoveGroup move(node);
        Plan plan;
        expect(move.plan(plan), ErrorCode::NotInitialized);
        if (plan.valid() || move.is_ready())
            throw std::runtime_error("uninitialized state reported ready");
        expect(move.initialize(std::chrono::milliseconds(0)), ErrorCode::Timeout);
        const auto left = RobotGroup::LeftArm, right = RobotGroup::RightArm;
        expect(move.add_groups({left, left}), ErrorCode::DuplicateGroup);
        if (!move.get_groups().empty()) throw std::runtime_error("duplicate add mutated groups");
        expect(move.add_groups({left, static_cast<RobotGroup>(3)}), ErrorCode::InvalidGroup);
        if (!move.get_groups().empty()) throw std::runtime_error("invalid add mutated groups");
        expect(move.add_group(left), ErrorCode::Success);
        expect(move.add_groups({right, left}), ErrorCode::DuplicateGroup);
        if (move.has_group(right))
            throw std::runtime_error("existing duplicate add mutated groups");
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.orientation.w = 1;
        expect(move.add_pose_target(right, pose), ErrorCode::GroupNotAdded);
        expect(move.add_pose_target(left, pose), ErrorCode::Success);
        expect(move.add_pose_target(left, pose), ErrorCode::TargetAlreadyExists);
        expect(move.remove_group(left), ErrorCode::GroupHasTarget);
        expect(move.clear_groups(), ErrorCode::GroupHasTarget);
        expect(move.remove_target(left), ErrorCode::Success);
        pose.pose.orientation.w = 0;
        expect(move.add_pose_target(left, pose), ErrorCode::InvalidTarget);
        pose.pose.orientation.w = 1;
        pose.header.frame_id.clear();
        expect(move.add_pose_target(left, pose), ErrorCode::InvalidTarget);
        pose.header.frame_id = "base_link";
        pose.pose.position.x = std::numeric_limits<double>::infinity();
        expect(move.add_pose_target(left, pose), ErrorCode::InvalidTarget);
        expect(move.add_joint_position_target(left, .1), ErrorCode::UnsupportedTarget);
        expect(move.set_planning_time(0), ErrorCode::InvalidConstraint);
        expect(
            move.set_planning_time(std::numeric_limits<double>::quiet_NaN()),
            ErrorCode::InvalidConstraint);
        expect(move.set_num_planning_attempts(0), ErrorCode::InvalidConstraint);
        expect(move.set_max_velocity_scaling_factor(1.01), ErrorCode::InvalidConstraint);
        expect(move.set_max_acceleration_scaling_factor(-1), ErrorCode::InvalidConstraint);
        expect(
            move.set_goal_position_tolerance(RobotGroup::Spine, .1), ErrorCode::UnsupportedTarget);
        expect(
            move.set_goal_joint_tolerance(static_cast<RobotGroup>(255), .1),
            ErrorCode::InvalidGroup);
        expect(move.clear_groups(), ErrorCode::Success);
        expect(move.stop(), ErrorCode::Success);
        CartesianPath path;
        expect(
            move.compute_cartesian_path(RobotGroup::Spine, {}, .01, path),
            ErrorCode::UnsupportedCombination);
        expect(move.compute_cartesian_path(left, {}, .01, path), ErrorCode::InvalidTarget);
        // Repeated facade destruction keeps ROS objects bounded and no threads.
        for (int i = 0; i < 30; ++i) {
            MoveGroup temporary(node);
        }
        std::cout << "MOVEIT_BUILDER_PASS" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        result = 1;
    }
    node.reset();
    rclcpp::shutdown();
    return result;
}
