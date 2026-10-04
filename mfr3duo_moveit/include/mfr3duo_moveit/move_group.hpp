#pragma once

#include <chrono>
#include <initializer_list>
#include <memory>
#include <string_view>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mfr3duo_moveit/moveit_types.hpp"

namespace mfr3duo_moveit {
// The application must spin node concurrently. This facade owns no executor/thread.
// Use one shared Node and exclusively/serially own the MoveGroup execution channel.
// stop() uses the official manager-wide stop event; it is not client-isolated.
class MoveGroup {
public:
    explicit MoveGroup(const rclcpp::Node::SharedPtr& node);
    ~MoveGroup();
    MoveGroup(const MoveGroup&) = delete;
    MoveGroup& operator=(const MoveGroup&) = delete;
    MoveGroup(MoveGroup&&) = delete;
    MoveGroup& operator=(MoveGroup&&) = delete;
    Result initialize(std::chrono::milliseconds timeout);
    bool is_ready() const;
    Result add_group(RobotGroup group);
    Result add_groups(std::initializer_list<RobotGroup> groups);
    Result remove_group(RobotGroup group);
    Result clear_groups();
    bool has_group(RobotGroup group) const;
    std::vector<RobotGroup> get_groups() const;
    Result add_pose_target(RobotGroup group, const geometry_msgs::msg::PoseStamped& target);
    Result add_position_target(RobotGroup group, const geometry_msgs::msg::PointStamped& target);
    Result add_orientation_target(
        RobotGroup group, const geometry_msgs::msg::QuaternionStamped& target);
    Result add_joint_position_target(RobotGroup group, double position);
    Result add_joint_position_target(RobotGroup group, const std::vector<double>& positions);
    Result add_named_target(RobotGroup group, std::string_view name);
    Result remove_target(RobotGroup group);
    Result clear_targets();
    bool has_target(RobotGroup group) const;
    Result set_start_state(const moveit_msgs::msg::RobotState& state);
    Result set_start_state_to_current_state();
    Result set_goal_joint_tolerance(RobotGroup group, double tolerance);
    Result set_goal_position_tolerance(RobotGroup group, double tolerance);
    Result set_goal_orientation_tolerance(RobotGroup group, double tolerance);
    Result add_path_constraint(const moveit_msgs::msg::Constraints& constraint);
    Result clear_path_constraints();
    Result set_planning_pipeline_id(std::string_view id);
    Result set_planner_id(std::string_view id);
    Result set_planning_time(double seconds);
    Result set_num_planning_attempts(std::size_t attempts);
    Result set_max_velocity_scaling_factor(double factor);
    Result set_max_acceleration_scaling_factor(double factor);
    Result plan(Plan& plan);
    Result execute(const Plan& plan);
    Result move();
    Result stop();
    Result compute_cartesian_path(
        RobotGroup group, const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
        double eef_step, CartesianPath& path);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace mfr3duo_moveit
