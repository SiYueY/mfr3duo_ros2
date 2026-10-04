#pragma once
#include <Eigen/Geometry>
#include <moveit_msgs/msg/collision_object.hpp>
#include <mfr3duo_moveit/planning_scene_interface.hpp>
#include "grasp_observer.hpp"
#include <mfr3duo_control/control.hpp>
namespace mfr3duo_robot {
enum class PhysicalHolding { Held, Released, Unknown };
struct GraspRecoveryResult {
    PhysicalHolding holding{PhysicalHolding::Unknown};
    bool scene_confirmed{false};
    std::string diagnostic;
};
// Called only after the caller confirms its owned motion actually terminated.
// Unknown observation/state never changes PlanningScene or opens the gripper.
GraspRecoveryResult recover_grasp(
    const rclcpp::Node::SharedPtr& node, mfr3duo_moveit::PlanningSceneInterface& scene,
    GraspObserver& observer, const std::string& object_id, Manipulator manipulator,
    const Eigen::Isometry3d& held_relative, const moveit_msgs::msg::CollisionObject& geometry,
    bool motion_confirmed, mfr3duo_control::Control& control, bool grasp_verified,
    double expected_width = 0.04, double inner = 0.005, double outer = 0.005);
}  // namespace mfr3duo_robot
