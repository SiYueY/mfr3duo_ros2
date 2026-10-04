#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mfr3duo_moveit/moveit_types.hpp"
namespace mfr3duo_moveit {
/** Confirmed scene updates on the application's externally spun node. */
class PlanningSceneInterface {
public:
    explicit PlanningSceneInterface(const rclcpp::Node::SharedPtr& node);
    ~PlanningSceneInterface();
    PlanningSceneInterface(const PlanningSceneInterface&) = delete;
    PlanningSceneInterface& operator=(const PlanningSceneInterface&) = delete;
    PlanningSceneInterface(PlanningSceneInterface&&) = delete;
    PlanningSceneInterface& operator=(PlanningSceneInterface&&) = delete;
    Result initialize(std::chrono::milliseconds timeout);
    Result add_collision_object(const moveit_msgs::msg::CollisionObject& object);
    Result add_collision_objects(const std::vector<moveit_msgs::msg::CollisionObject>& objects);
    Result remove_collision_object(const std::string& id);
    Result has_object(const std::string& id, bool& present) const;
    Result get_object(const std::string& id, moveit_msgs::msg::CollisionObject& object) const;
    Result is_attached(const std::string& id, bool& attached) const;
    Result attach_object(const std::string& id, RobotGroup arm);
    Result detach_object(const std::string& id);
    Result set_grasp_contact_allowed(const std::string& id, RobotGroup arm, bool allowed);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace mfr3duo_moveit
