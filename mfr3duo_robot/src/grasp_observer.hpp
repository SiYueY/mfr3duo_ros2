#pragma once
#include <memory>
#include <string>
#include <string_view>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mfr3duo_robot/types.hpp"
namespace mfr3duo_robot {
struct GraspObservation {
    bool valid{false};
    bool object_visible{false};
    geometry_msgs::msg::PoseStamped object_pose;
    geometry_msgs::msg::PoseStamped tool_pose;
    std::string diagnostic;
};
class GraspObserver {
public:
    virtual ~GraspObserver() = default;
    virtual GraspObservation observe(std::string_view object_id, Manipulator manipulator) = 0;
};
class SimulationGraspObserver final : public GraspObserver {
public:
    explicit SimulationGraspObserver(const rclcpp::Node::SharedPtr& node);
    ~SimulationGraspObserver() override;
    SimulationGraspObserver(const SimulationGraspObserver&) = delete;
    SimulationGraspObserver& operator=(const SimulationGraspObserver&) = delete;
    GraspObservation observe(std::string_view object_id, Manipulator manipulator) override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace mfr3duo_robot
