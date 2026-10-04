#pragma once
#include "mfr3duo_robot/task.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <utility>
namespace mfr3duo_robot {
class NavigateTask final : public RobotTask {
public:
    explicit NavigateTask(geometry_msgs::msg::PoseStamped target) : target_(std::move(target)) {}
    void set_target_pose(geometry_msgs::msg::PoseStamped target) { target_ = std::move(target); }
    const geometry_msgs::msg::PoseStamped& target_pose() const noexcept { return target_; }
    std::string_view name() const noexcept override { return "NavigateTask"; }
    std::unique_ptr<RobotTask> clone() const override;

private:
    geometry_msgs::msg::PoseStamped target_;
};
}  // namespace mfr3duo_robot
