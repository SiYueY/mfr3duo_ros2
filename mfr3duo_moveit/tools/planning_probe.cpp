// Phase 2A POC only. This executable is not the public MoveGroup facade.
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <chrono>
#include <deque>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>

namespace {
void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
void collision_free(planning_scene::PlanningScene& scene, const moveit::core::RobotState& state) {
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 100;
    collision_detection::CollisionResult result;
    scene.checkCollision(request, result, state);
    for (const auto& contact : result.contacts)
        std::cerr << "COLLISION " << contact.first.first << " " << contact.first.second << '\n';
    require(!result.collision, "full RobotState collision");
}
void model_contract(const moveit::core::RobotModelPtr& model) {
    const std::map<std::string, unsigned> groups{
        {"left_arm", 7},        {"right_arm", 7}, {"spine", 1},          {"left_arm_spine", 8},
        {"right_arm_spine", 8}, {"dual_arm", 14}, {"dual_arm_spine", 15}};
    for (const auto& item : groups) {
        const auto* group = model->getJointModelGroup(item.first);
        require(
            group && group->getActiveVariableCount() == item.second,
            "group dimensions " + item.first);
        for (const auto& name : group->getActiveJointModelNames())
            require(
                name.find("finger") == std::string::npos && name.find("tmrv") == std::string::npos,
                "nonplanning joint in group " + item.first);
        if (item.first.find("arm") != std::string::npos &&
            item.first.find("dual") == std::string::npos)
            require(group->getSolverInstance() != nullptr, "KDL solver missing " + item.first);
    }
    require(
        model->getEndEffector("left_hand") && model->getEndEffector("right_hand"), "end effectors");
    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState home(model);
    home.setToDefaultValues();
    require(
        home.setToDefaultValues(model->getJointModelGroup("dual_arm_spine"), "home"),
        "home absent");
    home.update();
    require(home.satisfiesBounds(), "home bounds");
    collision_free(scene, home);
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.id = "collision_contract_obstacle";
    obstacle.header.frame_id = model->getModelFrame();
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {.2, .2, .2};
    obstacle.primitives.push_back(box);
    geometry_msgs::msg::Pose pose;
    const auto tool = home.getGlobalLinkTransform("left_fr3v2_1_hand_tcp").translation();
    pose.position.x = tool.x();
    pose.position.y = tool.y();
    pose.position.z = tool.z();
    pose.orientation.w = 1;
    obstacle.primitive_poses.push_back(pose);
    obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
    require(scene.processCollisionObjectMsg(obstacle), "add collision fixture");
    require(scene.isStateColliding(home), "environment collision must reject blocked TCP");
    obstacle.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    require(scene.processCollisionObjectMsg(obstacle), "remove collision fixture");
    collision_free(scene, home);
    std::cout << "MODEL_CONTRACT_PASS" << std::endl;
}
rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers_ready(
    const rclcpp::Node::SharedPtr& node) {
    using Service = controller_manager_msgs::srv::ListControllers;
    auto client = node->create_client<Service>("/controller_manager/list_controllers");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(40);
    const std::set<std::string> expected{"joint_state_broadcaster", "imu_broadcaster",
                                         "left_arm_controller",     "right_arm_controller",
                                         "spine_controller",        "tmr_controller",
                                         "left_gripper_controller", "right_gripper_controller"};
    while (std::chrono::steady_clock::now() < deadline) {
        if (!client->wait_for_service(std::chrono::milliseconds(200))) continue;
        auto response = client->async_send_request(std::make_shared<Service::Request>());
        if (response.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            client->remove_pending_request(response);
            continue;
        }
        std::set<std::string> active;
        const auto message = response.get();
        for (const auto& controller : message->controller)
            if (controller.state == "active") active.insert(controller.name);
        if (std::includes(active.begin(), active.end(), expected.begin(), expected.end()))
            return client;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    throw std::runtime_error("eight controllers not ACTIVE before deadline");
}
void single_groups(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelPtr& model) {
    planning_scene::PlanningScene scene(model);
    struct StateHistory {
        std::mutex mutex;
        std::deque<sensor_msgs::msg::JointState> samples;
    };
    auto history = std::make_shared<StateHistory>();
    auto joint_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 100, [history](const sensor_msgs::msg::JointState& sample) {
            if (std::find(sample.name.begin(), sample.name.end(), "franka_spine_vertical_joint") ==
                sample.name.end())
                return;
            std::lock_guard<std::mutex> lock(history->mutex);
            history->samples.push_back(sample);
            while (history->samples.size() > 300) history->samples.pop_front();
        });
    auto transforms = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    transforms->setUsingDedicatedThread(true);  // Application executor supplies the TF callbacks.
    tf2_ros::TransformListener listener(*transforms, node, false);
    planning_scene_monitor::CurrentStateMonitor monitor(node, model, transforms, false);
    monitor.startStateMonitor();
    require(monitor.waitForCompleteState(10.0), "complete robot state missing");
    for (const std::string name : {"left_arm", "right_arm", "spine"}) {
        moveit::planning_interface::MoveGroupInterface group(
            node, name, transforms, rclcpp::Duration::from_seconds(10));
        group.setPlanningTime(5.0);
        group.setNumPlanningAttempts(3);
        group.setMaxVelocityScalingFactor(.1);
        group.setMaxAccelerationScalingFactor(.1);
        group.setGoalJointTolerance(.001);
        require(monitor.waitForCurrentState(node->now(), 3.0), "fresh complete state " + name);
        auto before = monitor.getCurrentState();
        require(before != nullptr, "current state " + name);
        for (const auto& joint :
             {"caster_front_left_steering_joint", "caster_front_left_joint", "rocker_arm_joint",
              "caster_rear_right_steering_joint", "caster_rear_right_joint"})
            require(std::isfinite(before->getVariablePosition(joint)), "passive position missing");
        require(
            monitor.haveCompleteState(rclcpp::Duration::from_seconds(1)),
            "complete robot state stale");
        before->update();
        require(before->satisfiesBounds(), "current bounds " + name);
        collision_free(scene, *before);
        const auto* jmg = model->getJointModelGroup(name);
        std::vector<double> target;
        before->copyJointGroupPositions(jmg, target);
        target[0] += name == "spine" ? .02 : .04;
        require(group.setJointValueTarget(target), "target bounds " + name);
        const auto reference = name == "spine" ? "franka_spine" : model->getModelFrame();
        const auto tip = name == "spine" ? "franka_spine_mounting_point"
                                         : name.substr(0, name.find('_')) + "_fr3v2_1_hand_tcp";
        const auto tf = transforms->lookupTransform(
            reference, tip, tf2::TimePointZero, tf2::durationFromSec(3));
        const auto tf_stamp = rclcpp::Time(tf.header.stamp);
        const auto tf_age = (node->now() - tf_stamp).seconds();
        require(tf_age >= -.1 && tf_age < 1., "TF sample freshness " + name);
        // RobotState and latest TF are separate asynchronous streams. Compare the
        // TF with its originating JointState, retaining the existing tolerances.
        sensor_msgs::msg::JointState matched;
        bool found = false;
        const auto match_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!found && std::chrono::steady_clock::now() < match_deadline) {
            {
                std::lock_guard<std::mutex> lock(history->mutex);
                for (const auto& sample : history->samples) {
                    if (std::abs((rclcpp::Time(sample.header.stamp) - tf_stamp).seconds()) < 1e-6) {
                        matched = sample;
                        found = true;
                        break;
                    }
                }
            }
            if (!found) std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        require(found, "TF originating JointState absent " + name);
        auto sampled = *before;
        require(matched.name.size() == matched.position.size(), "JointState dimensions");
        for (std::size_t i = 0; i < matched.name.size(); ++i) {
            if (model->hasJointModel(matched.name[i]))
                sampled.setVariablePosition(matched.name[i], matched.position[i]);
        }
        sampled.update();
        const Eigen::Isometry3d expected_tf = sampled.getGlobalLinkTransform(reference).inverse() *
                                              sampled.getGlobalLinkTransform(tip);
        const auto& translation = tf.transform.translation;
        const double error_tf = (Eigen::Vector3d(translation.x, translation.y, translation.z) -
                                 expected_tf.translation())
                                    .norm();
        std::cout << "SAME_STAMP_TF_CHECK " << name << " error=" << error_tf << " age=" << tf_age
                  << std::endl;
        require(error_tf < (name == "spine" ? .003 : .01), "same-stamp TF/FK mismatch " + name);
        group.setStartState(*before);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        require(group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS, "planning " + name);
        const auto& trajectory = plan.trajectory_.joint_trajectory;
        const auto& expected = jmg->getActiveJointModelNames();
        require(
            std::set<std::string>(trajectory.joint_names.begin(), trajectory.joint_names.end()) ==
                std::set<std::string>(expected.begin(), expected.end()),
            "exact trajectory joints " + name);
        require(trajectory.points.size() >= 2, "trajectory points " + name);
        double previous = -1;
        auto waypoint = *before;
        for (const auto& point : trajectory.points) {
            const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            require(time > previous, "trajectory monotonic time " + name);
            previous = time;
            require(point.positions.size() == trajectory.joint_names.size(), "point dimensions");
            for (std::size_t i = 0; i < point.positions.size(); ++i) {
                require(std::isfinite(point.positions[i]), "point finite");
                waypoint.setVariablePosition(trajectory.joint_names[i], point.positions[i]);
                if (!point.velocities.empty())
                    require(
                        std::abs(point.velocities[i]) <=
                            model->getVariableBounds(trajectory.joint_names[i]).max_velocity_ +
                                1e-5,
                        "trajectory velocity bounds");
            }
            waypoint.update();
            require(waypoint.satisfiesBounds(), "trajectory joint limits");
            collision_free(scene, waypoint);
        }
        require(
            group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS,
            "TEM/FJT execution " + name);
        require(monitor.waitForCurrentState(node->now(), 3.0), "fresh final state " + name);
        auto after = monitor.getCurrentState();
        require(after != nullptr, "final state " + name);
        double error = 0;
        for (std::size_t i = 0; i < expected.size(); ++i)
            error = std::max(error, std::abs(after->getVariablePosition(expected[i]) - target[i]));
        require(error < (name == "spine" ? .003 : .02), "final position error " + name);
        for (const auto& joint :
             model->getJointModelGroup("dual_arm_spine")->getActiveJointModelNames())
            if (std::find(expected.begin(), expected.end(), joint) == expected.end())
                require(
                    std::abs(
                        after->getVariablePosition(joint) - before->getVariablePosition(joint)) <
                        .003,
                    "inactive joint moved " + joint);
        std::cout << "SINGLE_GROUP_PASS " << name << " planning=" << plan.planning_time_
                  << " duration=" << previous << " error=" << error << std::endl;
    }
}
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "phase2a_planning_probe",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    int result = 0;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr readiness_client;
    try {
        robot_model_loader::RobotModelLoader loader(node);
        require(loader.getModel() != nullptr, "robot model");
        model_contract(loader.getModel());
        readiness_client = controllers_ready(node);
        single_groups(node, loader.getModel());
        std::cout << "PHASE2A_PASS" << std::endl;
    } catch (const std::exception& error) {
        std::cerr << "PHASE2A_FAIL " << error.what() << std::endl;
        result = 1;
    }
    executor.cancel();
    spinner.join();
    executor.remove_node(node);
    readiness_client.reset();
    node.reset();
    rclcpp::shutdown();
    return result;
}
