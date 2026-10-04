#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include "mfr3duo_moveit/planning_scene_interface.hpp"
using namespace std::chrono_literals;
using namespace mfr3duo_moveit;
namespace {
void require(bool valid, const char* text) {
    if (!valid) throw std::runtime_error(text);
}
void require(const Result& result, const char* text) {
    if (!result) throw std::runtime_error(std::string(text) + ": " + result.message);
}
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("scene_runtime_probe");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    std::thread spin([&] { executor.spin(); });
    int exit_code = 0;
    try {
        PlanningSceneInterface scene(node);
        bool value = true;
        require(
            scene.has_object("probe", value).code == ErrorCode::NotInitialized,
            "uninitialized query");
        require(scene.initialize(45s), "scene initialize");
        moveit_msgs::msg::CollisionObject object;
        object.id = "scene_probe";
        object.header.frame_id = "base_link";
        shape_msgs::msg::SolidPrimitive box;
        box.type = box.BOX;
        box.dimensions = {.04, .04, .05};
        geometry_msgs::msg::Pose pose;
        pose.position.x = 2;
        pose.position.z = 1;
        pose.orientation.w = 1;
        object.primitives = {box};
        object.primitive_poses = {pose};
        object.operation = object.ADD;
        const auto added = scene.add_collision_object(object);
        if (!added) {
            moveit_msgs::msg::CollisionObject queried;
            if (scene.get_object(object.id, queried)) {
                std::cerr << "OBJECT_QUERY origin=" << queried.pose.position.x << ','
                          << queried.pose.position.y << ',' << queried.pose.position.z
                          << " w=" << queried.pose.orientation.w;
                for (const auto& pose : queried.primitive_poses)
                    std::cerr << " primitive=" << pose.position.x << ',' << pose.position.y << ','
                              << pose.position.z << " w=" << pose.orientation.w;
                std::cerr << std::endl;
            }
        }
        require(added, "apply world object");
        require(scene.has_object(object.id, value) && value, "confirmed world query");
        auto invalid = object;
        invalid.id = "invalid_probe";
        invalid.primitives[0].dimensions[0] = -1;
        require(
            scene.add_collision_objects({object, invalid}).code == ErrorCode::InvalidTarget,
            "atomic invalid input");
        require(scene.has_object(invalid.id, value) && !value, "invalid update did not apply");
        require(
            scene.set_grasp_contact_allowed(object.id, RobotGroup::LeftArm, true),
            "temporary grasp contacts");
        auto query = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        request->components.components = request->components.ALLOWED_COLLISION_MATRIX;
        auto future = query->async_send_request(request);
        require(future.wait_for(2s) == std::future_status::ready, "ACM query");
        collision_detection::AllowedCollisionMatrix matrix(
            future.get()->scene.allowed_collision_matrix);
        collision_detection::AllowedCollision::Type type;
        require(
            matrix.getAllowedCollision(object.id, "left_fr3v2_1_leftfinger", type) &&
                type == collision_detection::AllowedCollision::ALWAYS,
            "selected finger contact enabled");
        require(
            !matrix.getAllowedCollision(object.id, "right_fr3v2_1_leftfinger", type) ||
                type == collision_detection::AllowedCollision::NEVER,
            "other hand not allowed");
        require(
            scene.attach_object(object.id, RobotGroup::Spine).code == ErrorCode::InvalidGroup,
            "non-arm attach rejected");
        require(scene.attach_object(object.id, RobotGroup::LeftArm), "attach representation");
        require(scene.is_attached(object.id, value) && value, "confirmed attached query");
        require(scene.has_object(object.id, value) && !value, "attached removed from world");
        require(
            scene.remove_collision_object(object.id).code == ErrorCode::InvalidTarget,
            "held remove rejected");
        require(
            scene.add_collision_object(object).code == ErrorCode::InvalidTarget,
            "held world replacement rejected");
        require(
            scene.set_grasp_contact_allowed(object.id, RobotGroup::LeftArm, false),
            "restore temporary contacts");
        require(scene.detach_object(object.id), "detach representation");
        require(scene.is_attached(object.id, value) && !value, "confirmed detach query");
        require(scene.add_collision_object(object), "replace released world pose");
        require(scene.remove_collision_object(object.id), "confirmed removal");
        require(scene.has_object(object.id, value) && !value, "removed object absent");
        std::cout << "SCENE_RUNTIME_PASS Level A apply/query, validation, finger-only contacts, "
                     "attach/detach; no physical grasp claimed\n";
    } catch (const std::exception& e) {
        std::cerr << "SCENE_RUNTIME_FAIL " << e.what() << '\n';
        exit_code = 1;
    }
    executor.cancel();
    spin.join();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return exit_code;
}
