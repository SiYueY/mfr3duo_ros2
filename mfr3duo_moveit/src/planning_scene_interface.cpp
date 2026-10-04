#include "mfr3duo_moveit/planning_scene_interface.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

namespace mfr3duo_moveit {
namespace {
using Clock = std::chrono::steady_clock;
using Get = moveit_msgs::srv::GetPlanningScene;
using Apply = moveit_msgs::srv::ApplyPlanningScene;
Result error(ErrorCode code, const char* text) { return {code, text}; }
bool valid_pose(const geometry_msgs::msg::Pose& p) {
    const auto& q = p.orientation;
    return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
           std::isfinite(p.position.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) &&
           std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) < 1e-5;
}
Eigen::Isometry3d pose_transform(const geometry_msgs::msg::Pose& pose) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    result.linear() =
        Eigen::Quaterniond(
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            .toRotationMatrix();
    return result;
}
geometry_msgs::msg::Pose pose_message(const Eigen::Isometry3d& transform) {
    geometry_msgs::msg::Pose result;
    result.position.x = transform.translation().x();
    result.position.y = transform.translation().y();
    result.position.z = transform.translation().z();
    Eigen::Quaterniond q(transform.linear());
    result.orientation.x = q.x();
    result.orientation.y = q.y();
    result.orientation.z = q.z();
    result.orientation.w = q.w();
    return result;
}
bool valid_object(const moveit_msgs::msg::CollisionObject& object) {
    const auto& origin = object.pose;
    const bool legacy_origin = origin.position.x == 0 && origin.position.y == 0 &&
                               origin.position.z == 0 && origin.orientation.x == 0 &&
                               origin.orientation.y == 0 && origin.orientation.z == 0 &&
                               origin.orientation.w == 0;
    if (!legacy_origin && !valid_pose(origin)) return false;
    if (object.id.empty() || object.header.frame_id.empty() || object.operation != object.ADD ||
        object.primitives.empty() || !object.meshes.empty() || !object.planes.empty() ||
        object.primitives.size() != object.primitive_poses.size() || !object.subframe_names.empty())
        return false;
    for (std::size_t i = 0; i < object.primitives.size(); ++i) {
        const auto& primitive = object.primitives[i];
        // V1 scene geometry is explicit solid primitives; unsupported mesh/plane
        // inputs fail rather than silently dropping geometry.
        const std::size_t count =
            primitive.type == primitive.BOX                                              ? 3
            : primitive.type == primitive.SPHERE                                         ? 1
            : (primitive.type == primitive.CYLINDER || primitive.type == primitive.CONE) ? 2
                                                                                         : 0;
        if (count == 0 || primitive.dimensions.size() != count ||
            !valid_pose(object.primitive_poses[i]))
            return false;
        for (double dimension : primitive.dimensions)
            if (!std::isfinite(dimension) || dimension <= 0) return false;
    }
    return true;
}
}  // namespace
struct PlanningSceneInterface::Impl {
    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callbacks;
    rclcpp::Client<Get>::SharedPtr get;
    rclcpp::Client<Apply>::SharedPtr apply;
    std::shared_ptr<robot_model_loader::RobotModelLoader> loader;
    moveit::core::RobotModelPtr model;
    mutable std::mutex mutex;
    bool initialized{false};
    bool uncertain{false};
    struct ApplyReceipt {
        std::atomic<bool> finished{false};
    };
    std::shared_ptr<ApplyReceipt> pending_apply;
    double timeout{2};
    std::map<std::pair<std::string, RobotGroup>, std::map<std::string, bool>> contact_restore;
    explicit Impl(rclcpp::Node::SharedPtr supplied) : node(std::move(supplied)) {
        if (!node) throw std::invalid_argument("PlanningSceneInterface requires external node");
        if (!node->has_parameter("planning_scene.timeout"))
            node->declare_parameter("planning_scene.timeout", 2.);
        timeout = node->get_parameter("planning_scene.timeout").as_double();
        if (!std::isfinite(timeout) || timeout <= 0)
            throw std::invalid_argument("planning_scene.timeout");
        callbacks = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
            [group = callbacks]() mutable { group.reset(); });
        get = node->create_client<Get>(
            "/get_planning_scene", rmw_qos_profile_services_default, callbacks);
        apply = node->create_client<Apply>(
            "/apply_planning_scene", rmw_qos_profile_services_default, callbacks);
    }
    Clock::time_point deadline() const {
        return Clock::now() +
               std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout));
    }
    Result query(moveit_msgs::msg::PlanningScene& output, Clock::time_point until) const {
        if (!get->service_is_ready()) return error(ErrorCode::NotReady, "scene query unavailable");
        auto request = std::make_shared<Get::Request>();
        request->components.components =
            Get::Request().components.SCENE_SETTINGS | request->components.ROBOT_STATE |
            request->components.ROBOT_STATE_ATTACHED_OBJECTS |
            request->components.WORLD_OBJECT_GEOMETRY | request->components.TRANSFORMS |
            request->components.ALLOWED_COLLISION_MATRIX;
        auto future = get->async_send_request(request);
        if (future.wait_until(until) != std::future_status::ready) {
            get->remove_pending_request(future);
            return error(ErrorCode::Timeout, "scene query deadline");
        }
        output = future.get()->scene;
        return {};
    }
    Result commit(const moveit_msgs::msg::PlanningScene& diff, Clock::time_point until) {
        if (!apply->service_is_ready())
            return error(ErrorCode::NotReady, "scene apply unavailable");
        if (Clock::now() >= until)
            return error(ErrorCode::Timeout, "scene preparation deadline; apply not sent");
        auto request = std::make_shared<Apply::Request>();
        request->scene = diff;
        auto receipt = std::make_shared<ApplyReceipt>();
        auto future =
            apply->async_send_request(request, [receipt](rclcpp::Client<Apply>::SharedFuture) {
                receipt->finished.store(true, std::memory_order_release);
            });
        if (future.wait_until(until) != std::future_status::ready) {
            pending_apply = std::move(receipt);
            uncertain = true;
            return error(
                ErrorCode::Timeout,
                "scene apply unconfirmed; initialize must reconcile before further mutations");
        }
        if (!future.get()->success)
            return error(ErrorCode::ExecutionFailed, "scene update rejected");
        return {};
    }
    Result ready() const {
        if (!initialized) return error(ErrorCode::NotInitialized, "initialize scene first");
        if (uncertain)
            return error(ErrorCode::PreviousOperationNotTerminated, "scene update not confirmed");
        return {};
    }
    Result arm_links(RobotGroup arm, std::string& tool, std::vector<std::string>& touch) const {
        if (arm != RobotGroup::LeftArm && arm != RobotGroup::RightArm)
            return error(ErrorCode::InvalidGroup, "attach requires one arm");
        const auto* gripper = model->getJointModelGroup(
            arm == RobotGroup::LeftArm ? "left_gripper" : "right_gripper");
        if (!gripper) return error(ErrorCode::InvalidGroup, "description missing gripper group");
        touch = gripper->getLinkModelNames();
        tool = arm == RobotGroup::LeftArm ? "left_fr3v2_1_hand_tcp" : "right_fr3v2_1_hand_tcp";
        if (!model->getLinkModel(tool) || touch.size() != 2)
            return error(ErrorCode::InvalidGroup, "description grasp links invalid");
        return {};
    }
};
PlanningSceneInterface::PlanningSceneInterface(const rclcpp::Node::SharedPtr& node)
: impl_(std::make_unique<Impl>(node)) {}
PlanningSceneInterface::~PlanningSceneInterface() = default;
Result PlanningSceneInterface::initialize(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    if (timeout.count() <= 0)
        return error(ErrorCode::InvalidTarget, "positive initialize timeout required");
    const auto until = Clock::now() + timeout;
    if (impl_->pending_apply && !impl_->pending_apply->finished.load(std::memory_order_acquire))
        return error(
            ErrorCode::PreviousOperationNotTerminated, "late scene apply still unconfirmed");
    try {
        if (!impl_->get->wait_for_service(
                std::max(Clock::duration::zero(), until - Clock::now())) ||
            !impl_->apply->wait_for_service(
                std::max(Clock::duration::zero(), until - Clock::now())))
            return error(ErrorCode::NotReady, "scene services unavailable");
        if (!impl_->node->has_parameter("robot_description") ||
            !impl_->node->has_parameter("robot_description_semantic")) {
            using Params = rcl_interfaces::srv::GetParameters;
            auto client = impl_->node->create_client<Params>(
                "/move_group/get_parameters", rmw_qos_profile_services_default, impl_->callbacks);
            if (!client->wait_for_service(std::max(Clock::duration::zero(), until - Clock::now())))
                return error(ErrorCode::NotReady, "model parameters unavailable");
            auto request = std::make_shared<Params::Request>();
            request->names = {"robot_description", "robot_description_semantic"};
            auto future = client->async_send_request(request);
            if (future.wait_until(until) != std::future_status::ready) {
                client->remove_pending_request(future);
                return error(ErrorCode::Timeout, "model parameters deadline");
            }
            auto response = future.get();
            if (response->values.size() != 2)
                return error(ErrorCode::NotReady, "model parameters missing");
            for (std::size_t i = 0; i < 2; ++i)
                if (!impl_->node->has_parameter(request->names[i]))
                    impl_->node->declare_parameter(
                        request->names[i], rclcpp::ParameterValue(response->values[i]));
        }
        impl_->loader = std::make_shared<robot_model_loader::RobotModelLoader>(
            impl_->node, "robot_description", false);
        impl_->model = impl_->loader->getModel();
        if (!impl_->model) return error(ErrorCode::NotReady, "description model unavailable");
        moveit_msgs::msg::PlanningScene scene;
        auto result = impl_->query(scene, until);
        if (!result) return result;
        impl_->initialized = true;
        // A late service response confirms the remote apply completed. Query
        // alone cannot rule out a request that may still execute later.
        if (impl_->pending_apply && !impl_->pending_apply->finished.load(std::memory_order_acquire))
            return error(
                ErrorCode::PreviousOperationNotTerminated, "late scene apply still unconfirmed");
        impl_->pending_apply.reset();
        impl_->uncertain = false;
        return {};
    } catch (const std::exception& e) {
        return {ErrorCode::InternalError, e.what()};
    }
}
Result PlanningSceneInterface::add_collision_object(
    const moveit_msgs::msg::CollisionObject& object) {
    return add_collision_objects({object});
}
Result PlanningSceneInterface::add_collision_objects(
    const std::vector<moveit_msgs::msg::CollisionObject>& objects) {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    auto result = impl_->ready();
    if (!result) return result;
    std::set<std::string> ids;
    if (objects.empty()) return error(ErrorCode::InvalidTarget, "empty scene update");
    for (const auto& object : objects)
        if (!valid_object(object) || !ids.insert(object.id).second)
            return error(ErrorCode::InvalidTarget, "invalid or duplicate scene object");
    const auto until = impl_->deadline();
    moveit_msgs::msg::PlanningScene scene;
    result = impl_->query(scene, until);
    if (!result) return result;
    for (const auto& attached : scene.robot_state.attached_collision_objects)
        if (ids.count(attached.object.id))
            return error(ErrorCode::InvalidTarget, "cannot replace held object as world object");
    moveit::core::RobotState snapshot(impl_->model);
    snapshot.setToDefaultValues();
    if (!moveit::core::robotStateMsgToRobotState(scene.robot_state, snapshot))
        return error(ErrorCode::InvalidStartState, "queried scene robot state invalid");
    snapshot.update();
    moveit::core::Transforms fixed(impl_->model->getModelFrame());
    fixed.setTransforms(scene.fixed_frame_transforms);
    auto expected = objects;
    for (auto& object : expected) {
        const bool robot_frame = snapshot.knowsFrameTransform(object.header.frame_id);
        if (!robot_frame && !fixed.canTransform(object.header.frame_id))
            return error(ErrorCode::InvalidTarget, "unknown collision object frame");
        const auto frame = robot_frame ? snapshot.getFrameTransform(object.header.frame_id)
                                       : fixed.getTransform(object.header.frame_id);
        for (auto& pose : object.primitive_poses) pose = pose_message(frame * pose_transform(pose));
        object.header.frame_id = impl_->model->getModelFrame();
    }
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    diff.world.collision_objects = objects;
    result = impl_->commit(diff, until);
    if (!result) return result;
    result = impl_->query(scene, until);
    if (!result) {
        impl_->uncertain = true;
        return result;
    }
    for (const auto& id : ids)
        if (std::none_of(
                scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
                [&](const auto& object) { return object.id == id; }))
            return error(ErrorCode::ExecutionFailed, "scene object absent after apply");
    for (const auto& object : expected) {
        const auto actual = std::find_if(
            scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
            [&](const auto& value) { return value.id == object.id; });
        if (actual == scene.world.collision_objects.end() ||
            actual->header.frame_id != object.header.frame_id ||
            actual->primitives != object.primitives ||
            actual->primitive_poses.size() != object.primitive_poses.size())
            return error(ErrorCode::ExecutionFailed, "scene geometry not confirmed");
        for (std::size_t i = 0; i < object.primitive_poses.size(); ++i) {
            if (!valid_pose(actual->primitive_poses[i]))
                return error(ErrorCode::ExecutionFailed, "invalid queried scene pose");
            const auto expected_pose = pose_transform(object.primitive_poses[i]);
            if (!valid_pose(actual->pose))
                return error(ErrorCode::ExecutionFailed, "invalid queried object origin");
            const auto actual_pose =
                pose_transform(actual->pose) * pose_transform(actual->primitive_poses[i]);
            if ((expected_pose.translation() - actual_pose.translation()).norm() > 1e-4 ||
                Eigen::AngleAxisd(expected_pose.linear() * actual_pose.linear().transpose())
                        .angle() > 1e-4)
                return error(ErrorCode::ExecutionFailed, "scene pose not confirmed");
        }
    }
    return {};
}
Result PlanningSceneInterface::remove_collision_object(const std::string& id) {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    auto result = impl_->ready();
    if (!result) return result;
    if (id.empty()) return error(ErrorCode::InvalidTarget, "empty object ID");
    const auto until = impl_->deadline();
    moveit_msgs::msg::PlanningScene scene;
    result = impl_->query(scene, until);
    if (!result) return result;
    for (const auto& object : scene.robot_state.attached_collision_objects)
        if (object.object.id == id)
            return error(ErrorCode::InvalidTarget, "cannot remove attached object");
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    moveit_msgs::msg::CollisionObject remove;
    remove.id = id;
    remove.operation = remove.REMOVE;
    diff.world.collision_objects = {remove};
    result = impl_->commit(diff, until);
    if (!result) return result;
    result = impl_->query(scene, until);
    if (!result) {
        impl_->uncertain = true;
        return result;
    }
    for (const auto& object : scene.world.collision_objects)
        if (object.id == id) return error(ErrorCode::ExecutionFailed, "remove not confirmed");
    return {};
}
Result PlanningSceneInterface::get_object(
    const std::string& id, moveit_msgs::msg::CollisionObject& object) const {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    auto result = impl_->ready();
    if (!result) return result;
    if (id.empty()) return error(ErrorCode::InvalidTarget, "empty object ID");
    moveit_msgs::msg::PlanningScene scene;
    result = impl_->query(scene, impl_->deadline());
    if (!result) return result;
    for (const auto& found : scene.world.collision_objects)
        if (found.id == id) {
            object = found;
            return {};
        }
    return error(ErrorCode::InvalidTarget, "world object absent");
}
Result PlanningSceneInterface::has_object(const std::string& id, bool& present) const {
    moveit_msgs::msg::CollisionObject object;
    auto result = get_object(id, object);
    if (result) {
        present = true;
        return {};
    }
    if (result.code == ErrorCode::InvalidTarget && !id.empty() &&
        result.message == "world object absent") {
        present = false;
        return {};
    }
    return result;
}
Result PlanningSceneInterface::is_attached(const std::string& id, bool& attached) const {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    auto result = impl_->ready();
    if (!result) return result;
    if (id.empty()) return error(ErrorCode::InvalidTarget, "empty object ID");
    moveit_msgs::msg::PlanningScene scene;
    result = impl_->query(scene, impl_->deadline());
    if (!result) return result;
    attached = std::any_of(
        scene.robot_state.attached_collision_objects.begin(),
        scene.robot_state.attached_collision_objects.end(),
        [&](const auto& object) { return object.object.id == id; });
    return {};
}
Result PlanningSceneInterface::attach_object(const std::string& id, RobotGroup arm) {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    auto result = impl_->ready();
    if (!result) return result;
    std::string tool;
    std::vector<std::string> touch;
    result = impl_->arm_links(arm, tool, touch);
    if (!result) return result;
    const auto until = impl_->deadline();
    moveit_msgs::msg::PlanningScene scene;
    result = impl_->query(scene, until);
    if (!result) return result;
    if (std::none_of(
            scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
            [&](const auto& object) { return object.id == id; }))
        return error(ErrorCode::InvalidTarget, "attach requires existing world object");
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = tool;
    attached.touch_links = touch;
    attached.object.id = id;
    attached.object.operation = attached.object.ADD;
    diff.robot_state.attached_collision_objects = {attached};
    result = impl_->commit(diff, until);
    if (!result) return result;
    result = impl_->query(scene, until);
    if (!result) {
        impl_->uncertain = true;
        return result;
    }
    const bool held = std::any_of(
        scene.robot_state.attached_collision_objects.begin(),
        scene.robot_state.attached_collision_objects.end(), [&](const auto& value) {
            return value.object.id == id && value.link_name == tool && value.touch_links == touch;
        });
    const bool world = std::any_of(
        scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
        [&](const auto& value) { return value.id == id; });
    return held && !world ? Result{} : error(ErrorCode::ExecutionFailed, "attach not confirmed");
}
Result PlanningSceneInterface::detach_object(const std::string& id) {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    auto result = impl_->ready();
    if (!result) return result;
    if (id.empty()) return error(ErrorCode::InvalidTarget, "empty object ID");
    const auto until = impl_->deadline();
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    moveit_msgs::msg::AttachedCollisionObject object;
    object.object.id = id;
    object.object.operation = object.object.REMOVE;
    diff.robot_state.attached_collision_objects = {object};
    result = impl_->commit(diff, until);
    if (!result) return result;
    moveit_msgs::msg::PlanningScene scene;
    result = impl_->query(scene, until);
    if (!result) {
        impl_->uncertain = true;
        return result;
    }
    for (const auto& value : scene.robot_state.attached_collision_objects)
        if (value.object.id == id) return error(ErrorCode::ExecutionFailed, "detach not confirmed");
    // MoveIt includes attached touch links in its effective ACM. Restore the
    // saved world-object rules after removal of that attached representation.
    collision_detection::AllowedCollisionMatrix matrix(scene.allowed_collision_matrix);
    bool restore = false;
    for (const auto& saved : impl_->contact_restore)
        if (saved.first.first == id) {
            for (const auto& pair : saved.second) matrix.setEntry(id, pair.first, pair.second);
            restore = true;
        }
    if (restore) {
        moveit_msgs::msg::PlanningScene restored;
        restored.is_diff = true;
        restored.robot_state.is_diff = true;
        matrix.getMessage(restored.allowed_collision_matrix);
        result = impl_->commit(restored, until);
        if (!result) return result;
        result = impl_->query(scene, until);
        if (!result) {
            impl_->uncertain = true;
            return result;
        }
        collision_detection::AllowedCollisionMatrix confirmed(scene.allowed_collision_matrix);
        for (const auto& saved : impl_->contact_restore)
            if (saved.first.first == id)
                for (const auto& pair : saved.second) {
                    collision_detection::AllowedCollision::Type type =
                        collision_detection::AllowedCollision::NEVER;
                    confirmed.getAllowedCollision(id, pair.first, type);
                    if ((type == collision_detection::AllowedCollision::ALWAYS) != pair.second)
                        return error(
                            ErrorCode::ExecutionFailed, "world contact restoration not confirmed");
                }
        for (auto it = impl_->contact_restore.begin(); it != impl_->contact_restore.end();)
            if (it->first.first == id)
                it = impl_->contact_restore.erase(it);
            else
                ++it;
    }
    return {};
}
Result PlanningSceneInterface::set_grasp_contact_allowed(
    const std::string& id, RobotGroup arm, bool allowed) {
    std::unique_lock<std::mutex> lock(impl_->mutex, std::try_to_lock);
    if (!lock.owns_lock()) return error(ErrorCode::Busy, "scene busy");
    auto result = impl_->ready();
    if (!result) return result;
    if (id.empty()) return error(ErrorCode::InvalidTarget, "empty object ID");
    std::string tool;
    std::vector<std::string> links;
    result = impl_->arm_links(arm, tool, links);
    if (!result) return result;
    const auto key = std::make_pair(id, arm);
    if (!allowed && !impl_->contact_restore.count(key)) return {};
    const auto until = impl_->deadline();
    moveit_msgs::msg::PlanningScene scene;
    result = impl_->query(scene, until);
    if (!result) return result;
    if (allowed && std::none_of(
                       scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
                       [&](const auto& object) { return object.id == id; }))
        return error(ErrorCode::InvalidTarget, "contact requires world object");
    const bool attached = std::any_of(
        scene.robot_state.attached_collision_objects.begin(),
        scene.robot_state.attached_collision_objects.end(), [&](const auto& object) {
            return object.object.id == id && object.link_name == tool &&
                   std::all_of(links.begin(), links.end(), [&](const auto& link) {
                       return std::find(
                                  object.touch_links.begin(), object.touch_links.end(), link) !=
                              object.touch_links.end();
                   });
        });
    collision_detection::AllowedCollisionMatrix matrix(scene.allowed_collision_matrix);
    auto previous = impl_->contact_restore.count(key) ? impl_->contact_restore.at(key)
                                                      : std::map<std::string, bool>{};
    for (const auto& link : links) {
        collision_detection::AllowedCollision::Type type =
            collision_detection::AllowedCollision::NEVER;
        matrix.getAllowedCollision(id, link, type);
        if (type == collision_detection::AllowedCollision::CONDITIONAL)
            return error(
                ErrorCode::InvalidConstraint, "conditional grasp contact rule unsupported");
        if (!previous.count(link))
            previous[link] = type == collision_detection::AllowedCollision::ALWAYS;
        matrix.setEntry(id, link, allowed || attached ? true : previous.at(link));
    }
    moveit_msgs::msg::PlanningScene diff;
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    matrix.getMessage(diff.allowed_collision_matrix);
    result = impl_->commit(diff, until);
    if (!result) return result;
    result = impl_->query(scene, until);
    if (!result) {
        impl_->uncertain = true;
        return result;
    }
    collision_detection::AllowedCollisionMatrix confirmed(scene.allowed_collision_matrix);
    for (const auto& link : links) {
        collision_detection::AllowedCollision::Type type =
            collision_detection::AllowedCollision::NEVER;
        if (!confirmed.getAllowedCollision(id, link, type) ||
            (type == collision_detection::AllowedCollision::ALWAYS) !=
                (allowed || attached ? true : previous.at(link)))
            return error(ErrorCode::ExecutionFailed, "contact rule not confirmed");
    }
    if (allowed || attached)
        impl_->contact_restore[key] = std::move(previous);
    else
        impl_->contact_restore.erase(key);
    return {};
}
}  // namespace mfr3duo_moveit
