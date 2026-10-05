#include "mfr3duo_moveit/move_group.hpp"
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "execution_monitor.hpp"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <ompl/util/Console.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <tf2_ros/buffer.h>
#include <algorithm>
#include <array>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <variant>

namespace mfr3duo_moveit {
namespace {
using Clock = std::chrono::steady_clock;
using Action = moveit_msgs::action::ExecuteTrajectory;
using Client = rclcpp_action::Client<Action>;
using SceneService = moveit_msgs::srv::GetPlanningScene;
using Constraints = moveit_msgs::msg::Constraints;
using Pose = geometry_msgs::msg::PoseStamped;
using Point = geometry_msgs::msg::PointStamped;
using Quaternion = geometry_msgs::msg::QuaternionStamped;
using Target = std::variant<Pose, Point, Quaternion, std::vector<double>>;
// MoveIt 2.5.9 installs a process-global raw OMPL logger pointer owned by
// each planner instance. Destroying the last-created instance leaves it dangling.
// Give facade pipelines a shared logger whose lifetime exceeds every pipeline.
class OmplOutput final : public ompl::msg::OutputHandler {
public:
    void log(const std::string& text, ompl::msg::LogLevel level, const char* filename, int line)
        override {
        const auto logger = rclcpp::get_logger("mfr3duo_moveit.ompl");
        if (level == ompl::msg::LOG_ERROR)
            RCLCPP_ERROR(logger, "%s:%d - %s", filename, line, text.c_str());
        else if (level == ompl::msg::LOG_WARN)
            RCLCPP_WARN(logger, "%s:%d - %s", filename, line, text.c_str());
        else if (level != ompl::msg::LOG_NONE)
            RCLCPP_DEBUG(logger, "%s:%d - %s", filename, line, text.c_str());
    }
};
struct RestoreOmplOutput {
    RestoreOmplOutput() { output(); }
    ~RestoreOmplOutput() { ompl::msg::useOutputHandler(&output()); }
    static OmplOutput& output() {
        static OmplOutput handler;
        return handler;
    }
};
constexpr char kSpine[] = "franka_spine_vertical_joint";
constexpr std::array<const char*, 3> kGroups{"left_arm", "right_arm", "spine"};
constexpr std::array<const char*, 2> kTips{"left_fr3v2_1_hand_tcp", "right_fr3v2_1_hand_tcp"};
Result error(ErrorCode code, std::string message) { return {code, std::move(message)}; }
bool valid_group(RobotGroup group) { return static_cast<unsigned>(group) < 3; }
bool positive(double value) { return std::isfinite(value) && value > 0; }
bool valid_header(const std_msgs::msg::Header& header) {
    return !header.frame_id.empty() && header.stamp.sec >= 0 && header.stamp.nanosec < 1000000000U;
}
bool valid_point(const geometry_msgs::msg::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
bool valid_quaternion(const geometry_msgs::msg::Quaternion& q) {
    const double squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(squared) && std::abs(squared - 1) < 1e-6;
}
Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& p) {
    auto result = Eigen::Isometry3d::Identity();
    result.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    result.linear() =
        Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
            .toRotationMatrix();
    return result;
}
Pose pose_message(const Eigen::Isometry3d& t, const std::string& frame) {
    Pose p;
    p.header.frame_id = frame;
    p.pose.position.x = t.translation().x();
    p.pose.position.y = t.translation().y();
    p.pose.position.z = t.translation().z();
    const Eigen::Quaterniond q(t.rotation());
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    return p;
}
std::string resolve(const std::set<RobotGroup>& groups) {
    unsigned mask = 0;
    for (const auto group : groups) mask |= 1u << static_cast<unsigned>(group);
    constexpr std::array<const char*, 8> names{
        "",      "left_arm",       "right_arm",       "dual_arm",
        "spine", "left_arm_spine", "right_arm_spine", "dual_arm_spine"};
    return names[mask];
}
Result mapped(int code, ErrorCode fallback) {
    using E = moveit_msgs::msg::MoveItErrorCodes;
    switch (code) {
        case E::SUCCESS:
            return {};
        case E::TIMED_OUT:
            return error(ErrorCode::PlanningTimeout, "MoveIt TIMED_OUT");
        case E::NO_IK_SOLUTION:
            return error(ErrorCode::NoIKSolution, "solver reported NO_IK_SOLUTION");
        case E::START_STATE_IN_COLLISION:
        case E::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        case E::INVALID_ROBOT_STATE:
            return error(
                ErrorCode::InvalidStartState, "MoveIt start state error " + std::to_string(code));
        case E::INVALID_GOAL_CONSTRAINTS:
        case E::GOAL_VIOLATES_PATH_CONSTRAINTS:
            return error(
                ErrorCode::InvalidConstraint, "MoveIt constraint error " + std::to_string(code));
        case E::INVALID_GROUP_NAME:
            return error(ErrorCode::InvalidGroup, "MoveIt invalid group");
        case E::PREEMPTED:
            return error(ErrorCode::Canceled, "MoveIt preempted");
        default:
            return error(fallback, "MoveIt error " + std::to_string(code));
    }
}
}  // namespace

struct MoveGroup::Impl {
    rclcpp::Node::SharedPtr node;
    rclcpp::CallbackGroup::SharedPtr callbacks;
    Client::SharedPtr execution;
    std::shared_ptr<ExecutionMonitor> monitor;
    rclcpp::Client<SceneService>::SharedPtr scene_service;
    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controllers;
    rclcpp::TimerBase::SharedPtr readiness_timer;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf, static_tf;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states;
    std::shared_ptr<MeasuredState> measured = std::make_shared<MeasuredState>();
    std::unique_ptr<robot_model_loader::RobotModelLoader> loader;
    moveit::core::RobotModelPtr model;
    std::shared_ptr<planning_pipeline::PlanningPipeline> pipeline;
    mutable std::mutex mutex;
    bool initialized{false}, busy{false}, settings_valid{true}, planning_canceled{false};
    std::shared_ptr<Operation> operation;
    std::set<RobotGroup> groups;
    std::map<RobotGroup, Target> targets;
    Constraints constraints;
    std::optional<moveit_msgs::msg::RobotState> start_state;
    rclcpp::Time snapshot_stamp{0, 0, RCL_ROS_TIME};
    std::array<double, 3> joint_tolerance{{.0001, .0001, .0001}},
        position_tolerance{{.003, .003, .003}}, orientation_tolerance{{.01, .01, .01}};
    std::string pipeline_id{"ompl"}, planner_id{"RRTConnectkConfigDefault"};
    double planning_time{5}, velocity{.1}, acceleration{.1};
    unsigned attempts{1};
    double response_timeout, margin, cancel_timeout, terminal_timeout, state_timeout, scene_timeout,
        sampling_timeout;
    double arm_start_tolerance, spine_start_tolerance, inactive_tolerance, relative_jump,
        revolute_jump, prismatic_jump;
    explicit Impl(rclcpp::Node::SharedPtr n) : node(std::move(n)) {
        if (!node) throw std::invalid_argument("MoveGroup requires node");
        const auto parameter = [&](const char* name, double fallback) {
            if (!node->has_parameter(name)) node->declare_parameter(name, fallback);
            const double v = node->get_parameter(name).as_double();
            if (!positive(v)) settings_valid = false;
            return v;
        };
        response_timeout = parameter("execution.goal_response_timeout", 2);
        margin = parameter("execution.timeout_margin", 5);
        cancel_timeout = parameter("execution.cancel_timeout", 1);
        terminal_timeout = parameter("execution.terminal_timeout", 4);
        state_timeout = parameter("move_group.state_timeout", 1);
        scene_timeout = parameter("move_group.scene_timeout", 2);
        sampling_timeout = parameter("move_group.sampling_timeout", 2);
        arm_start_tolerance = parameter("execution.start_tolerance.arm_revolute", .02);
        spine_start_tolerance = parameter("execution.start_tolerance.spine_prismatic", .005);
        inactive_tolerance = parameter("move_group.inactive_joint_validation_tolerance", .003);
        relative_jump = parameter("move_group.cartesian.relative_jump_threshold", 5);
        revolute_jump = parameter("move_group.cartesian.absolute_revolute_threshold", .3);
        prismatic_jump = parameter("move_group.cartesian.absolute_prismatic_threshold", .03);
        callbacks = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
            [group = callbacks]() mutable { group.reset(); });
        buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        rclcpp::SubscriptionOptions subscriptions;
        subscriptions.callback_group = callbacks;
        std::weak_ptr<tf2_ros::Buffer> transforms = buffer;
        const auto receive_tf = [transforms](const tf2_msgs::msg::TFMessage& message, bool fixed) {
            if (const auto buffer = transforms.lock())
                for (const auto& transform : message.transforms) try {
                        buffer->setTransform(transform, "mfr3duo_moveit", fixed);
                    } catch (const tf2::TransformException&) {
                    }
        };
        tf = node->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::QoS(100),
            [receive_tf](const tf2_msgs::msg::TFMessage& message) { receive_tf(message, false); },
            subscriptions);
        static_tf = node->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", rclcpp::QoS(100).transient_local(),
            [receive_tf](const tf2_msgs::msg::TFMessage& message) { receive_tf(message, true); },
            subscriptions);
        std::weak_ptr<MeasuredState> weak_state = measured;
        const auto clock = node->get_clock();
        joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [weak_state, clock](const sensor_msgs::msg::JointState& message) {
                const auto state = weak_state.lock();
                if (!state || message.header.stamp.sec < 0 ||
                    message.header.stamp.nanosec >= 1000000000U ||
                    message.name.size() != message.position.size() ||
                    (!message.velocity.empty() && message.velocity.size() != message.name.size()))
                    return;
                const rclcpp::Time stamp(message.header.stamp);
                const auto age = clock->now() - stamp;
                if (age.seconds() < -.1 || age.seconds() > 1) return;
                std::set<std::string> names;
                for (std::size_t i = 0; i < message.name.size(); ++i)
                    if (!names.insert(message.name[i]).second ||
                        !std::isfinite(message.position[i]) ||
                        (!message.velocity.empty() && !std::isfinite(message.velocity[i])))
                        return;
                std::lock_guard<std::mutex> lock(state->mutex);
                const auto received = Clock::now();
                for (std::size_t i = 0; i < message.name.size(); ++i)
                    if (state->required.count(message.name[i])) {
                        const auto old = state->joints.find(message.name[i]);
                        if (old != state->joints.end() && old->second.stamp > stamp) continue;
                        state->joints[message.name[i]] = {
                            message.position[i], message.velocity.empty() ? 0 : message.velocity[i],
                            stamp, received, !message.velocity.empty()};
                    }
            },
            subscriptions);
        execution = rclcpp_action::create_client<Action>(node, "/execute_trajectory", callbacks);
        monitor = execution_monitor(node);
        scene_service = node->create_client<SceneService>(
            "/get_planning_scene", rmw_qos_profile_services_default, callbacks);
        using Controllers = controller_manager_msgs::srv::ListControllers;
        controllers = node->create_client<Controllers>(
            "/controller_manager/list_controllers", rmw_qos_profile_services_default, callbacks);
        std::weak_ptr<rclcpp::Client<Controllers>> weak_controllers = controllers;
        readiness_timer = node->create_wall_timer(
            std::chrono::milliseconds(200),
            [weak_state, weak_controllers] {
                const auto state = weak_state.lock();
                const auto client = weak_controllers.lock();
                if (!state || !client) return;
                if (!client->service_is_ready()) {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    state->controllers_ready = false;
                    return;
                }
                unsigned generation;
                std::int64_t expired;
                {
                    std::lock_guard<std::mutex> lock(state->mutex);
                    if (state->checking &&
                        Clock::now() - state->checking_since < std::chrono::seconds(1))
                        return;
                    expired = state->checking ? state->pending : -1;
                    state->checking = true;
                    state->checking_since = Clock::now();
                    generation = ++state->generation;
                }
                if (expired >= 0) client->remove_pending_request(expired);
                auto request = client->async_send_request(
                    std::make_shared<Controllers::Request>(),
                    [weak_state, generation](rclcpp::Client<Controllers>::SharedFuture future) {
                        const auto state = weak_state.lock();
                        if (!state) return;
                        const auto response = future.get();
                        bool active = true;
                        for (const auto* name :
                             {"left_arm_controller", "right_arm_controller", "spine_controller"})
                            active = active &&
                                     std::any_of(
                                         response->controller.begin(), response->controller.end(),
                                         [&](const auto& controller) {
                                             return controller.name == name &&
                                                    controller.state == "active";
                                         });
                        std::lock_guard<std::mutex> lock(state->mutex);
                        if (state->generation != generation) return;
                        state->controllers_ready = active;
                        state->controllers_checked = Clock::now();
                        state->checking = false;
                    });
                std::lock_guard<std::mutex> lock(state->mutex);
                if (state->checking && state->generation == generation)
                    state->pending = request.request_id;
            },
            callbacks);
    }
    Clock::time_point after(double seconds) const {
        return Clock::now() +
               std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
    }
    Result mutable_status() const {
        if (operation) {
            std::lock_guard<std::mutex> lock(operation->mutex);
            if (operation->unknown)
                return error(
                    ErrorCode::PreviousOperationNotTerminated, "previous execution not terminated");
            if (!operation->terminal) return error(ErrorCode::Busy, "execution active");
        }
        if (busy) return error(ErrorCode::Busy, "planning/execution active");
        return {};
    }
    Result begin(const std::shared_ptr<Operation>& record = {}) {
        std::lock_guard<std::mutex> lock(mutex);
        auto status = mutable_status();
        if (!status) return status;
        if (!initialized) return error(ErrorCode::NotInitialized, "initialize first");
        busy = true;
        planning_canceled = false;
        if (record) {
            auto channel = monitor->claim(record);
            if (!channel) {
                busy = false;
                return channel;
            }
            operation = record;
            record->measured = measured;
            record->ros_clock = node->get_clock();
            const auto publisher = monitor->stop;
            record->send_cancel = [publisher] {
                std_msgs::msg::String event;
                event.data = "stop";
                publisher->publish(event);
            };
        }
        return {};
    }
    void release() {
        std::lock_guard<std::mutex> lock(mutex);
        busy = false;
    }
    Result copy_parameters(Clock::time_point deadline) {
        if (node->has_parameter("robot_description") &&
            node->has_parameter("robot_description_semantic"))
            return {};
        using List = rcl_interfaces::srv::ListParameters;
        using Get = rcl_interfaces::srv::GetParameters;
        auto list = node->create_client<List>(
            "/move_group/list_parameters", rmw_qos_profile_services_default, callbacks);
        auto get = node->create_client<Get>(
            "/move_group/get_parameters", rmw_qos_profile_services_default, callbacks);
        if (!list->wait_for_service(std::max(Clock::duration::zero(), deadline - Clock::now())))
            return error(ErrorCode::NotReady, "MoveGroup parameters unavailable");
        auto query = std::make_shared<List::Request>();
        query->depth = 0;
        auto names = list->async_send_request(query);
        if (names.wait_until(deadline) != std::future_status::ready) {
            list->remove_pending_request(names);
            return error(ErrorCode::Timeout, "parameter list deadline");
        }
        auto request = std::make_shared<Get::Request>();
        const auto listing = names.get();
        for (const auto& name : listing->result.names)
            if (name.compare(0, 17, "robot_description") == 0 || name.compare(0, 5, "ompl.") == 0)
                request->names.push_back(name);
        auto values = get->async_send_request(request);
        if (values.wait_until(deadline) != std::future_status::ready) {
            get->remove_pending_request(values);
            return error(ErrorCode::Timeout, "parameter read deadline");
        }
        const auto response = values.get();
        for (std::size_t i = 0; i < request->names.size(); ++i)
            if (response->values.at(i).type !=
                    rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET &&
                !node->has_parameter(request->names[i]))
                node->declare_parameter(
                    request->names[i], rclcpp::ParameterValue(response->values.at(i)));
        return {};
    }
    Result current(moveit::core::RobotStatePtr& state) const {
        std::lock_guard<std::mutex> lock(measured->mutex);
        if (!measured->controllers_ready ||
            Clock::now() - measured->controllers_checked > std::chrono::seconds(1))
            return error(
                ErrorCode::NotReady, "arm/spine controllers not ACTIVE or readiness stale");
        if (!model || measured->required.empty())
            return error(ErrorCode::NotReady, "model unavailable");
        state = std::make_shared<moveit::core::RobotState>(model);
        state->setToDefaultValues();
        for (const auto& name : measured->required) {
            const auto found = measured->joints.find(name);
            if (found == measured->joints.end() ||
                Clock::now() - found->second.received >
                    std::chrono::duration<double>(state_timeout) ||
                (node->now() - found->second.stamp).seconds() > state_timeout ||
                (node->now() - found->second.stamp).seconds() < -.1)
                return error(ErrorCode::NotReady, "complete fresh state unavailable: " + name);
            state->setVariablePosition(name, found->second.position);
            state->setVariableVelocity(name, found->second.velocity);
        }
        state->update();
        if (!state->satisfiesBounds())
            return error(ErrorCode::InvalidStartState, "measured state out of bounds");
        return {};
    }
    Result scene(
        planning_scene::PlanningScenePtr& output, moveit::core::RobotStatePtr& state,
        bool explicit_start) {
        auto status = current(state);
        if (!status) return status;
        const auto deadline = after(scene_timeout);
        if (!scene_service->service_is_ready())
            return error(ErrorCode::NotReady, "planning scene unavailable");
        auto request = std::make_shared<SceneService::Request>();
        request->components.components = 1023;
        auto response = scene_service->async_send_request(request);
        if (response.wait_until(deadline) != std::future_status::ready) {
            scene_service->remove_pending_request(response);
            return error(ErrorCode::Timeout, "planning scene deadline");
        }
        output = std::make_shared<planning_scene::PlanningScene>(model);
        output->setPlanningSceneMsg(response.get()->scene);
        auto with_attachments = output->getCurrentState();
        for (const auto& name : model->getVariableNames()) {
            with_attachments.setVariablePosition(name, state->getVariablePosition(name));
            with_attachments.setVariableVelocity(name, state->getVariableVelocity(name));
        }
        state = std::make_shared<moveit::core::RobotState>(with_attachments);
        {
            std::lock_guard<std::mutex> lock(measured->mutex);
            snapshot_stamp = measured->joints.at(kSpine).stamp;
        }
        if (explicit_start && start_state)
            for (std::size_t i = 0; i < start_state->joint_state.name.size(); ++i)
                state->setVariablePosition(
                    start_state->joint_state.name[i], start_state->joint_state.position[i]);
        state->update();
        output->setCurrentState(*state);
        if (!state->satisfiesBounds() ||
            !output->isStateValid(*state, explicit_start ? constraints : Constraints(), "")) {
            collision_detection::CollisionRequest request;
            collision_detection::CollisionResult collisions;
            request.contacts = true;
            request.max_contacts = 4;
            request.max_contacts_per_pair = 1;
            output->checkCollision(request, collisions, *state);
            std::ostringstream diagnostic;
            diagnostic << "start collision/bounds/path constraints";
            for (const auto& pair : collisions.contacts)
                diagnostic << "; " << pair.first.first << "/" << pair.first.second
                           << " depth=" << pair.second.front().depth;
            return error(ErrorCode::InvalidStartState, diagnostic.str());
        }
        return {};
    }
    Result frame(
        const std::string& name, const moveit::core::RobotState& state,
        Eigen::Isometry3d& output) const {
        if (name == model->getModelFrame()) {
            output.setIdentity();
            return {};
        }
        if (model->hasLinkModel(name)) {
            output = state.getGlobalLinkTransform(name);
            return {};
        }
        try {
            const auto t = buffer->lookupTransform(
                model->getModelFrame(), name, snapshot_stamp, rclcpp::Duration::from_seconds(0));
            geometry_msgs::msg::Pose p;
            p.position.x = t.transform.translation.x;
            p.position.y = t.transform.translation.y;
            p.position.z = t.transform.translation.z;
            p.orientation = t.transform.rotation;
            output = transform(p);
            return {};
        } catch (const tf2::TransformException& e) {
            return error(ErrorCode::InvalidTarget, e.what());
        }
    }
    Result insert(RobotGroup group, Target target) {
        std::lock_guard<std::mutex> lock(mutex);
        auto status = mutable_status();
        if (!status) return status;
        if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
        if (!groups.count(group)) return error(ErrorCode::GroupNotAdded, "add group first");
        if (targets.count(group))
            return error(ErrorCode::TargetAlreadyExists, "remove target before replacement");
        targets.emplace(group, std::move(target));
        return {};
    }
    Result support() const {
        if (targets.empty()) return error(ErrorCode::NoTarget, "at least one target required");
        if (groups.size() == 1 && groups.count(RobotGroup::Spine) &&
            !targets.count(RobotGroup::Spine))
            return error(ErrorCode::NoTarget, "spine needs explicit target");
        if (groups.size() == 3 && !targets.count(RobotGroup::Spine))
            return error(
                ErrorCode::UnsupportedCombination,
                "dual arm with free shared spine requires Gate D");
        return {};
    }
    Result goals(
        const moveit::core::RobotState& start, Constraints& merged,
        std::map<RobotGroup, Eigen::Isometry3d>& poses) const {
        for (const auto& entry : targets) {
            const auto group = entry.first;
            const unsigned i = static_cast<unsigned>(group);
            const auto* jmg = model->getJointModelGroup(kGroups[i]);
            if (const auto* values = std::get_if<std::vector<double>>(&entry.second)) {
                auto state = start;
                state.setJointGroupPositions(jmg, *values);
                state.update();
                const auto part =
                    kinematic_constraints::constructGoalConstraints(state, jmg, joint_tolerance[i]);
                merged.joint_constraints.insert(
                    merged.joint_constraints.end(), part.joint_constraints.begin(),
                    part.joint_constraints.end());
                continue;
            }
            auto goal = start.getGlobalLinkTransform(kTips[i]);
            Eigen::Isometry3d conversion;
            if (const auto* pose = std::get_if<Pose>(&entry.second)) {
                auto status = frame(pose->header.frame_id, start, conversion);
                if (!status) return status;
                goal = conversion * transform(pose->pose);
            } else if (const auto* point = std::get_if<Point>(&entry.second)) {
                auto status = frame(point->header.frame_id, start, conversion);
                if (!status) return status;
                goal.translation() =
                    conversion * Eigen::Vector3d(point->point.x, point->point.y, point->point.z);
            } else {
                const auto& q = std::get<Quaternion>(entry.second);
                auto status = frame(q.header.frame_id, start, conversion);
                if (!status) return status;
                goal.linear() = conversion.rotation() *
                                Eigen::Quaterniond(
                                    q.quaternion.w, q.quaternion.x, q.quaternion.y, q.quaternion.z)
                                    .toRotationMatrix();
            }
            poses[group] = goal;
            auto part = kinematic_constraints::constructGoalConstraints(
                kTips[i], pose_message(goal, model->getModelFrame()), position_tolerance[i],
                orientation_tolerance[i]);
            if (!std::holds_alternative<Quaternion>(entry.second))
                merged.position_constraints.insert(
                    merged.position_constraints.end(), part.position_constraints.begin(),
                    part.position_constraints.end());
            if (!std::holds_alternative<Point>(entry.second))
                merged.orientation_constraints.insert(
                    merged.orientation_constraints.end(), part.orientation_constraints.begin(),
                    part.orientation_constraints.end());
        }
        return {};
    }
    Result sample(
        const planning_scene::PlanningScene& scene, const moveit::core::RobotState& start,
        const Constraints& merged, const std::map<RobotGroup, Eigen::Isometry3d>& poses,
        moveit::core::RobotState& output) const {
        const auto deadline = after(sampling_timeout);
        std::mt19937 random(1101);
        bool found_ik = false;
        for (unsigned trial = 0; trial < 200 && Clock::now() < deadline; ++trial) {
            output = start;
            for (const auto& entry : targets)
                if (const auto* values = std::get_if<std::vector<double>>(&entry.second))
                    output.setJointGroupPositions(
                        kGroups[static_cast<unsigned>(entry.first)], *values);
            output.update();
            bool solved = true;
            for (const auto& entry : poses) {
                const auto side = static_cast<unsigned>(entry.first);
                const bool free_spine =
                    groups.count(RobotGroup::Spine) && !targets.count(RobotGroup::Spine);
                const auto* chain = model->getJointModelGroup(
                    free_spine ? (side == 0 ? "left_arm_spine" : "right_arm_spine")
                               : kGroups[side]);
                const auto solver = chain->getSolverInstance();
                if (!solver) return error(ErrorCode::IKFailed, "IK plugin unavailable");
                std::vector<double> seed;
                output.copyJointGroupPositions(chain, seed);
                for (std::size_t j = 0; j < seed.size(); ++j) {
                    const auto& name = chain->getVariableNames()[j];
                    const auto& bounds = model->getVariableBounds(name);
                    double lower = bounds.min_position_, upper = bounds.max_position_;
                    for (const auto& constraint : constraints.joint_constraints)
                        if (constraint.joint_name == name) {
                            lower =
                                std::max(lower, constraint.position - constraint.tolerance_below);
                            upper =
                                std::min(upper, constraint.position + constraint.tolerance_above);
                            if (!trial) seed[j] = constraint.position;
                        }
                    if (lower > upper)
                        return error(
                            ErrorCode::InvalidConstraint,
                            "joint constraint outside mechanical limits");
                    if (trial)
                        seed[j] = std::uniform_real_distribution<double>(lower, upper)(random);
                    else
                        seed[j] = std::clamp(seed[j], lower, upper);
                }
                auto target_pose = entry.second;
                if (trial && std::holds_alternative<Point>(targets.at(entry.first))) {
                    std::normal_distribution<double> normal;
                    Eigen::Quaterniond orientation(
                        normal(random), normal(random), normal(random), normal(random));
                    orientation.normalize();
                    target_pose.linear() = orientation.toRotationMatrix();
                } else if (trial && std::holds_alternative<Quaternion>(targets.at(entry.first))) {
                    auto free_position = output;
                    free_position.setJointGroupPositions(chain, seed);
                    free_position.update();
                    target_pose.translation() =
                        free_position.getGlobalLinkTransform(kTips[side]).translation();
                }
                Eigen::Isometry3d root = Eigen::Isometry3d::Identity();
                if (solver->getBaseFrame() != model->getModelFrame())
                    root = output.getGlobalLinkTransform(solver->getBaseFrame());
                std::vector<double> solution;
                moveit_msgs::msg::MoveItErrorCodes code;
                if (!solver->getPositionIK(
                        pose_message(root.inverse() * target_pose, solver->getBaseFrame()).pose,
                        seed, solution, code)) {
                    solved = false;
                    break;
                }
                output.setJointGroupPositions(chain, solution);
                output.update();
            }
            if (!solved) continue;
            found_ik = true;
            // Prefer redundant IK solutions that keep physical tracking error
            // inside mechanical limits. Explicit joint targets retain their
            // specified values and undergo ordinary bounds validation.
            bool margin = true;
            for (const auto& entry : poses) {
                const bool free_spine =
                    groups.count(RobotGroup::Spine) && !targets.count(RobotGroup::Spine);
                const std::string chain = kGroups[static_cast<unsigned>(entry.first)] +
                                          std::string(free_spine ? "_spine" : "");
                for (const auto& name : model->getJointModelGroup(chain)->getVariableNames()) {
                    const auto& bounds = model->getVariableBounds(name);
                    const double reserve = name == kSpine ? .003 : .02;
                    const double position = output.getVariablePosition(name);
                    if (bounds.position_bounded_ && (position < bounds.min_position_ + reserve ||
                                                     position > bounds.max_position_ - reserve))
                        margin = false;
                }
            }
            if (!margin) continue;
            if (output.satisfiesBounds() && scene.isStateValid(output, merged, "") &&
                scene.isStateConstrained(output, constraints))
                return {};
        }
        return error(
            found_ik ? ErrorCode::GoalSamplingFailed : ErrorCode::IKFailed,
            "bounded IK/full-state AND sampling exhausted");
    }
    Result validate(
        const Plan& plan, const planning_scene::PlanningScene& scene,
        const moveit::core::RobotState& actual, double& duration) const {
        if (!plan.valid_ || plan.groups_.empty() ||
            !plan.trajectory_.multi_dof_joint_trajectory.points.empty())
            return error(ErrorCode::InvalidPlan, "invalid/default or multi-DOF plan");
        if (plan.cartesian_fraction_ && *plan.cartesian_fraction_ < 1 - 1e-6)
            return error(ErrorCode::IncompleteCartesianPath, "partial path is diagnostic only");
        std::set<RobotGroup> selected;
        for (const auto group : plan.groups_)
            if (!valid_group(group) || !selected.insert(group).second)
                return error(ErrorCode::InvalidPlan, "invalid/duplicate plan groups");
        const auto* jmg = model->getJointModelGroup(resolve(selected));
        const auto& path = plan.trajectory_.joint_trajectory;
        const std::set<std::string> expected(
            jmg->getVariableNames().begin(), jmg->getVariableNames().end());
        const std::set<std::string> names(path.joint_names.begin(), path.joint_names.end());
        if (names != expected || names.size() != path.joint_names.size() || path.points.empty())
            return error(ErrorCode::InvalidPlan, "trajectory joint set mismatch");
        const auto& initial = plan.start_state_.joint_state;
        if (plan.start_state_.is_diff || initial.name.size() != initial.position.size() ||
            initial.header.stamp.sec < 0 || initial.header.stamp.nanosec >= 1000000000U)
            return error(ErrorCode::InvalidPlan, "invalid plan start state");
        std::set<std::string> initial_names;
        for (std::size_t i = 0; i < initial.name.size(); ++i)
            if (std::find(
                    model->getVariableNames().begin(), model->getVariableNames().end(),
                    initial.name[i]) == model->getVariableNames().end() ||
                !initial_names.insert(initial.name[i]).second ||
                !std::isfinite(initial.position[i]))
                return error(ErrorCode::InvalidPlan, "unknown/duplicate/nonfinite plan start");
        for (const auto& name : model->getVariableNames())
            if (!model->getJointOfVariable(name)->getMimic() && !initial_names.count(name))
                return error(ErrorCode::InvalidPlan, "incomplete plan start");
        for (const auto* values : {&initial.velocity, &initial.effort}) {
            if (!values->empty() && values->size() != initial.name.size())
                return error(ErrorCode::InvalidPlan, "plan start vector size");
            for (double value : *values)
                if (!std::isfinite(value))
                    return error(ErrorCode::InvalidPlan, "nonfinite plan start dynamics");
        }
        auto start = actual;
        // The plan freezes joint positions, while collision checks must retain
        // attachments from the freshly queried scene rather than replay old ones.
        for (std::size_t i = 0; i < initial.name.size(); ++i)
            start.setVariablePosition(initial.name[i], initial.position[i]);
        start.update();
        if (!start.satisfiesBounds())
            return error(ErrorCode::InvalidPlan, "plan start outside bounds");
        for (const auto& name : model->getJointModelGroup("dual_arm_spine")->getVariableNames()) {
            const double tolerance = name == kSpine ? spine_start_tolerance : arm_start_tolerance;
            if (std::abs(actual.getVariablePosition(name) - start.getVariablePosition(name)) >
                tolerance)
                return error(ErrorCode::ExecutionStartStateMismatch, "stale plan start " + name);
        }
        double previous = -1;
        for (const auto& point : path.points) {
            const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U ||
                time <= previous || point.positions.size() != names.size())
                return error(ErrorCode::InvalidPlan, "point/time invalid");
            auto state = start;
            for (const auto* values : {&point.velocities, &point.accelerations, &point.effort}) {
                if (!values->empty() && values->size() != names.size())
                    return error(ErrorCode::InvalidPlan, "vector size mismatch");
                for (double value : *values)
                    if (!std::isfinite(value))
                        return error(ErrorCode::InvalidPlan, "nonfinite dynamics");
            }
            for (std::size_t i = 0; i < path.joint_names.size(); ++i) {
                const auto& name = path.joint_names[i];
                if (!std::isfinite(point.positions[i]))
                    return error(ErrorCode::InvalidPlan, "nonfinite position");
                state.setVariablePosition(name, point.positions[i]);
                const auto& bounds = model->getVariableBounds(name);
                if ((!point.velocities.empty() &&
                     std::abs(point.velocities[i]) > bounds.max_velocity_ + 1e-6) ||
                    (!point.accelerations.empty() && bounds.acceleration_bounded_ &&
                     std::abs(point.accelerations[i]) > bounds.max_acceleration_ + 1e-6))
                    return error(ErrorCode::InvalidPlan, "dynamics outside limits");
            }
            state.update();
            if (!state.satisfiesBounds() || !scene.isStateValid(state))
                return error(ErrorCode::InvalidPlan, "waypoint collision/bounds");
            if (previous < 0)
                for (const auto& name : path.joint_names)
                    if (std::abs(
                            state.getVariablePosition(name) - start.getVariablePosition(name)) >
                        (name == kSpine ? spine_start_tolerance : arm_start_tolerance))
                        return error(ErrorCode::InvalidPlan, "first point differs from plan start");
            previous = time;
        }
        duration = previous;
        return {};
    }
    Result await_canceled(const std::shared_ptr<Operation>& record) {
        // A stop event has no Action cancel ACK. Await the application fence.
        return await_termination(record, after(cancel_timeout + terminal_timeout));
    }
};

MoveGroup::MoveGroup(const rclcpp::Node::SharedPtr& node) {
    if (!node) throw std::invalid_argument("MoveGroup requires node");
    impl_ = std::make_unique<Impl>(node);
}
MoveGroup::~MoveGroup() {
    stop();
    const auto record = impl_->operation;
    if (record) {
        std::lock_guard<std::mutex> lock(record->mutex);
        if (!record->terminal) {
            // Only unresolved communication/identity survives owner destruction.
            // Late Accepted is stopped through the exclusive execution channel. Terminal result
            // releases this lease; Context shutdown is the final lifetime cap.
            record->retained_client = impl_->execution;
            record->retained_states = impl_->joint_states;
            impl_->node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
                [record] {
                    std::lock_guard<std::mutex> lock(record->mutex);
                    record->retained_client.reset();
                    record->retained_states.reset();
                    record->monitor.reset();
                });
        }
    }
}
Result MoveGroup::initialize(std::chrono::milliseconds timeout) {
    if (timeout.count() <= 0)
        return error(ErrorCode::Timeout, "positive initialization deadline required");
    std::unique_lock<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!impl_->settings_valid)
        return error(ErrorCode::InvalidConstraint, "invalid configured timeout/tolerance/jump");
    impl_->busy = true;
    struct ResetInitializing {
        bool& busy;
        ~ResetInitializing() { busy = false; }
    } initializing{impl_->busy};
    impl_->initialized = false;
    const auto deadline = Clock::now() + timeout;
    try {
        status = impl_->copy_parameters(deadline);
        if (!status) return status;
        if (!impl_->node->has_parameter("robot_description") ||
            !impl_->node->has_parameter("robot_description_semantic"))
            return error(ErrorCode::NotReady, "robot descriptions absent");
        if (!impl_->model || !impl_->pipeline) {
            impl_->loader = std::make_unique<robot_model_loader::RobotModelLoader>(impl_->node);
            impl_->model = impl_->loader->getModel();
            if (!impl_->model) return error(ErrorCode::NotReady, "model unavailable");
            for (const auto& name :
                 {"left_arm", "right_arm", "spine", "dual_arm", "left_arm_spine", "right_arm_spine",
                  "dual_arm_spine"})
                if (!impl_->model->hasJointModelGroup(name))
                    return error(
                        ErrorCode::InvalidGroup, "missing exact SRDF group " + std::string(name));
            {
                std::lock_guard<std::mutex> state_lock(impl_->measured->mutex);
                for (const auto& name : impl_->model->getVariableNames())
                    if (!impl_->model->getJointOfVariable(name)->getMimic())
                        impl_->measured->required.insert(name);
            }
            RestoreOmplOutput restore_output;
            impl_->pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
                impl_->model, impl_->node, "ompl");
        }
        if (!impl_->execution->wait_for_action_server(
                std::max(Clock::duration::zero(), deadline - Clock::now())) ||
            !impl_->scene_service->wait_for_service(
                std::max(Clock::duration::zero(), deadline - Clock::now())))
            return error(ErrorCode::Timeout, "readiness deadline exceeded");
        moveit::core::RobotStatePtr state;
        do {
            status = impl_->current(state);
            if (status) break;
            if (Clock::now() >= deadline)
                return error(
                    ErrorCode::Timeout, "complete fresh state deadline: " + status.message);
            // Condition-variable readiness polling has a single bounded deadline.
            std::condition_variable pending;
            pending.wait_for(lock, std::chrono::milliseconds(10));
        } while (!status);
        if (Clock::now() >= deadline)
            return error(ErrorCode::Timeout, "initialization deadline exceeded");
        impl_->initialized = true;
        return {};
    } catch (const std::exception& e) {
        return error(ErrorCode::InternalError, e.what());
    }
}
bool MoveGroup::is_ready() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->initialized || !impl_->monitor->available() ||
        !impl_->execution->action_server_is_ready() || !impl_->scene_service->service_is_ready())
        return false;
    if (impl_->operation) {
        std::lock_guard<std::mutex> op(impl_->operation->mutex);
        if (impl_->operation->unknown) return false;
    }
    moveit::core::RobotStatePtr state;
    return static_cast<bool>(impl_->current(state));
}
Result MoveGroup::add_group(RobotGroup group) { return add_groups({group}); }
Result MoveGroup::add_groups(std::initializer_list<RobotGroup> groups) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    auto result = impl_->groups;
    for (const auto group : groups) {
        if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
        if (!result.insert(group).second)
            return error(ErrorCode::DuplicateGroup, "duplicate group");
    }
    impl_->groups = std::move(result);
    return {};
}
Result MoveGroup::remove_group(RobotGroup group) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (!impl_->groups.count(group)) return error(ErrorCode::GroupNotAdded, "group absent");
    if (impl_->targets.count(group)) return error(ErrorCode::GroupHasTarget, "remove target first");
    impl_->groups.erase(group);
    return {};
}
Result MoveGroup::clear_groups() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!impl_->targets.empty()) return error(ErrorCode::GroupHasTarget, "clear targets first");
    impl_->groups.clear();
    return {};
}
bool MoveGroup::has_group(RobotGroup group) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->groups.count(group);
}
std::vector<RobotGroup> MoveGroup::get_groups() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return {impl_->groups.begin(), impl_->groups.end()};
}
Result MoveGroup::add_pose_target(RobotGroup group, const Pose& pose) {
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (group == RobotGroup::Spine)
        return error(ErrorCode::UnsupportedTarget, "spine accepts joint/named targets");
    if (!valid_header(pose.header) || !valid_point(pose.pose.position) ||
        !valid_quaternion(pose.pose.orientation))
        return error(ErrorCode::InvalidTarget, "invalid pose/header/quaternion");
    return impl_->insert(group, pose);
}
Result MoveGroup::add_position_target(RobotGroup group, const Point& point) {
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (group == RobotGroup::Spine)
        return error(ErrorCode::UnsupportedTarget, "spine accepts joint/named targets");
    if (!valid_header(point.header) || !valid_point(point.point))
        return error(ErrorCode::InvalidTarget, "invalid point/header");
    return impl_->insert(group, point);
}
Result MoveGroup::add_orientation_target(RobotGroup group, const Quaternion& q) {
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (group == RobotGroup::Spine)
        return error(ErrorCode::UnsupportedTarget, "spine accepts joint/named targets");
    if (!valid_header(q.header) || !valid_quaternion(q.quaternion))
        return error(ErrorCode::InvalidTarget, "invalid quaternion/header");
    return impl_->insert(group, q);
}
Result MoveGroup::add_joint_position_target(RobotGroup group, double position) {
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (group != RobotGroup::Spine)
        return error(ErrorCode::UnsupportedTarget, "scalar target only for spine");
    return add_joint_position_target(group, std::vector<double>{position});
}
Result MoveGroup::add_joint_position_target(
    RobotGroup group, const std::vector<double>& positions) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (!impl_->groups.count(group)) return error(ErrorCode::GroupNotAdded, "add group first");
    if (impl_->targets.count(group)) return error(ErrorCode::TargetAlreadyExists, "target exists");
    if (!impl_->initialized) return error(ErrorCode::NotInitialized, "model required");
    const auto* jmg = impl_->model->getJointModelGroup(kGroups[static_cast<unsigned>(group)]);
    if (positions.size() != jmg->getVariableCount())
        return error(ErrorCode::InvalidTarget, "target size mismatch");
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto& b = impl_->model->getVariableBounds(jmg->getVariableNames()[i]);
        if (!std::isfinite(positions[i]) ||
            (b.position_bounded_ &&
             (positions[i] < b.min_position_ || positions[i] > b.max_position_)))
            return error(ErrorCode::InvalidTarget, "target outside limits");
    }
    impl_->targets.emplace(group, positions);
    return {};
}
Result MoveGroup::add_named_target(RobotGroup group, std::string_view name) {
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    std::vector<double> values;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        auto status = impl_->mutable_status();
        if (!status) return status;
        if (!impl_->initialized) return error(ErrorCode::NotInitialized, "model required");
        const auto* jmg = impl_->model->getJointModelGroup(kGroups[static_cast<unsigned>(group)]);
        moveit::core::RobotState state(impl_->model);
        state.setToDefaultValues();
        if (name.empty() || !state.setToDefaultValues(jmg, std::string(name)))
            return error(ErrorCode::InvalidTarget, "unknown named target");
        state.copyJointGroupPositions(jmg, values);
    }
    return add_joint_position_target(group, values);
}
Result MoveGroup::remove_target(RobotGroup group) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (!impl_->groups.count(group)) return error(ErrorCode::GroupNotAdded, "group absent");
    impl_->targets.erase(group);
    return {};
}
Result MoveGroup::clear_targets() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    impl_->targets.clear();
    return {};
}
bool MoveGroup::has_target(RobotGroup group) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->targets.count(group);
}
Result MoveGroup::set_start_state(const moveit_msgs::msg::RobotState& state) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!impl_->initialized) return error(ErrorCode::NotInitialized, "model required");
    const auto& joints = state.joint_state;
    if (state.is_diff || !state.multi_dof_joint_state.joint_names.empty() ||
        joints.name.size() != joints.position.size() || joints.header.stamp.sec < 0 ||
        joints.header.stamp.nanosec >= 1000000000U || !state.attached_collision_objects.empty())
        return error(
            ErrorCode::InvalidStartState,
            "full coherent joint state required; attachments come from scene");
    std::set<std::string> names;
    for (std::size_t i = 0; i < joints.name.size(); ++i)
        if (!(std::find(
                  impl_->model->getVariableNames().begin(), impl_->model->getVariableNames().end(),
                  joints.name[i]) != impl_->model->getVariableNames().end()) ||
            !names.insert(joints.name[i]).second || !std::isfinite(joints.position[i]))
            return error(ErrorCode::InvalidStartState, "unknown/duplicate/nonfinite state joint");
    for (const auto& name : impl_->model->getVariableNames())
        if (!names.count(name))
            return error(ErrorCode::InvalidStartState, "incomplete explicit start state");
    for (const auto* values : {&joints.velocity, &joints.effort}) {
        if (!values->empty() && values->size() != names.size())
            return error(ErrorCode::InvalidStartState, "state vector size mismatch");
        for (double value : *values)
            if (!std::isfinite(value))
                return error(ErrorCode::InvalidStartState, "nonfinite state dynamics");
    }
    moveit::core::RobotState parsed(impl_->model);
    parsed.setToDefaultValues();
    moveit::core::robotStateMsgToRobotState(state, parsed);
    if (!parsed.satisfiesBounds()) return error(ErrorCode::InvalidStartState, "state limits");
    impl_->start_state = state;
    return {};
}
Result MoveGroup::set_start_state_to_current_state() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    impl_->start_state.reset();
    return {};
}
Result MoveGroup::set_goal_joint_tolerance(RobotGroup group, double value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (!positive(value))
        return error(ErrorCode::InvalidConstraint, "positive finite tolerance required");
    impl_->joint_tolerance[static_cast<unsigned>(group)] = value;
    return {};
}
Result MoveGroup::set_goal_position_tolerance(RobotGroup group, double value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (group == RobotGroup::Spine)
        return error(ErrorCode::UnsupportedTarget, "spine has no pose goal");
    if (!positive(value))
        return error(ErrorCode::InvalidConstraint, "positive finite tolerance required");
    impl_->position_tolerance[static_cast<unsigned>(group)] = value;
    return {};
}
Result MoveGroup::set_goal_orientation_tolerance(RobotGroup group, double value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid group");
    if (group == RobotGroup::Spine)
        return error(ErrorCode::UnsupportedTarget, "spine has no pose goal");
    if (!positive(value))
        return error(ErrorCode::InvalidConstraint, "positive finite tolerance required");
    impl_->orientation_tolerance[static_cast<unsigned>(group)] = value;
    return {};
}
Result MoveGroup::add_path_constraint(const Constraints& constraint) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!impl_->initialized) return error(ErrorCode::NotInitialized, "model required");
    if (!constraint.visibility_constraints.empty() ||
        (constraint.joint_constraints.empty() && constraint.position_constraints.empty() &&
         constraint.orientation_constraints.empty()))
        return error(ErrorCode::InvalidConstraint, "empty/unsupported visibility constraint");
    auto merged = impl_->constraints;
    const auto known_frame = [&](const std::string& frame) {
        if (frame == impl_->model->getModelFrame() || impl_->model->hasLinkModel(frame))
            return true;
        return impl_->buffer->canTransform(
            impl_->model->getModelFrame(), frame, rclcpp::Time(0),
            rclcpp::Duration::from_seconds(0));
    };

    for (const auto& item : constraint.joint_constraints) {
        if (!(std::find(
                  impl_->model->getVariableNames().begin(), impl_->model->getVariableNames().end(),
                  item.joint_name) != impl_->model->getVariableNames().end()) ||
            !std::isfinite(item.position) || !std::isfinite(item.tolerance_above) ||
            !std::isfinite(item.tolerance_below) || item.tolerance_above < 0 ||
            item.tolerance_below < 0 || !positive(item.weight))
            return error(ErrorCode::InvalidConstraint, "invalid joint constraint");
        const auto& bounds = impl_->model->getVariableBounds(item.joint_name);
        if (bounds.position_bounded_ &&
            (item.position - item.tolerance_below > bounds.max_position_ ||
             item.position + item.tolerance_above < bounds.min_position_))
            return error(ErrorCode::InvalidConstraint, "joint constraint excludes model bounds");
        const auto existing = std::find_if(
            merged.joint_constraints.begin(), merged.joint_constraints.end(),
            [&](const auto& c) { return c.joint_name == item.joint_name; });
        if (existing != merged.joint_constraints.end()) {
            const double lower = std::max(
                item.position - item.tolerance_below,
                existing->position - existing->tolerance_below);
            const double upper = std::min(
                item.position + item.tolerance_above,
                existing->position + existing->tolerance_above);
            if (lower > upper)
                return error(
                    ErrorCode::InvalidConstraint, "conflicting joint constraint intervals");
            existing->position = (lower + upper) / 2;
            existing->tolerance_below = existing->tolerance_above = (upper - lower) / 2;
        } else
            merged.joint_constraints.push_back(item);
    }
    for (const auto& item : constraint.orientation_constraints) {
        if (!valid_header(item.header) || !known_frame(item.header.frame_id) ||
            !impl_->model->hasLinkModel(item.link_name) || !valid_quaternion(item.orientation) ||
            !positive(item.weight) || !std::isfinite(item.absolute_x_axis_tolerance) ||
            !std::isfinite(item.absolute_y_axis_tolerance) ||
            !std::isfinite(item.absolute_z_axis_tolerance) || item.absolute_x_axis_tolerance < 0 ||
            item.absolute_y_axis_tolerance < 0 || item.absolute_z_axis_tolerance < 0)
            return error(ErrorCode::InvalidConstraint, "invalid orientation constraint");
        const auto old = std::find_if(
            merged.orientation_constraints.begin(), merged.orientation_constraints.end(),
            [&](const auto& c) { return c.link_name == item.link_name; });
        if (old != merged.orientation_constraints.end() && *old == item) continue;
        if (old != merged.orientation_constraints.end() &&
            old->header.frame_id == item.header.frame_id) {
            const Eigen::Quaterniond a(
                old->orientation.w, old->orientation.x, old->orientation.y, old->orientation.z);
            const Eigen::Quaterniond b(
                item.orientation.w, item.orientation.x, item.orientation.y, item.orientation.z);
            const double angle = a.angularDistance(b);
            const double radius = old->absolute_x_axis_tolerance + old->absolute_y_axis_tolerance +
                                  old->absolute_z_axis_tolerance + item.absolute_x_axis_tolerance +
                                  item.absolute_y_axis_tolerance + item.absolute_z_axis_tolerance;
            if (angle > radius + 1e-9)
                return error(ErrorCode::InvalidConstraint, "conflicting orientation regions");
        }
        merged.orientation_constraints.push_back(item);
    }
    for (const auto& item : constraint.position_constraints) {
        if (!valid_header(item.header) || !known_frame(item.header.frame_id) ||
            !impl_->model->hasLinkModel(item.link_name) || !positive(item.weight) ||
            !std::isfinite(item.target_point_offset.x) ||
            !std::isfinite(item.target_point_offset.y) ||
            !std::isfinite(item.target_point_offset.z) || !item.constraint_region.meshes.empty() ||
            item.constraint_region.primitives.empty() ||
            item.constraint_region.primitives.size() !=
                item.constraint_region.primitive_poses.size())
            return error(ErrorCode::InvalidConstraint, "invalid position constraint region");
        for (std::size_t i = 0; i < item.constraint_region.primitives.size(); ++i) {
            const auto& shape = item.constraint_region.primitives[i];
            const auto required =
                shape.type == shape.SPHERE
                    ? 1u
                    : (shape.type == shape.BOX
                           ? 3u
                           : (shape.type == shape.CYLINDER || shape.type == shape.CONE ? 2u : 0u));
            if (!required || shape.dimensions.size() != required ||
                !valid_point(item.constraint_region.primitive_poses[i].position) ||
                !valid_quaternion(item.constraint_region.primitive_poses[i].orientation))
                return error(ErrorCode::InvalidConstraint, "invalid primitive");
            for (double d : shape.dimensions)
                if (!positive(d))
                    return error(ErrorCode::InvalidConstraint, "invalid primitive dimension");
        }
        const auto old = std::find_if(
            merged.position_constraints.begin(), merged.position_constraints.end(),
            [&](const auto& c) { return c.link_name == item.link_name; });
        if (old != merged.position_constraints.end() && *old == item) continue;
        if (old != merged.position_constraints.end() &&
            old->header.frame_id == item.header.frame_id &&
            old->target_point_offset == item.target_point_offset &&
            old->constraint_region.primitives.size() == 1 &&
            item.constraint_region.primitives.size() == 1 &&
            old->constraint_region.primitives.front().type ==
                shape_msgs::msg::SolidPrimitive::SPHERE &&
            item.constraint_region.primitives.front().type ==
                shape_msgs::msg::SolidPrimitive::SPHERE) {
            const auto& a = old->constraint_region.primitive_poses.front().position;
            const auto& b = item.constraint_region.primitive_poses.front().position;
            const double distance = Eigen::Vector3d(a.x - b.x, a.y - b.y, a.z - b.z).norm();
            if (distance > old->constraint_region.primitives.front().dimensions.front() +
                               item.constraint_region.primitives.front().dimensions.front())
                return error(ErrorCode::InvalidConstraint, "conflicting position regions");
        }
        merged.position_constraints.push_back(item);
    }
    impl_->constraints = std::move(merged);
    return {};
}
Result MoveGroup::clear_path_constraints() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    impl_->constraints = Constraints();
    return {};
}
Result MoveGroup::set_planning_pipeline_id(std::string_view id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (id != "ompl")
        return error(ErrorCode::InvalidConstraint, "only configured ompl pipeline is available");
    impl_->pipeline_id = std::string(id);
    return {};
}
Result MoveGroup::set_planner_id(std::string_view id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (id != "RRTConnectkConfigDefault")
        return error(ErrorCode::InvalidConstraint, "planner not configured");
    impl_->planner_id = std::string(id);
    return {};
}
Result MoveGroup::set_planning_time(double value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!positive(value))
        return error(ErrorCode::InvalidConstraint, "positive finite planning time required");
    impl_->planning_time = value;
    return {};
}
Result MoveGroup::set_num_planning_attempts(std::size_t value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!value || value > 100)
        return error(ErrorCode::InvalidConstraint, "attempts outside 1..100");
    impl_->attempts = value;
    return {};
}
Result MoveGroup::set_max_velocity_scaling_factor(double value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!positive(value) || value > 1)
        return error(ErrorCode::InvalidConstraint, "scaling outside (0,1]");
    impl_->velocity = value;
    return {};
}
Result MoveGroup::set_max_acceleration_scaling_factor(double value) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto status = impl_->mutable_status();
    if (!status) return status;
    if (!positive(value) || value > 1)
        return error(ErrorCode::InvalidConstraint, "scaling outside (0,1]");
    impl_->acceleration = value;
    return {};
}
Result MoveGroup::plan(Plan& output) {
    output = {};
    auto status = impl_->begin();
    if (!status) return status;
    struct Release {
        Impl& owner;
        ~Release() { owner.release(); }
    } release{*impl_};
    try {
        status = impl_->support();
        if (!status) return status;
        planning_scene::PlanningScenePtr scene;
        moveit::core::RobotStatePtr start;
        status = impl_->scene(scene, start, true);
        if (!status) return status;
        Constraints merged;
        std::map<RobotGroup, Eigen::Isometry3d> poses;
        status = impl_->goals(*start, merged, poses);
        if (!status) return status;
        auto sampled = *start;
        status = impl_->sample(*scene, *start, merged, poses, sampled);
        if (!status) return status;
        const auto name = resolve(impl_->groups);
        const auto* jmg = impl_->model->getJointModelGroup(name);
        const auto joints = kinematic_constraints::constructGoalConstraints(sampled, jmg, .0001);
        merged.joint_constraints.insert(
            merged.joint_constraints.end(), joints.joint_constraints.begin(),
            joints.joint_constraints.end());
        planning_interface::MotionPlanRequest request;
        request.group_name = name;
        request.pipeline_id = impl_->pipeline_id;
        request.planner_id = impl_->planner_id;
        request.allowed_planning_time = impl_->planning_time;
        request.num_planning_attempts = impl_->attempts;
        request.max_velocity_scaling_factor = impl_->velocity;
        request.max_acceleration_scaling_factor = impl_->acceleration;
        moveit::core::robotStateToRobotStateMsg(*start, request.start_state);
        request.goal_constraints = {merged};
        request.path_constraints = impl_->constraints;
        planning_interface::MotionPlanResponse response;
        const auto began = Clock::now();
        const bool success = impl_->pipeline->generatePlan(scene, request, response);
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            if (impl_->planning_canceled) return error(ErrorCode::Canceled, "planning stopped");
        }
        if (!success || response.error_code_.val != response.error_code_.SUCCESS ||
            !response.trajectory_)
            return mapped(response.error_code_.val, ErrorCode::PathPlanningFailed);
        if (!scene->isStateConstrained(response.trajectory_->getLastWayPoint(), merged)) {
            std::ostringstream diagnostic;
            diagnostic << "planned endpoint violates AND goal";
            const auto& endpoint = response.trajectory_->getLastWayPoint();
            for (const auto& joint : merged.joint_constraints) {
                const double actual = endpoint.getVariablePosition(joint.joint_name);
                if (actual < joint.position - joint.tolerance_below ||
                    actual > joint.position + joint.tolerance_above)
                    diagnostic << "; " << joint.joint_name << " actual=" << actual
                               << " target=" << joint.position;
            }
            return error(ErrorCode::GoalSamplingFailed, diagnostic.str());
        }
        for (std::size_t i = 0; i < response.trajectory_->getWayPointCount(); ++i) {
            const auto& state = response.trajectory_->getWayPoint(i);
            if (!state.satisfiesBounds() || !scene->isStateValid(state, impl_->constraints, ""))
                return error(ErrorCode::PathPlanningFailed, "planned waypoint invalid");
            for (const auto& joint :
                 impl_->model->getJointModelGroup("dual_arm_spine")->getVariableNames())
                if (!jmg->hasJointModel(joint) &&
                    std::abs(state.getVariablePosition(joint) - start->getVariablePosition(joint)) >
                        1e-12)
                    return error(ErrorCode::PathPlanningFailed, "inactive joint changed");
        }
        Plan candidate;
        candidate.start_state_ = request.start_state;
        candidate.groups_ = {impl_->groups.begin(), impl_->groups.end()};
        candidate.planning_time_ = std::chrono::duration<double>(Clock::now() - began).count();
        response.trajectory_->getRobotTrajectoryMsg(candidate.trajectory_);
        candidate.valid_ = true;
        double duration = 0;
        status = impl_->validate(candidate, *scene, *start, duration);
        if (!status) return status;
        output = std::move(candidate);
        return {};
    } catch (const std::exception& e) {
        return error(ErrorCode::InternalError, e.what());
    }
}
Result MoveGroup::execute(const Plan& plan) {
    auto record = std::make_shared<Operation>();
    auto status = impl_->begin(record);
    if (!status) return status;
    struct Release {
        Impl& owner;
        ~Release() { owner.release(); }
    } release{*impl_};
    planning_scene::PlanningScenePtr scene;
    moveit::core::RobotStatePtr start;
    double duration = 0;
    try {
        if (!plan.valid()) {
            terminal(record, error(ErrorCode::InvalidPlan, "default/invalid plan"));
            return record->result;
        }
        status = impl_->scene(scene, start, false);
        if (status) status = impl_->validate(plan, *scene, *start, duration);
        if (!status) {
            terminal(record, status);
            return status;
        }
        if (!impl_->execution->action_server_is_ready()) {
            status = error(ErrorCode::ExecutionServerUnavailable, "ExecuteTrajectory unavailable");
            terminal(record, status);
            return status;
        }
        Client::SendGoalOptions options;
        options.goal_response_callback = [record](auto handle) {
            if (!handle) {
                terminal(record, error(ErrorCode::ExecutionFailed, "ExecuteTrajectory rejected"));
                return;
            }
            int reason;
            {
                std::lock_guard<std::mutex> lock(record->mutex);
                record->response = true;
                if (!record->unknown) record->state = OperationState::Active;
                record->cancel_sent = false;
                reason = record->reason;
            }
            record->changed.notify_all();
            if (reason) request_cancel(record, reason);
        };
        options.result_callback = [record](const auto& response) {
            Result result;
            if (response.code == rclcpp_action::ResultCode::CANCELED)
                result = error(ErrorCode::Canceled, "execution canceled");
            else if (!response.result)
                result = error(ErrorCode::ExecutionFailed, "missing execution result");
            else {
                if (response.code != rclcpp_action::ResultCode::SUCCEEDED ||
                    response.result->error_code.val != response.result->error_code.SUCCESS)
                    result = error(
                        ErrorCode::ExecutionFailed,
                        "execution terminal status=" +
                            std::to_string(static_cast<int>(response.code)) +
                            " MoveIt code=" + std::to_string(response.result->error_code.val));
            }
            {
                std::lock_guard<std::mutex> lock(record->mutex);
                record->parent_done = true;
                record->parent_result = std::move(result);
            }
            record->changed.notify_all();
        };
        Action::Goal goal;
        goal.trajectory = plan.trajectory_;
        const auto response_deadline = impl_->after(impl_->response_timeout);
        const auto deadline = impl_->after(duration + impl_->margin);
        {
            std::lock_guard<std::mutex> lock(record->mutex);
            record->joints = plan.trajectory_.joint_trajectory.joint_names;
            for (const auto& name : record->joints) {
                if (name == kSpine)
                    record->expected[2] = true;
                else if (name.rfind("left_", 0) == 0)
                    record->expected[0] = true;
                else if (name.rfind("right_", 0) == 0)
                    record->expected[1] = true;
            }
            record->dispatch_stamp = impl_->node->now().nanoseconds();
            record->sent = true;
        }
        impl_->execution->async_send_goal(goal, options);
        {
            std::unique_lock<std::mutex> lock(record->mutex);
            if (!record->changed.wait_until(lock, std::min(response_deadline, deadline), [&] {
                    return record->response || record->terminal || record->reason;
                })) {
                lock.unlock();
                request_cancel(record, 1);
                lock.lock();
            }
            if (record->terminal) return record->result;
        }
        Result observation;
        while (Clock::now() < deadline) {
            {
                std::unique_lock<std::mutex> lock(record->mutex);
                if (record->changed.wait_for(lock, std::chrono::milliseconds(20), [&] {
                        return record->terminal || record->reason;
                    })) {
                    if (record->terminal) return record->result;
                    break;
                }
            }
            moveit::core::RobotStatePtr actual;
            observation = impl_->current(actual);
            if (observation && !scene->isStateValid(*actual)) {
                collision_detection::CollisionRequest request;
                collision_detection::CollisionResult collision;
                request.contacts = true;
                request.max_contacts = 8;
                scene->checkCollision(request, collision, *actual);
                std::ostringstream diagnostic;
                diagnostic << "measured execution collision/bounds";
                for (const auto& contact : collision.contacts)
                    diagnostic << "; " << contact.first.first << " / " << contact.first.second;
                observation = error(ErrorCode::ExecutionFailed, diagnostic.str());
            }
            if (observation)
                for (const auto& joint :
                     impl_->model->getJointModelGroup("dual_arm_spine")->getVariableNames())
                    if (std::find(
                            plan.trajectory_.joint_trajectory.joint_names.begin(),
                            plan.trajectory_.joint_trajectory.joint_names.end(),
                            joint) == plan.trajectory_.joint_trajectory.joint_names.end() &&
                        std::abs(
                            actual->getVariablePosition(joint) -
                            start->getVariablePosition(joint)) > impl_->inactive_tolerance) {
                        observation = error(
                            ErrorCode::ExecutionFailed,
                            "measured inactive joint moved " + joint +
                                " from=" + std::to_string(start->getVariablePosition(joint)) +
                                " to=" + std::to_string(actual->getVariablePosition(joint)));
                        break;
                    }
            if (!observation) {
                request_cancel(record, 2);
                break;
            }
        }
        {
            std::lock_guard<std::mutex> lock(record->mutex);
            if (record->terminal) return record->result;
        }
        if (Clock::now() >= deadline) request_cancel(record, 1);
        status = impl_->await_canceled(record);
        if (!observation && status.code != ErrorCode::CancelFailed) return observation;
        return status;
    } catch (const std::exception& e) {
        request_cancel(record, 2);
        bool received;
        {
            std::lock_guard<std::mutex> lock(record->mutex);
            received = record->response;
        }
        if (received) return impl_->await_canceled(record);
        // A send may have succeeded before an exception. Preserve operation
        // identity and let late response/result callbacks settle the record.
        status = await_termination(record, impl_->after(impl_->terminal_timeout));
        if (status.code == ErrorCode::CancelFailed) return status;
        return error(ErrorCode::InternalError, e.what());
    }
}
Result MoveGroup::move() {
    Plan result;
    auto status = plan(result);
    return status ? execute(result) : status;
}
Result MoveGroup::stop() {
    std::shared_ptr<Operation> record;
    std::shared_ptr<planning_pipeline::PlanningPipeline> pipeline;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        record = impl_->operation;
        bool active = false;
        if (record) {
            std::lock_guard<std::mutex> op(record->mutex);
            active = !record->terminal;
        }
        if (!active && impl_->busy) {
            impl_->planning_canceled = true;
            pipeline = impl_->pipeline;
        }
        if (!active) record.reset();
    }
    if (pipeline) {
        pipeline->terminate();
        return error(ErrorCode::Canceled, "planning stop requested");
    }
    if (!record) return {};
    request_cancel(record, 2);
    return impl_->await_canceled(record);
}
Result MoveGroup::compute_cartesian_path(
    RobotGroup group, const std::vector<Pose>& waypoints, double step, CartesianPath& output) {
    output = {};
    if (!valid_group(group)) return error(ErrorCode::InvalidGroup, "invalid Cartesian group");
    if (group == RobotGroup::Spine)
        return error(
            ErrorCode::UnsupportedCombination, "Cartesian supports one arm with fixed spine");
    if (!positive(step) || waypoints.empty())
        return error(ErrorCode::InvalidTarget, "positive step and waypoints required");
    for (const auto& p : waypoints)
        if (!valid_header(p.header) || p.header.frame_id != waypoints.front().header.frame_id ||
            !valid_point(p.pose.position) || !valid_quaternion(p.pose.orientation))
            return error(ErrorCode::InvalidTarget, "invalid/mixed-frame Cartesian waypoint");
    auto status = impl_->begin();
    if (!status) return status;
    struct Release {
        Impl& owner;
        ~Release() { owner.release(); }
    } release{*impl_};
    if (!impl_->groups.count(group))
        return error(ErrorCode::GroupNotAdded, "add Cartesian arm first");
    try {
        planning_scene::PlanningScenePtr scene;
        moveit::core::RobotStatePtr start;
        status = impl_->scene(scene, start, true);
        if (!status) return status;
        Eigen::Isometry3d conversion;
        status = impl_->frame(waypoints.front().header.frame_id, *start, conversion);
        if (!status) return status;
        EigenSTL::vector_Isometry3d transformed;
        for (const auto& pose : waypoints) transformed.push_back(conversion * transform(pose.pose));
        const auto side = static_cast<unsigned>(group);
        const auto* jmg = impl_->model->getJointModelGroup(kGroups[side]);
        moveit::core::JumpThreshold jump;
        jump.factor = impl_->relative_jump;
        jump.revolute = impl_->revolute_jump;
        jump.prismatic = impl_->prismatic_jump;
        std::vector<moveit::core::RobotStatePtr> states;
        auto state = *start;
        const auto cartesian_deadline = impl_->after(impl_->planning_time);
        std::string invalid_sample;
        const auto valid = [scene, constraints = impl_->constraints, before = *start,
                            cartesian_deadline, &invalid_sample](
                               moveit::core::RobotState* candidate,
                               const moveit::core::JointModelGroup* selected,
                               const double* values) {
            if (Clock::now() >= cartesian_deadline) return false;
            candidate->setJointGroupPositions(selected, values);
            candidate->update();
            for (const auto& name :
                 before.getRobotModel()->getJointModelGroup("dual_arm_spine")->getVariableNames())
                if (!selected->hasJointModel(name) && std::abs(
                                                          candidate->getVariablePosition(name) -
                                                          before.getVariablePosition(name)) > 1e-12)
                    return false;
            if (!candidate->satisfiesBounds()) {
                invalid_sample = "joint bounds";
                return false;
            }
            if (!scene->isStateValid(*candidate, constraints, "")) {
                collision_detection::CollisionRequest request;
                collision_detection::CollisionResult collisions;
                request.contacts = true;
                request.max_contacts = 8;
                scene->checkCollision(request, collisions, *candidate);
                invalid_sample = "collision or constraints";
                for (const auto& contact : collisions.contacts)
                    invalid_sample += "; " + contact.first.first + " / " + contact.first.second;
                return false;
            }
            return true;
        };
        auto previous_pose = start->getGlobalLinkTransform(kTips[side]);
        for (const auto& waypoint : transformed) {
            if ((waypoint.translation() - previous_pose.translation()).norm() / step > 1000000)
                return error(
                    ErrorCode::PlanningTimeout,
                    "Cartesian sample count exceeds bounded computation budget");
            previous_pose = waypoint;
        }
        const auto began = Clock::now();
        const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
            &state, jmg, states, impl_->model->getLinkModel(kTips[side]), transformed, true,
            moveit::core::MaxEEFStep(step), jump, valid);
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            if (impl_->planning_canceled) return error(ErrorCode::Canceled, "Cartesian stopped");
        }
        if (Clock::now() >= cartesian_deadline)
            return error(ErrorCode::PlanningTimeout, "Cartesian computation deadline");
        if (!std::isfinite(fraction) || states.empty())
            return error(ErrorCode::PathPlanningFailed, "Cartesian computation failed");
        if (fraction < 1.)
            RCLCPP_WARN(
                impl_->node->get_logger(), "Partial Cartesian path %.4f: %s", fraction,
                invalid_sample.empty() ? "IK or jump limit" : invalid_sample.c_str());
        robot_trajectory::RobotTrajectory path(impl_->model, kGroups[side]);
        for (const auto& point : states) path.addSuffixWayPoint(*point, 0);
        trajectory_processing::TimeOptimalTrajectoryGeneration timing(.001, .1, 1e-6);
        if (!timing.computeTimeStamps(path, impl_->velocity, impl_->acceleration))
            return error(ErrorCode::PathPlanningFailed, "Cartesian time parameterization failed");
        // TOTG resamples geometry; check its output again rather than trusting IK alone.
        for (std::size_t i = 0; i < path.getWayPointCount(); ++i)
            if (!path.getWayPoint(i).satisfiesBounds() ||
                !scene->isStateValid(path.getWayPoint(i), impl_->constraints, ""))
                return error(ErrorCode::PathPlanningFailed, "timed Cartesian waypoint invalid");
        Plan candidate;
        candidate.groups_ = {group};
        candidate.valid_ = true;
        moveit::core::robotStateToRobotStateMsg(*start, candidate.start_state_);
        path.getRobotTrajectoryMsg(candidate.trajectory_);
        candidate.planning_time_ = std::chrono::duration<double>(Clock::now() - began).count();
        double duration = 0;
        status = impl_->validate(candidate, *scene, *start, duration);
        if (!status) return status;
        candidate.cartesian_fraction_ = fraction;
        output.plan = std::move(candidate);
        output.fraction = fraction;
        return {};
    } catch (const std::exception& e) {
        return error(ErrorCode::InternalError, e.what());
    }
}
}  // namespace mfr3duo_moveit
