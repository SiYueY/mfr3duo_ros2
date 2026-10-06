#include "mfr3duo_robot/robot.hpp"
#include <atomic>
#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mfr3duo_control/control.hpp>
#include <mfr3duo_moveit/move_group.hpp>
#include <mfr3duo_moveit/planning_scene_interface.hpp>
#include <mfr3duo_nav/navigator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include "grasp_observer.hpp"
#include "grasp_recovery.hpp"
#include "profile_path.hpp"
#include <mfr3duo_msgs/srv/scene_joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
using namespace std::chrono_literals;
namespace mfr3duo_robot {
namespace {
using Clock = std::chrono::steady_clock;
using Group = mfr3duo_moveit::RobotGroup;
bool valid_timeout(std::chrono::milliseconds timeout) {
    // Reject chrono values whose conversion/addition would overflow the steady deadline.
    return timeout.count() > 0 && timeout < std::chrono::duration_cast<std::chrono::milliseconds>(
                                                Clock::time_point::max() - Clock::now());
}
TaskResult success() { return {TaskState::Succeeded, TaskError::None, {}}; }
TaskResult failure(TaskError error, std::string message) {
    return {
        error == TaskError::Canceled ? TaskState::Canceled : TaskState::Failed, error,
        std::move(message)};
}
Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& p) {
    auto t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    t.linear() =
        Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
            .toRotationMatrix();
    return t;
}
geometry_msgs::msg::Pose message(const Eigen::Isometry3d& t) {
    geometry_msgs::msg::Pose p;
    p.position.x = t.translation().x();
    p.position.y = t.translation().y();
    p.position.z = t.translation().z();
    Eigen::Quaterniond q(t.linear());
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
}
bool valid_pose(const geometry_msgs::msg::PoseStamped& p) {
    const auto& v = p.pose.position;
    const auto& q = p.pose.orientation;
    return !p.header.frame_id.empty() && p.header.stamp.sec >= 0 && std::isfinite(v.x) &&
           std::isfinite(v.y) && std::isfinite(v.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) &&
           std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) < 1e-5;
}
bool explicit_hand(Manipulator hand) {
    return hand == Manipulator::Left || hand == Manipulator::Right;
}
Group arm(Manipulator hand) { return hand == Manipulator::Left ? Group::LeftArm : Group::RightArm; }
mfr3duo_control::Gripper gripper(Manipulator hand) {
    return hand == Manipulator::Left ? mfr3duo_control::Gripper::Left
                                     : mfr3duo_control::Gripper::Right;
}
std::string tcp(Manipulator hand) {
    return hand == Manipulator::Left ? "left_fr3v2_1_hand_tcp" : "right_fr3v2_1_hand_tcp";
}
struct StepFailure : std::runtime_error {
    TaskError error;
    StepFailure(TaskError code, std::string message)
    : std::runtime_error(std::move(message)), error(code) {}
};
}  // namespace
struct TaskHandle::Impl {
    mutable std::mutex mutex;
    std::condition_variable changed;
    std::optional<TaskResult> result;
    std::atomic<bool> dispatched{false}, cancel_requested{false}, timed_out{false};
    Clock::time_point deadline, step_deadline;
    std::chrono::milliseconds terminal_timeout{4000};
    std::function<TaskResult()> cancel;
};
struct Robot::Impl {
    struct Engine : std::enable_shared_from_this<Engine> {
        enum class Stage { None, Move, Nav, LeftGripper, RightGripper, Scene };
        rclcpp::Node::SharedPtr node;
        mfr3duo_control::Control control;
        mfr3duo_moveit::MoveGroup move;
        mfr3duo_moveit::PlanningSceneInterface scene;
        mfr3duo_nav::Navigator navigator;
        std::unique_ptr<GraspObserver> observer;
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener;
        rclcpp::CallbackGroup::SharedPtr worker_group, deadline_group, health_group;
        rclcpp::TimerBase::SharedPtr worker, monitor, health_timer;
        std::atomic<bool> observer_healthy{false};
        std::atomic<long long> observer_seen{0};
        mutable std::mutex mutex;
        std::mutex initialize_mutex;
        std::mutex cancellation_mutex;
        RobotState state{RobotState::Uninitialized};
        bool live{false}, closing{false}, initializing{false};
        std::shared_ptr<TaskHandle::Impl> current;
        std::atomic<Stage> stage{Stage::None};
        std::chrono::milliseconds task_timeout, terminal_timeout;
        YAML::Node profile, posture, environment;
        std::vector<std::string> object_ids;
        std::string holding_id, recovery_id;
        Manipulator holding_hand{Manipulator::Auto}, recovery_hand{Manipulator::Auto};
        Eigen::Isometry3d held_relative{Eigen::Isometry3d::Identity()};
        moveit_msgs::msg::CollisionObject holding_geometry, recovery_geometry;
        bool grasp_stage{false}, relative_known{false};
        Eigen::Isometry3d return_pose{Eigen::Isometry3d::Identity()};
        bool pregrasp_spine_first{false};
        double pregrasp, place_clearance, lift, effort, velocity, acceleration, grasp_height,
            grasp_offset, open_width;
        double transit_velocity, transit_acceleration;
        using SceneCommand = mfr3duo_msgs::srv::SceneJointCommand;
        struct SceneSample {
            double position, velocity;
            Clock::time_point received;
        };
        struct SceneCache {
            std::mutex mutex;
            std::map<std::string, SceneSample> samples;
        };
        std::shared_ptr<SceneCache> scene_cache = std::make_shared<SceneCache>();
        std::mutex scene_command_mutex;
        rclcpp::Client<SceneCommand>::SharedPtr scene_command;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr scene_subscription;
        std::map<std::string, std::pair<double, double>> scene_limits;
        std::string active_scene_joint;
        explicit Engine(const rclcpp::Node::SharedPtr& supplied)
        : node(supplied),
          control(node),
          move(node),
          scene(node),
          navigator(node),
          observer(std::make_unique<SimulationGraspObserver>(node)),
          buffer(node->get_clock()),
          listener(buffer, node, false) {
            buffer.setUsingDedicatedThread(true);  // External executor provides TF callbacks.
            profile = YAML::LoadFile(profile_path(
                node, "grasp_profile_path",
                ament_index_cpp::get_package_share_directory("mfr3duo_robot") +
                    "/config/grasp.yaml"));
            posture = YAML::LoadFile(profile_path(
                node, "navigation_posture_path",
                ament_index_cpp::get_package_share_directory("mfr3duo_nav") +
                    "/config/navigation_posture.yaml"));
            const auto environment_path = profile_path(node, "environment_scene_path", "");
            if (!environment_path.empty()) environment = YAML::LoadFile(environment_path);
            const auto interactions_path = profile_path(node, "scene_interactions_path", "");
            if (!interactions_path.empty()) {
                const auto interactions = YAML::LoadFile(interactions_path);
                for (const auto& entry : interactions["joints"])
                    scene_limits.emplace(
                        entry["name"].as<std::string>(),
                        std::make_pair(entry["lower"].as<double>(), entry["upper"].as<double>()));
                scene_command =
                    node->create_client<SceneCommand>("/simulation/scene/joint_command");
                scene_subscription = node->create_subscription<sensor_msgs::msg::JointState>(
                    "/simulation/scene/joint_states", 10,
                    [cache = scene_cache, clock = node->get_clock(),
                     names = scene_limits](sensor_msgs::msg::JointState::ConstSharedPtr message) {
                        if (message->name.size() > 64 ||
                            message->name.size() != message->position.size() ||
                            message->name.size() != message->velocity.size() ||
                            message->header.stamp.sec < 0 ||
                            message->header.stamp.nanosec >= 1000000000U)
                            return;
                        const auto age =
                            (clock->now() - rclcpp::Time(message->header.stamp)).seconds();
                        if (age < -.1 || age > .3) return;
                        std::lock_guard<std::mutex> lock(cache->mutex);
                        for (std::size_t i = 0;
                             i < message->name.size() && i < message->position.size() &&
                             i < message->velocity.size();
                             ++i)
                            if (names.count(message->name[i]) &&
                                std::isfinite(message->position[i]) &&
                                std::isfinite(message->velocity[i]))
                                cache->samples[message->name[i]] = {
                                    message->position[i], message->velocity[i], Clock::now()};
                    });
            }
            const auto seconds = [&](const char* key, double fallback) {
                if (!node->has_parameter(key)) node->declare_parameter(key, fallback);
                const double value = node->get_parameter(key).as_double();
                if (!std::isfinite(value) || value < .001 || value > 3600)
                    throw std::invalid_argument(std::string("invalid parameter ") + key);
                return std::chrono::milliseconds(static_cast<long long>(value * 1000));
            };
            task_timeout = seconds("execution.task_timeout", 180.);
            terminal_timeout = seconds("robot.terminal_timeout", 4.);
            const auto setting = [&](const char* key, double fallback) {
                const std::string name = std::string("robot.") + key;
                if (!node->has_parameter(name)) node->declare_parameter(name, fallback);
                const double value = node->get_parameter(name).as_double();
                if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(name);
                return value;
            };
            pregrasp =
                setting("pregrasp_distance", profile["fixture"]["pregrasp_distance"].as<double>());
            place_clearance = setting(
                "place_clearance", profile["fixture"]["place_clearance"]
                                       ? profile["fixture"]["place_clearance"].as<double>()
                                       : pregrasp);
            lift = setting("lift_distance", profile["fixture"]["lift_distance"].as<double>());
            effort = setting(
                "acquisition_effort", profile["fixture"]["acquisition_effort"].as<double>());
            velocity =
                setting("velocity_scaling", profile["fixture"]["velocity_scaling"].as<double>());
            acceleration = setting(
                "acceleration_scaling", profile["fixture"]["acceleration_scaling"].as<double>());
            transit_velocity = setting("transit_velocity_scaling", .1);
            transit_acceleration = setting("transit_acceleration_scaling", .1);
            open_width = setting("gripper_open_width", .08);
            if (open_width < .075 || open_width > .08)
                throw std::invalid_argument("gripper_open_width must be in [.075, .08]");
            grasp_height = setting("grasp_spine_height", .2);
            if (!node->has_parameter("robot.pregrasp_spine_first"))
                node->declare_parameter("robot.pregrasp_spine_first", false);
            pregrasp_spine_first = node->get_parameter("robot.pregrasp_spine_first").as_bool();
            grasp_offset = setting("grasp_tcp_offset", .01);
            if (grasp_offset > .025)
                throw std::invalid_argument("grasp_tcp_offset exceeds configured box half height");
            for (const auto& entry : profile["observation"]["objects"])
                object_ids.push_back(entry.first.as<std::string>());
            if (object_ids.empty() || effort > 100 || velocity > 1 || acceleration > 1 ||
                grasp_height > .85 || transit_velocity > 1 || transit_acceleration > 1)
                throw std::invalid_argument("invalid Robot grasp profile");
            worker_group =
                node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            deadline_group =
                node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            health_group =
                node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            node->get_node_base_interface()->get_context()->add_on_shutdown_callback(
                [a = worker_group, b = deadline_group, c = health_group]() mutable {
                    a.reset();
                    b.reset();
                    c.reset();
                });
        }
        bool ready() const {
            const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 Clock::now().time_since_epoch())
                                 .count() -
                             observer_seen.load();
            return observer_healthy && age >= 0 && age < 1000 && control.is_ready() &&
                   move.is_ready() && navigator.is_ready();
        }
        void update_health() {
            std::string held_id;
            Manipulator hand = Manipulator::Auto;
            Eigen::Isometry3d reference = Eigen::Isometry3d::Identity();
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (initializing || closing || state == RobotState::Uninitialized) return;
                if (!live) {
                    held_id = holding_id;
                    hand = holding_hand;
                    reference = held_relative;
                }
            }
            bool healthy = true;
            for (const auto& id : object_ids) {
                auto obs = observer->observe(id, id == held_id ? hand : Manipulator::Left);
                healthy = healthy && obs.valid && obs.object_visible;
                if (id == held_id && obs.valid && obs.object_visible) {
                    const auto relative =
                        transform(obs.tool_pose.pose).inverse() * transform(obs.object_pose.pose);
                    // Pick validates its lift with the tighter 5 mm / 0.05 rad bounds.
                    // Once idle, compliant grippers can settle slightly while still holding
                    // the object.  Keep a bounded, independent holding envelope instead of
                    // turning that known physical settling into a recovery fault.
                    healthy = healthy && grasp_verified && width_matches(id, hand) &&
                              (relative.translation() - reference.translation()).norm() <= .015 &&
                              Eigen::AngleAxisd(relative.linear() * reference.linear().transpose())
                                      .angle() <= .10;
                }
            }
            observer_healthy = healthy;
            observer_seen = std::chrono::duration_cast<std::chrono::milliseconds>(
                                Clock::now().time_since_epoch())
                                .count();
            if (!healthy && !held_id.empty()) {
                std::lock_guard<std::mutex> lock(mutex);
                if (!live && holding_id == held_id) state = RobotState::Error;
            }
        }
        bool known(const std::string& id) const {
            return std::find(object_ids.begin(), object_ids.end(), id) != object_ids.end();
        }
        bool validate(const RobotTask& task, unsigned depth = 0) const {
            if (depth > 32 || (task.timeout() && !valid_timeout(*task.timeout()))) return false;
            if (const auto* p = dynamic_cast<const SceneJointTask*>(&task)) {
                const auto found = scene_limits.find(p->joint_name());
                return found != scene_limits.end() && std::isfinite(p->position()) &&
                       p->position() >= found->second.first &&
                       p->position() <= found->second.second;
            }
            if (const auto* p = dynamic_cast<const NavigateTask*>(&task))
                return valid_pose(p->target_pose()) &&
                       p->target_pose().header.frame_id ==
                           node->get_parameter("navigator.map_frame").as_string();
            if (const auto* p = dynamic_cast<const PickTask*>(&task))
                return known(p->object_id()) &&
                       (p->manipulator() == Manipulator::Auto || explicit_hand(p->manipulator())) &&
                       (!p->grasp_pose() || valid_pose(*p->grasp_pose()));
            if (const auto* p = dynamic_cast<const PlaceTask*>(&task))
                return known(p->object_id()) &&
                       (p->manipulator() == Manipulator::Auto || explicit_hand(p->manipulator())) &&
                       p->place_pose() && valid_pose(*p->place_pose());
            if (const auto* p = dynamic_cast<const TaskSequence*>(&task)) {
                if (p->tasks().empty()) return false;
                for (const auto& item : p->tasks())
                    if (!item || !validate(*item, depth + 1)) return false;
                return true;
            }
            return false;
        }
        void check(const std::shared_ptr<TaskHandle::Impl>& op) {
            Clock::time_point deadline;
            {
                std::lock_guard<std::mutex> lock(op->mutex);
                deadline = op->step_deadline;
            }
            if (Clock::now() >= deadline) {
                op->timed_out = true;
                op->cancel_requested = true;
            }
            if (op->cancel_requested)
                throw StepFailure(
                    op->timed_out ? TaskError::Timeout : TaskError::Canceled,
                    op->timed_out ? "Task deadline expired" : "Task cancellation requested");
        }
        template <class R>
        void require(const R& result, TaskError code, const std::string& label) {
            if (!result) {
                if (result.code == decltype(result.code)::CancelFailed ||
                    result.code == decltype(result.code)::PreviousOperationNotTerminated)
                    code = TaskError::RecoveryRequired;
                throw StepFailure(code, label + ": " + result.message);
            }
        }
        GraspObservation observe(const std::string& id, Manipulator hand) {
            auto obs = observer->observe(id, hand);
            if (!obs.valid || !obs.object_visible)
                throw StepFailure(TaskError::RecoveryRequired, "observation: " + obs.diagnostic);
            return obs;
        }
        geometry_msgs::msg::PoseStamped relative(
            const GraspObservation& obs, const Eigen::Isometry3d& world, Manipulator hand) {
            geometry_msgs::msg::PoseStamped target;
            target.header.frame_id = tcp(hand);
            target.pose = message(transform(obs.tool_pose.pose).inverse() * world);
            return target;
        }
        void refresh_environment(Manipulator hand) {
            if (!environment || !environment["boxes"]) return;
            auto obs = observe(object_ids.front(), hand);
            std::vector<moveit_msgs::msg::CollisionObject> objects;
            for (const auto& entry : environment["boxes"]) {
                const auto position = entry["position"].as<std::vector<double>>();
                const auto dimensions = entry["dimensions"].as<std::vector<double>>();
                if (position.size() != 3 || dimensions.size() != 3)
                    throw StepFailure(TaskError::InvalidTask, "invalid environment box");
                auto world = Eigen::Isometry3d::Identity();
                world.translation() = Eigen::Vector3d(position[0], position[1], position[2]);
                if (entry["joint"]) {
                    const double value = scene_sample(entry["joint"].as<std::string>()).position;
                    const auto axis_values = entry["axis"].as<std::vector<double>>();
                    const auto pivot_values = entry["pivot"].as<std::vector<double>>();
                    if (axis_values.size() != 3 || pivot_values.size() != 3)
                        throw StepFailure(TaskError::InvalidTask, "invalid fixture joint geometry");
                    const Eigen::Vector3d axis(axis_values[0], axis_values[1], axis_values[2]);
                    const Eigen::Vector3d pivot(pivot_values[0], pivot_values[1], pivot_values[2]);
                    if (entry["joint_type"].as<std::string>() == "slide")
                        world.translation() += axis * value;
                    else {
                        world.linear() =
                            Eigen::AngleAxisd(value, axis.normalized()).toRotationMatrix();
                        world.translation() =
                            pivot + world.linear() * (world.translation() - pivot);
                    }
                }
                const auto pose = relative(obs, world, hand);
                moveit_msgs::msg::CollisionObject object;
                object.id = entry["id"].as<std::string>();
                object.header = pose.header;
                object.operation = object.ADD;
                shape_msgs::msg::SolidPrimitive box;
                box.type = box.BOX;
                box.dimensions.assign(dimensions.begin(), dimensions.end());
                object.primitives = {box};
                object.primitive_poses = {pose.pose};
                objects.push_back(std::move(object));
            }
            if (!objects.empty())
                require(
                    scene.add_collision_objects(objects), TaskError::PlanningFailed,
                    "fresh kitchen collision geometry");
        }
        moveit_msgs::msg::CollisionObject support_geometry(
            const GraspObservation& obs, Manipulator hand) {
            auto table = Eigen::Isometry3d::Identity();
            const auto position = profile["fixture"]["table_position"].as<std::vector<double>>();
            table.translation() = Eigen::Vector3d(position.at(0), position.at(1), position.at(2));
            moveit_msgs::msg::CollisionObject support;
            support.id = "grasp_table";
            support.operation = support.ADD;
            const auto pose = relative(obs, table, hand);
            support.header = pose.header;
            shape_msgs::msg::SolidPrimitive box;
            box.type = box.BOX;
            const auto dimensions =
                profile["fixture"]["table_dimensions"].as<std::vector<double>>();
            box.dimensions.assign(dimensions.begin(), dimensions.end());
            // MuJoCo's compliant resting contact penetrates about 21 micrometres.
            // Keep the measured table center and a fixed 0.1 mm planning surface skin;
            // this does not alter physical geometry or allow object/support collisions.
            box.dimensions.at(2) -= .0002;
            support.primitives = {box};
            support.primitive_poses = {pose.pose};
            return support;
        }
        Eigen::Isometry3d world_pose(
            const geometry_msgs::msg::PoseStamped& target, const GraspObservation& obs) {
            if (target.header.frame_id == obs.object_pose.header.frame_id)
                return transform(target.pose);
            try {
                const auto tf = buffer.lookupTransform(
                    obs.object_pose.header.frame_id, target.header.frame_id, tf2::TimePointZero);
                if (tf.header.stamp.sec != 0 || tf.header.stamp.nanosec != 0) {
                    const double age = (node->now() - rclcpp::Time(tf.header.stamp)).seconds();
                    if (age < -.1 || age > .3)
                        throw StepFailure(TaskError::InvalidTask, "target transform is stale");
                }
                geometry_msgs::msg::Pose p;
                p.position.x = tf.transform.translation.x;
                p.position.y = tf.transform.translation.y;
                p.position.z = tf.transform.translation.z;
                p.orientation = tf.transform.rotation;
                return transform(p) * transform(target.pose);
            } catch (const tf2::TransformException& e) {
                throw StepFailure(TaskError::InvalidTask, std::string("target frame: ") + e.what());
            }
        }
        moveit_msgs::msg::CollisionObject geometry(
            const std::string& id, const GraspObservation& obs, Manipulator hand) {
            moveit_msgs::msg::CollisionObject out;
            out.id = id;
            out.operation = out.ADD;
            out.header = relative(obs, transform(obs.object_pose.pose), hand).header;
            shape_msgs::msg::SolidPrimitive box;
            box.type = box.BOX;
            const auto dimensions =
                profile["observation"]["objects"][id]["dimensions"].as<std::vector<double>>();
            box.dimensions.assign(dimensions.begin(), dimensions.end());
            out.primitives = {box};
            out.primitive_poses = {relative(obs, transform(obs.object_pose.pose), hand).pose};
            return out;
        }
        void reset_builder() {
            require(
                move.clear_path_constraints(), TaskError::PlanningFailed, "clear path constraints");
            require(move.clear_targets(), TaskError::PlanningFailed, "clear targets");
            require(move.clear_groups(), TaskError::PlanningFailed, "clear groups");
            require(
                move.set_start_state_to_current_state(), TaskError::PlanningFailed,
                "current start state");
        }
        std::atomic<bool> grasp_verified{false};
        double grasp_width(const std::string& id) const {
            return profile["observation"]["objects"][id]["grasp_width"].as<double>();
        }
        bool width_matches(const std::string& id, Manipulator hand) const {
            const auto width = control.get_gripper_width(gripper(hand));
            const auto object = profile["observation"]["objects"][id];
            return width && width.value > grasp_width(id) - object["epsilon_inner"].as<double>() &&
                   width.value < grasp_width(id) + object["epsilon_outer"].as<double>();
        }
        void open_close(
            const std::shared_ptr<TaskHandle::Impl>& op, Manipulator hand, double position) {
            check(op);
            stage = hand == Manipulator::Left ? Stage::LeftGripper : Stage::RightGripper;
            auto result = control.move_gripper(
                gripper(hand), 2 * position, profile["fixture"]["gripper_speed"].as<double>());
            require(result, TaskError::GraspFailed, "gripper action");
            stage = Stage::None;
            check(op);
        }
        void cartesian(
            const std::shared_ptr<TaskHandle::Impl>& op, Manipulator hand, const std::string& id,
            const Eigen::Isometry3d& world, const char* label) {
            check(op);
            stage = Stage::Move;
            refresh_environment(hand);
            auto obs = observe(id, hand);
            // The planning model has a fixed base. Arm reactions can move the
            // simulated mobile base, so reproject support and world objects too.
            std::vector<moveit_msgs::msg::CollisionObject> current{support_geometry(obs, hand)};
            if (!relative_known) current.push_back(geometry(id, obs, hand));
            require(
                scene.add_collision_objects(current), TaskError::PlanningFailed,
                "fresh Cartesian scene");
            mfr3duo_moveit::CartesianPath path;
            require(move.clear_targets(), TaskError::PlanningFailed, "clear Cartesian targets");
            require(
                move.set_start_state_to_current_state(), TaskError::PlanningFailed,
                "Cartesian current start");
            require(
                move.compute_cartesian_path(arm(hand), {relative(obs, world, hand)}, .005, path),
                TaskError::PlanningFailed, label);
            if (path.fraction != 1)
                throw StepFailure(
                    TaskError::PlanningFailed, std::string(label) +
                                                   ": partial Cartesian path forbidden, fraction=" +
                                                   std::to_string(path.fraction));
            check(op);
            require(move.execute(path.plan), TaskError::ExecutionFailed, label);
            stage = Stage::None;
            check(op);
        }
        bool stop_owned() {
            switch (stage.load()) {
                case Stage::Scene: {
                    std::string name;
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        name = active_scene_joint;
                    }
                    if (name.empty()) return true;
                    if (!send_scene_command(name, 0., true)) return false;
                    const auto deadline = Clock::now() + 1s;
                    auto stable = Clock::now();
                    try {
                        double previous = scene_sample(name).position;
                        while (Clock::now() < deadline) {
                            const auto sample = scene_sample(name);
                            if (std::abs(sample.velocity) > .015 ||
                                std::abs(sample.position - previous) > .003)
                                stable = Clock::now();
                            previous = sample.position;
                            if (Clock::now() - stable >= 200ms) return true;
                            std::this_thread::sleep_for(20ms);
                        }
                    } catch (const StepFailure&) {
                        return false;
                    }
                    return false;
                }
                case Stage::Move: {
                    auto r = move.stop();
                    return r.code == mfr3duo_moveit::ErrorCode::Success ||
                           r.code == mfr3duo_moveit::ErrorCode::Canceled;
                }
                case Stage::Nav: {
                    auto r = navigator.cancel();
                    return r.code == mfr3duo_nav::ErrorCode::Success ||
                           r.code == mfr3duo_nav::ErrorCode::Canceled;
                }
                case Stage::LeftGripper:
                case Stage::RightGripper: {
                    auto r = control.stop_gripper(
                        stage == Stage::LeftGripper ? mfr3duo_control::Gripper::Left
                                                    : mfr3duo_control::Gripper::Right);
                    return r.code == mfr3duo_control::ErrorCode::Success ||
                           r.code == mfr3duo_control::ErrorCode::Canceled;
                }
                case Stage::None:
                    return true;
            }
            return false;
        }
        void navigate(const NavigateTask& task, const std::shared_ptr<TaskHandle::Impl>& op) {
            if (!holding_id.empty())
                throw StepFailure(
                    TaskError::InvalidTask,
                    "carried-object navigation footprint is not validated in V1");
            check(op);
            refresh_environment(Manipulator::Left);
            stage = Stage::Move;
            reset_builder();
            require(
                move.add_groups({Group::LeftArm, Group::RightArm, Group::Spine}),
                TaskError::PlanningFailed, "transport groups");
            require(
                move.add_joint_position_target(
                    Group::LeftArm, posture["left_arm"].as<std::vector<double>>()),
                TaskError::PlanningFailed, "left transport target");
            require(
                move.add_joint_position_target(
                    Group::RightArm, posture["right_arm"].as<std::vector<double>>()),
                TaskError::PlanningFailed, "right transport target");
            require(
                move.add_joint_position_target(Group::Spine, posture["spine_height"].as<double>()),
                TaskError::PlanningFailed, "spine transport target");
            require(
                move.set_max_velocity_scaling_factor(transit_velocity), TaskError::PlanningFailed,
                "transport velocity");
            require(
                move.set_max_acceleration_scaling_factor(transit_acceleration),
                TaskError::PlanningFailed, "transport acceleration");
            require(move.move(), TaskError::ExecutionFailed, "transport posture");
            stage = Stage::None;
            const auto left = control.get_arm_joint_positions(mfr3duo_control::Arm::Left);
            const auto right = control.get_arm_joint_positions(mfr3duo_control::Arm::Right);
            const auto spine = control.get_spine_position();
            if (!left || !right || !spine)
                throw StepFailure(TaskError::RobotNotReady, "fresh transport measurements missing");
            const auto near = [&](const std::vector<double>& actual, const char* key) {
                const auto target = posture[key].as<std::vector<double>>();
                if (actual.size() != target.size()) return false;
                for (std::size_t i = 0; i < actual.size(); ++i)
                    if (std::abs(actual[i] - target[i]) > posture["joint_tolerance"].as<double>())
                        return false;
                return true;
            };
            if (!near(left.value, "left_arm") || !near(right.value, "right_arm") ||
                std::abs(spine.value - posture["spine_height"].as<double>()) >
                    posture["spine_tolerance"].as<double>())
                throw StepFailure(
                    TaskError::ExecutionFailed, "measured transport posture outside tolerance");
            open_close(op, Manipulator::Left, posture["gripper_width"].as<double>() * .5);
            open_close(op, Manipulator::Right, posture["gripper_width"].as<double>() * .5);
            check(op);
            stage = Stage::Nav;
            require(
                navigator.navigate_to(task.target_pose()), TaskError::NavigationFailed,
                "navigation");
            stage = Stage::None;
            check(op);
        }
        void settle_spine(const std::shared_ptr<TaskHandle::Impl>& op) {
            auto settle = Clock::now() + 5s;
            double previous = 0;
            auto stable = Clock::now();
            bool sampled = false;
            while (Clock::now() < settle) {
                check(op);
                auto p = control.get_spine_position();
                if (!p)
                    throw StepFailure(
                        TaskError::RobotNotReady, "spine state stale before Cartesian stage");
                if (!sampled || std::abs(p.value - previous) > .0005) stable = Clock::now();
                if (!sampled || std::abs(p.value - previous) > .0005) previous = p.value;
                sampled = true;
                if (Clock::now() - stable >= 500ms) break;
                std::this_thread::sleep_for(20ms);
            }
            if (Clock::now() - stable < 500ms)
                throw StepFailure(TaskError::ExecutionFailed, "inactive spine did not settle");
        }
        bool send_scene_command(
            const std::string& name, double target, bool hold = false,
            const std::shared_ptr<TaskHandle::Impl>& op = {}) {
            // A cancel hold follows any in-flight target, and no later target can
            // overwrite it after cancellation has been requested.
            std::lock_guard<std::mutex> serial(scene_command_mutex);
            if (op) check(op);
            if (!scene_command || !scene_command->service_is_ready()) return false;
            auto request = std::make_shared<SceneCommand::Request>();
            request->joint_name = name;
            request->position = target;
            request->hold = hold;
            auto future = scene_command->async_send_request(request);
            if (future.wait_for(2s) != std::future_status::ready) {
                scene_command->remove_pending_request(future);
                return false;
            }
            return future.get()->success;
        }
        SceneSample scene_sample(const std::string& name) {
            std::lock_guard<std::mutex> lock(scene_cache->mutex);
            const auto found = scene_cache->samples.find(name);
            if (found == scene_cache->samples.end() ||
                Clock::now() - found->second.received > 300ms)
                throw StepFailure(TaskError::RobotNotReady, "stale scene joint measurement");
            return found->second;
        }
        void interact_scene(
            const SceneJointTask& task, const std::shared_ptr<TaskHandle::Impl>& op) {
            if (!holding_id.empty())
                throw StepFailure(
                    TaskError::InvalidTask, "scene interaction while carrying forbidden");
            auto sample = scene_sample(task.joint_name());
            {
                std::lock_guard<std::mutex> lock(mutex);
                active_scene_joint = task.joint_name();
            }
            stage = Stage::Scene;
            const auto limits = scene_limits.at(task.joint_name());
            double command = std::clamp(sample.position, limits.first, limits.second);
            auto previous = Clock::now();
            auto stable = previous;
            while (true) {
                check(op);
                const auto now = Clock::now();
                sample = scene_sample(task.joint_name());
                if (std::abs(sample.position - task.position()) > .015 ||
                    std::abs(sample.velocity) > .015)
                    stable = now;
                if (now - stable >= 500ms) break;
                const double increment =
                    .10 * std::chrono::duration<double>(now - previous).count();
                command += std::clamp(task.position() - command, -increment, increment);
                previous = now;
                if (!send_scene_command(task.joint_name(), command, false, op))
                    throw StepFailure(
                        TaskError::ExecutionFailed, "fixture actuator command unconfirmed");
                std::this_thread::sleep_for(20ms);
            }
            if (!send_scene_command(task.joint_name(), task.position(), false, op))
                throw StepFailure(TaskError::ExecutionFailed, "fixture final hold unconfirmed");
            stage = Stage::None;
            refresh_environment(Manipulator::Left);
        }
        void pick(const PickTask& task, const std::shared_ptr<TaskHandle::Impl>& op) {
            if (!holding_id.empty())
                throw StepFailure(TaskError::InvalidTask, "already holding an object");
            grasp_verified = false;
            const auto hand =
                task.manipulator() == Manipulator::Auto ? Manipulator::Left : task.manipulator();
            refresh_environment(hand);
            auto obs = observe(task.object_id(), hand);
            bool attached = false;
            require(
                scene.is_attached(task.object_id(), attached), TaskError::RecoveryRequired,
                "pick scene query");
            if (attached)
                throw StepFailure(
                    TaskError::RecoveryRequired, "unexpected attached object before Pick");
            recovery_id = task.object_id();
            recovery_hand = hand;
            recovery_geometry = geometry(task.object_id(), obs, hand);
            require(
                scene.add_collision_objects({support_geometry(obs, hand), recovery_geometry}),
                TaskError::RecoveryRequired, "fresh support and world object");
            open_close(op, hand, open_width * .5);
            auto grasp = task.grasp_pose() ? world_pose(*task.grasp_pose(), obs)
                                           : transform(obs.object_pose.pose);
            if (!task.grasp_pose()) {
                grasp.translation().z() += grasp_offset;
                grasp.linear() = Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitX())
                                     .toRotationMatrix();
            }
            return_pose = grasp;
            return_pose.translation().z() += pregrasp;
            check(op);
            stage = Stage::Move;
            reset_builder();
            if (pregrasp_spine_first) {
                require(
                    move.add_group(Group::Spine), TaskError::PlanningFailed,
                    "pregrasp spine group");
                require(
                    move.add_joint_position_target(Group::Spine, grasp_height),
                    TaskError::PlanningFailed, "pregrasp spine target");
                require(
                    move.set_max_velocity_scaling_factor(transit_velocity),
                    TaskError::PlanningFailed, "pregrasp spine velocity");
                require(
                    move.set_max_acceleration_scaling_factor(transit_acceleration),
                    TaskError::PlanningFailed, "pregrasp spine acceleration");
                require(move.move(), TaskError::ExecutionFailed, "pregrasp spine move");
                settle_spine(op);
                check(op);
                reset_builder();
            }
            require(move.add_group(arm(hand)), TaskError::PlanningFailed, "pregrasp arm group");
            if (!pregrasp_spine_first) {
                require(
                    move.add_group(Group::Spine), TaskError::PlanningFailed,
                    "pregrasp spine group");
                require(
                    move.add_joint_position_target(Group::Spine, grasp_height),
                    TaskError::PlanningFailed, "grasp spine height");
            }
            obs = observe(task.object_id(), hand);
            if (const auto limits = profile["fixture"]["pregrasp_joint_constraints"]) {
                moveit_msgs::msg::Constraints constraints;
                for (const auto& entry : limits) {
                    const auto values = entry.second.as<std::vector<double>>();
                    if (values.size() != 2)
                        throw StepFailure(
                            TaskError::InvalidTask, "invalid pregrasp joint constraint");
                    moveit_msgs::msg::JointConstraint joint;
                    joint.joint_name = std::string(hand == Manipulator::Left ? "left_" : "right_") +
                                       "fr3v2_1_" + entry.first.as<std::string>();
                    joint.position = values[0];
                    joint.tolerance_above = joint.tolerance_below = values[1];
                    joint.weight = 1.;
                    constraints.joint_constraints.push_back(joint);
                }
                require(
                    move.add_path_constraint(constraints), TaskError::PlanningFailed,
                    "counter pregrasp joint constraints");
            }
            require(
                move.add_pose_target(arm(hand), relative(obs, return_pose, hand)),
                TaskError::PlanningFailed, "pregrasp target");
            require(
                move.set_max_velocity_scaling_factor(transit_velocity), TaskError::PlanningFailed,
                "pregrasp transit velocity");
            require(
                move.set_max_acceleration_scaling_factor(transit_acceleration),
                TaskError::PlanningFailed, "pregrasp transit acceleration");
            require(move.move(), TaskError::ExecutionFailed, "pregrasp move");
            stage = Stage::None;
            settle_spine(op);
            require(
                move.set_max_velocity_scaling_factor(velocity), TaskError::PlanningFailed,
                "grasp velocity");
            require(
                move.set_max_acceleration_scaling_factor(acceleration), TaskError::PlanningFailed,
                "grasp acceleration");
            require(
                scene.set_grasp_contact_allowed(task.object_id(), arm(hand), true),
                TaskError::RecoveryRequired, "selected fingers");
            grasp_stage = true;
            cartesian(op, hand, task.object_id(), grasp, "approach");
            // Static load causes actual joint tracking offsets. Correct the measured TCP,
            // through collision-checked full paths, before establishing the held baseline.
            auto commanded = grasp;
            bool aligned = false;
            for (unsigned attempt = 0; attempt < 4; ++attempt) {
                check(op);
                obs = observe(task.object_id(), hand);
                const auto actual = transform(obs.tool_pose.pose);
                const double position_error = (actual.translation() - grasp.translation()).norm();
                const double angle_error =
                    Eigen::AngleAxisd(actual.linear() * grasp.linear().transpose()).angle();
                if (position_error <= .002 && angle_error <= .02) {
                    aligned = true;
                    break;
                }
                if (position_error > .02 || angle_error > .05)
                    throw StepFailure(
                        TaskError::ExecutionFailed,
                        "actual approach error exceeds correction envelope");
                commanded = grasp * actual.inverse() * commanded;
                cartesian(op, hand, task.object_id(), commanded, "measured grasp alignment");
            }
            if (!aligned)
                throw StepFailure(
                    TaskError::ExecutionFailed, "actual TCP grasp alignment did not converge");
            check(op);
            stage = hand == Manipulator::Left ? Stage::LeftGripper : Stage::RightGripper;
            const auto object = profile["observation"]["objects"][task.object_id()];
            require(
                control.grasp_gripper(
                    gripper(hand), grasp_width(task.object_id()),
                    profile["fixture"]["gripper_speed"].as<double>(), effort,
                    object["epsilon_inner"].as<double>(), object["epsilon_outer"].as<double>()),
                TaskError::GraspFailed, "grasp device action");
            stage = Stage::None;
            auto position = control.get_gripper_position(gripper(hand));
            if (!width_matches(task.object_id(), hand))
                throw StepFailure(
                    TaskError::GraspFailed,
                    "measured grasp opening outside expected width tolerance");
            obs = observe(task.object_id(), hand);
            // Device success alone does not mean the arm and closing load have settled.
            // Establish the pre-lift reference only after a stationary measured window.
            auto settled_tool = transform(obs.tool_pose.pose);
            auto settled_relative = settled_tool.inverse() * transform(obs.object_pose.pose);
            auto load_stable_since = Clock::now();
            const auto load_settle_deadline = Clock::now() + 3s;
            while (Clock::now() - load_stable_since < 500ms) {
                check(op);
                if (Clock::now() >= load_settle_deadline)
                    throw StepFailure(
                        TaskError::GraspFailed, "grasp load did not settle before lift");
                obs = observe(task.object_id(), hand);
                if (!width_matches(task.object_id(), hand))
                    throw StepFailure(
                        TaskError::GraspFailed, "grasp opening changed while settling");
                const auto tool = transform(obs.tool_pose.pose);
                const auto current_relative = tool.inverse() * transform(obs.object_pose.pose);
                if ((tool.translation() - settled_tool.translation()).norm() > .0005 ||
                    Eigen::AngleAxisd(tool.linear() * settled_tool.linear().transpose()).angle() >
                        .005 ||
                    (current_relative.translation() - settled_relative.translation()).norm() >
                        .0005) {
                    load_stable_since = Clock::now();
                    settled_tool = tool;
                    settled_relative = current_relative;
                }
                std::this_thread::sleep_for(20ms);
            }
            held_relative =
                transform(obs.tool_pose.pose).inverse() * transform(obs.object_pose.pose);
            relative_known = true;
            RCLCPP_INFO(
                node->get_logger(),
                "Measured grasp baseline: object=(%.6f,%.6f,%.6f) TCP=(%.6f,%.6f,%.6f) "
                "half_width=%.6f",
                obs.object_pose.pose.position.x, obs.object_pose.pose.position.y,
                obs.object_pose.pose.position.z, obs.tool_pose.pose.position.x,
                obs.tool_pose.pose.position.y, obs.tool_pose.pose.position.z, position.value);
            holding_geometry = geometry(task.object_id(), obs, hand);
            require(
                scene.add_collision_objects({support_geometry(obs, hand), holding_geometry}),
                TaskError::RecoveryRequired, "fresh attached geometry and support");
            require(
                scene.attach_object(task.object_id(), arm(hand)), TaskError::RecoveryRequired,
                "attach after measured device grasp");
            holding_id = task.object_id();
            holding_hand = hand;
            require(
                scene.set_grasp_contact_allowed(task.object_id(), arm(hand), false),
                TaskError::RecoveryRequired, "restore attached contact rules");
            grasp_stage = false;
            auto raised = transform(obs.tool_pose.pose);
            raised.translation().z() += lift;
            const double start_z = obs.object_pose.pose.position.z;
            cartesian(op, hand, task.object_id(), raised, "lift");
            const auto until = Clock::now() + 500ms;
            double maximum_drift = 0, maximum_angle = 0;
            while (Clock::now() < until) {
                check(op);
                obs = observe(task.object_id(), hand);
                const auto rel =
                    transform(obs.tool_pose.pose).inverse() * transform(obs.object_pose.pose);
                maximum_drift = std::max(
                    maximum_drift, (rel.translation() - held_relative.translation()).norm());
                maximum_angle = std::max(
                    maximum_angle,
                    Eigen::AngleAxisd(rel.linear() * held_relative.linear().transpose()).angle());
                if (!width_matches(task.object_id(), hand) ||
                    obs.object_pose.pose.position.z < start_z + lift * .8 ||
                    (rel.translation() - held_relative.translation()).norm() > .005 ||
                    Eigen::AngleAxisd(rel.linear() * held_relative.linear().transpose()).angle() >
                        .05)
                    throw StepFailure(
                        TaskError::GraspFailed,
                        "actual lift validation: rise=" +
                            std::to_string(obs.object_pose.pose.position.z - start_z) + " drift=" +
                            std::to_string(
                                (rel.translation() - held_relative.translation()).norm()) +
                            " delta_xyz=" +
                            std::to_string(
                                rel.translation().x() - held_relative.translation().x()) +
                            "," +
                            std::to_string(
                                rel.translation().y() - held_relative.translation().y()) +
                            "," +
                            std::to_string(
                                rel.translation().z() - held_relative.translation().z()) +
                            " angle=" +
                            std::to_string(
                                Eigen::AngleAxisd(rel.linear() * held_relative.linear().transpose())
                                    .angle()));
                std::this_thread::sleep_for(20ms);
            }
            // The arm has moved while carrying the object.  Keep the physical baseline
            // used by the idle health monitor and the subsequent PlaceTask in the same
            // settled, post-lift measurement frame.  Retaining the pre-lift transform
            // makes a successful Pick immediately look like a held-object drift when
            // the compliant simulated/physical gripper settles after the lift.
            held_relative =
                transform(obs.tool_pose.pose).inverse() * transform(obs.object_pose.pose);
            grasp_verified = true;
            RCLCPP_INFO(
                node->get_logger(),
                "Physical lift verified: rise=%.6f m relative_drift=%.6f m angle=%.6f rad",
                obs.object_pose.pose.position.z - start_z, maximum_drift, maximum_angle);
        }
        void place(const PlaceTask& task, const std::shared_ptr<TaskHandle::Impl>& op) {
            if (holding_id != task.object_id() || !explicit_hand(holding_hand) ||
                (task.manipulator() != Manipulator::Auto && task.manipulator() != holding_hand))
                throw StepFailure(
                    TaskError::InvalidTask, "Place requires the specified held object and hand");
            const auto hand = holding_hand;
            auto held = recover_grasp(
                node, scene, *observer, holding_id, hand, held_relative, holding_geometry, true,
                control, grasp_verified, grasp_width(holding_id),
                profile["observation"]["objects"][holding_id]["epsilon_inner"].as<double>(),
                profile["observation"]["objects"][holding_id]["epsilon_outer"].as<double>());
            if (held.holding != PhysicalHolding::Held || !held.scene_confirmed)
                throw StepFailure(
                    TaskError::RecoveryRequired, "Place holding confirmation: " + held.diagnostic);
            refresh_environment(hand);
            auto obs = observe(task.object_id(), hand);
            require(
                scene.add_collision_object(support_geometry(obs, hand)),
                TaskError::RecoveryRequired, "fresh Place support");
            const auto resting = world_pose(*task.place_pose(), obs);
            auto target = resting * held_relative.inverse();
            target.translation().z() += .02;
            auto preplace = target;
            preplace.translation().z() += place_clearance;
            check(op);
            stage = Stage::Move;
            reset_builder();
            require(move.add_group(arm(hand)), TaskError::PlanningFailed, "preplace group");
            require(
                move.add_pose_target(arm(hand), relative(obs, preplace, hand)),
                TaskError::PlanningFailed, "preplace target");
            require(
                move.set_max_velocity_scaling_factor(velocity), TaskError::PlanningFailed,
                "preplace carrying velocity");
            require(
                move.set_max_acceleration_scaling_factor(acceleration), TaskError::PlanningFailed,
                "preplace carrying acceleration");
            const auto measured_tool = transform(obs.tool_pose.pose);
            if ((preplace.translation() - measured_tool.translation()).norm() <= .1 &&
                Eigen::AngleAxisd(preplace.linear() * measured_tool.linear().transpose()).angle() <=
                    .05)
                cartesian(op, hand, task.object_id(), preplace, "preplace move");
            else
                require(move.move(), TaskError::ExecutionFailed, "preplace move");
            stage = Stage::None;
            require(
                move.set_max_velocity_scaling_factor(velocity), TaskError::PlanningFailed,
                "place velocity");
            require(
                move.set_max_acceleration_scaling_factor(acceleration), TaskError::PlanningFailed,
                "place acceleration");
            cartesian(op, hand, task.object_id(), target, "place approach");
            open_close(op, hand, open_width * .5);
            const auto until = Clock::now() + 500ms;
            while (Clock::now() < until) {
                check(op);
                obs = observe(task.object_id(), hand);
                const auto opening = control.get_gripper_width(gripper(hand));
                if (!opening || opening.value < .075)
                    throw StepFailure(
                        TaskError::RecoveryRequired, "release not physically confirmed");
                std::this_thread::sleep_for(20ms);
            }
            if ((transform(obs.object_pose.pose).translation() - resting.translation()).norm() >
                    .01 ||
                Eigen::AngleAxisd(
                    transform(obs.object_pose.pose).linear() * resting.linear().transpose())
                        .angle() > .05)
                throw StepFailure(
                    TaskError::GraspLost, "released object outside intended support pose");
            auto retreat = transform(obs.tool_pose.pose);
            retreat.translation().z() += place_clearance;
            cartesian(op, hand, task.object_id(), retreat, "place retreat");
            const auto settled = Clock::now() + 500ms;
            while (Clock::now() < settled) {
                check(op);
                obs = observe(task.object_id(), hand);
                const auto actual = transform(obs.object_pose.pose);
                if ((actual.translation() - resting.translation()).norm() > .01 ||
                    Eigen::AngleAxisd(actual.linear() * resting.linear().transpose()).angle() > .05)
                    throw StepFailure(
                        TaskError::GraspLost, "object moved outside support after tool retreat");
                std::this_thread::sleep_for(20ms);
            }
            auto recovered = recover_grasp(
                node, scene, *observer, holding_id, hand, held_relative, holding_geometry, true,
                control, grasp_verified, grasp_width(holding_id),
                profile["observation"]["objects"][holding_id]["epsilon_inner"].as<double>(),
                profile["observation"]["objects"][holding_id]["epsilon_outer"].as<double>());
            if (recovered.holding != PhysicalHolding::Released || !recovered.scene_confirmed)
                throw StepFailure(
                    TaskError::RecoveryRequired, "Place world restore: " + recovered.diagnostic);
            holding_id.clear();
            holding_hand = Manipulator::Auto;
            relative_known = false;
            grasp_verified = false;
        }
        Robot::PreflightResult preflight_pick(const PickTask& task) {
            Robot::PreflightResult result;
            std::string last_error;
            for (const auto hand : std::array<Manipulator, 2>{Manipulator::Left, Manipulator::Right}) {
                if (task.manipulator() != Manipulator::Auto && task.manipulator() != hand) continue;
                try {
                    refresh_environment(hand);
                    const auto obs = observe(task.object_id(), hand);
                    require(scene.add_collision_objects({support_geometry(obs, hand), geometry(task.object_id(), obs, hand)}),
                            TaskError::PlanningFailed, "preflight collision scene");
                    auto grasp = task.grasp_pose() ? world_pose(*task.grasp_pose(), obs) : transform(obs.object_pose.pose);
                    if (!task.grasp_pose()) {
                        grasp.translation().z() += grasp_offset;
                        grasp.linear() = Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitX()).toRotationMatrix();
                    }
                    grasp.translation().z() += pregrasp;
                    reset_builder();
                    require(move.add_group(arm(hand)), TaskError::PlanningFailed, "preflight arm group");
                    require(move.add_group(Group::Spine), TaskError::PlanningFailed, "preflight spine group");
                    require(move.add_joint_position_target(Group::Spine, grasp_height), TaskError::PlanningFailed, "preflight spine target");
                    if (const auto limits = profile["fixture"]["pregrasp_joint_constraints"]) {
                        moveit_msgs::msg::Constraints constraints;
                        for (const auto& entry : limits) {
                            const auto values = entry.second.as<std::vector<double>>();
                            if (values.size() != 2)
                                throw StepFailure(TaskError::InvalidTask,
                                                  "invalid pregrasp joint constraint");
                            moveit_msgs::msg::JointConstraint joint;
                            joint.joint_name = std::string(hand == Manipulator::Left ? "left_" : "right_") +
                                              "fr3v2_1_" + entry.first.as<std::string>();
                            joint.position = values[0];
                            joint.tolerance_above = joint.tolerance_below = values[1];
                            joint.weight = 1.;
                            constraints.joint_constraints.push_back(joint);
                        }
                        require(move.add_path_constraint(constraints), TaskError::PlanningFailed,
                                "preflight joint constraints");
                    }
                    require(move.add_pose_target(arm(hand), relative(obs, grasp, hand)), TaskError::PlanningFailed, "preflight target");
                    require(move.set_max_velocity_scaling_factor(transit_velocity), TaskError::PlanningFailed, "preflight velocity");
                    require(move.set_max_acceleration_scaling_factor(transit_acceleration), TaskError::PlanningFailed, "preflight acceleration");
                    mfr3duo_moveit::Plan plan;
                    require(move.plan(plan), TaskError::PlanningFailed, "pregrasp plan");
                    result.feasible = true;
                    result.manipulator = hand;
                    result.message = "collision-aware pregrasp plan found";
                    return result;
                } catch (const StepFailure& error) {
                    last_error = error.what();
                }
            }
            result.error = TaskError::PlanningFailed;
            result.message = last_error.empty() ? "no manipulator candidate" : last_error;
            return result;
        }
        void run_task(const RobotTask& task, const std::shared_ptr<TaskHandle::Impl>& op) {
            Clock::time_point previous;
            {
                std::lock_guard<std::mutex> lock(op->mutex);
                previous = op->step_deadline;
                if (task.timeout())
                    op->step_deadline = std::min(previous, Clock::now() + *task.timeout());
            }
            check(op);
            if (const auto* sequence = dynamic_cast<const TaskSequence*>(&task)) {
                for (std::size_t i = 0; i < sequence->tasks().size(); ++i) {
                    try {
                        run_task(*sequence->tasks()[i], op);
                    } catch (const StepFailure& e) {
                        throw StepFailure(
                            e.error, "Sequence[" + std::to_string(i) + "] " +
                                         std::string(sequence->tasks()[i]->name()) + ": " +
                                         e.what());
                    }
                }
            } else if (const auto* t = dynamic_cast<const NavigateTask*>(&task))
                navigate(*t, op);
            else if (const auto* t = dynamic_cast<const PickTask*>(&task))
                pick(*t, op);
            else if (const auto* t = dynamic_cast<const PlaceTask*>(&task))
                place(*t, op);
            else if (const auto* t = dynamic_cast<const SceneJointTask*>(&task))
                interact_scene(*t, op);
            else
                throw StepFailure(TaskError::InvalidTask, "unsupported Task type");
            {
                std::lock_guard<std::mutex> lock(op->mutex);
                op->step_deadline = previous;
            }
            check(op);
        }
        TaskResult recover(TaskResult original) {
            RCLCPP_WARN(
                node->get_logger(), "Physical task failed before recovery: %s",
                original.message.c_str());
            if (!stop_owned())
                return failure(
                    TaskError::RecoveryRequired,
                    "owned operation termination unknown: " + original.message);
            if (!holding_id.empty()) {
                auto result = recover_grasp(
                    node, scene, *observer, holding_id, holding_hand, held_relative,
                    holding_geometry, true, control, grasp_verified, grasp_width(holding_id),
                    profile["observation"]["objects"][holding_id]["epsilon_inner"].as<double>(),
                    profile["observation"]["objects"][holding_id]["epsilon_outer"].as<double>());
                if (!result.scene_confirmed)
                    return failure(
                        TaskError::RecoveryRequired, result.diagnostic + ": " + original.message);
                if (result.holding == PhysicalHolding::Released) {
                    holding_id.clear();
                    holding_hand = Manipulator::Auto;
                    relative_known = false;
                    grasp_verified = false;
                    original =
                        failure(TaskError::GraspLost, "physical grasp lost: " + original.message);
                } else if (original.error == TaskError::GraspFailed) {
                    original = failure(
                        TaskError::ExecutionFailed,
                        "still held; lift validation: " + original.message);
                }
            } else if (grasp_stage) {
                auto obs = observer->observe(recovery_id, recovery_hand);
                const auto opening = control.get_gripper_width(gripper(recovery_hand));
                if (!obs.valid || !obs.object_visible || !opening || opening.value < .075)
                    return failure(
                        TaskError::RecoveryRequired,
                        "pre-attach physical state unknown: " + original.message);
                auto restored =
                    scene.set_grasp_contact_allowed(recovery_id, arm(recovery_hand), false);
                if (!restored)
                    return failure(
                        TaskError::RecoveryRequired, restored.message + ": " + original.message);
                auto opened = control.move_gripper(
                    gripper(recovery_hand), open_width,
                    profile["fixture"]["gripper_speed"].as<double>());
                if (!opened)
                    return failure(
                        TaskError::RecoveryRequired, "grasp cleanup open: " + opened.message);
                // Only retreat after collision-checked full path. Cleanup has its own finite
                // backend deadlines.
                mfr3duo_moveit::CartesianPath path;
                auto planned = move.compute_cartesian_path(
                    arm(recovery_hand), {relative(obs, return_pose, recovery_hand)}, .005, path);
                if (!planned || path.fraction != 1)
                    return failure(
                        TaskError::RecoveryRequired,
                        "safe grasp retreat unavailable: " + original.message);
                auto executed = move.execute(path.plan);
                if (!executed)
                    return failure(
                        TaskError::RecoveryRequired,
                        "grasp cleanup retreat: " + executed.message + ": " + original.message);
                grasp_stage = false;
            }
            stage = Stage::None;
            return original;
        }
        void finish(const std::shared_ptr<TaskHandle::Impl>& op, TaskResult result) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                std::lock_guard<std::mutex> operation_lock(op->mutex);
                if (!op->result) op->result = std::move(result);
                live = false;
                const auto error = op->result->error;
                state = (error == TaskError::RecoveryRequired || error == TaskError::CancelFailed ||
                         error == TaskError::PreviousOperationNotTerminated)
                            ? RobotState::Error
                            : RobotState::Ready;
            }
            op->changed.notify_all();
        }
        bool stop_for(const std::shared_ptr<TaskHandle::Impl>& op) {
            std::lock_guard<std::mutex> serial(cancellation_mutex);
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (current != op || !live) return true;
            }
            return stop_owned();
        }
        TaskResult request_cancel(const std::shared_ptr<TaskHandle::Impl>& op) {
            {
                std::lock_guard<std::mutex> lock(op->mutex);
                if (op->result) return *op->result;
            }
            op->cancel_requested = true;
            bool not_started = false;
            if (op->dispatched.compare_exchange_strong(not_started, true)) {
                if (worker) worker->cancel();
                finish(
                    op, failure(
                            op->timed_out ? TaskError::Timeout : TaskError::Canceled,
                            "Task canceled before dispatch"));
            } else
                stop_for(op);
            std::unique_lock<std::mutex> lock(op->mutex);
            if (!op->changed.wait_for(
                    lock, terminal_timeout, [&] { return op->result.has_value(); })) {
                op->result = failure(
                    TaskError::CancelFailed, "Task worker/owned motion terminal not confirmed");
                lock.unlock();
                {
                    std::lock_guard<std::mutex> held(mutex);
                    state = RobotState::Error;
                }
                op->changed.notify_all();
                return failure(
                    TaskError::CancelFailed, "Task worker/owned motion terminal not confirmed");
            }
            return *op->result;
        }
        void watchdog() {
            std::shared_ptr<TaskHandle::Impl> op;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (!live) return;
                op = current;
            }
            {
                std::lock_guard<std::mutex> lock(op->mutex);
                if (Clock::now() >= op->step_deadline) {
                    op->timed_out = true;
                    op->cancel_requested = true;
                }
            }
            if (op->cancel_requested) stop_for(op);
        }
        void arm_monitor() {
            std::weak_ptr<Engine> weak = shared_from_this();
            monitor = node->create_wall_timer(
                50ms,
                [weak] {
                    if (auto self = weak.lock()) self->watchdog();
                },
                deadline_group);
            health_timer = node->create_wall_timer(
                200ms,
                [weak] {
                    if (auto self = weak.lock()) self->update_health();
                },
                health_group);
        }
    };
    std::shared_ptr<Engine> engine;
    explicit Impl(const rclcpp::Node::SharedPtr& node) : engine(std::make_shared<Engine>(node)) {
        engine->arm_monitor();
    }
};
Robot::Robot(const rclcpp::Node::SharedPtr& node) {
    if (!node) throw std::invalid_argument("Robot requires an external Node");
    impl_ = std::make_unique<Impl>(node);
}
Robot::~Robot() {
    try {
        auto engine = impl_->engine;
        {
            std::lock_guard<std::mutex> lock(engine->mutex);
            engine->closing = true;
        }
        cancel();
        // A running callback retains all resources until actual termination. No raw owner capture.
        if (engine->worker) engine->worker->cancel();
    } catch (...) {
    }
}
RobotState Robot::state() const {
    std::lock_guard<std::mutex> lock(impl_->engine->mutex);
    return impl_->engine->state;
}
bool Robot::is_busy() const {
    std::lock_guard<std::mutex> lock(impl_->engine->mutex);
    return impl_->engine->live;
}
bool Robot::is_ready() const { return state() == RobotState::Ready && impl_->engine->ready(); }
TaskResult Robot::initialize(std::chrono::milliseconds timeout) {
    auto e = impl_->engine;
    std::unique_lock<std::mutex> init(e->initialize_mutex, std::try_to_lock);
    if (!init.owns_lock()) return failure(TaskError::RobotBusy, "initialize already running");
    {
        std::lock_guard<std::mutex> lock(e->mutex);
        if (e->live)
            return failure(
                TaskError::PreviousOperationNotTerminated, "previous Task callback still active");
    }
    if (!valid_timeout(timeout))
        return failure(
            TaskError::InvalidTask, "positive representable initialization timeout required");
    struct InitializationGuard {
        std::shared_ptr<Impl::Engine> engine;
        ~InitializationGuard() {
            std::lock_guard<std::mutex> lock(engine->mutex);
            engine->initializing = false;
        }
    } guard{e};
    {
        std::lock_guard<std::mutex> lock(e->mutex);
        e->initializing = true;
        if (e->state != RobotState::Error) e->state = RobotState::Uninitialized;
    }
    const auto deadline = Clock::now() + timeout;
    const auto remaining = [&] {
        return std::max(
            0ms, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()));
    };
    const auto budget = [&](std::chrono::milliseconds required) {
        if (remaining() < required)
            throw StepFailure(
                TaskError::Timeout, "Robot initialize total deadline budget exhausted");
    };
    const auto observation_budget =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(
            e->node->get_parameter("grasp_observation.timeout").as_double()));
    const auto scene_budget =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(
            e->node->get_parameter("planning_scene.timeout").as_double()));
    try {
        if (e->node->get_parameter("use_sim_time").as_bool())
            return failure(TaskError::RobotNotReady, "V1 requires wall time");
        e->require(
            e->control.initialize(remaining()), TaskError::RobotNotReady, "Control readiness");
        e->require(
            e->move.initialize(remaining()), TaskError::RobotNotReady, "MoveGroup readiness");
        e->require(
            e->scene.initialize(remaining()), TaskError::RobotNotReady, "PlanningScene readiness");
        e->require(
            e->navigator.initialize(remaining()), TaskError::RobotNotReady, "Navigator readiness");
        e->refresh_environment(Manipulator::Left);
        for (const auto& id : e->object_ids) {
            budget(observation_budget);
            auto obs = e->observe(id, Manipulator::Left);
            if (!obs.valid) throw StepFailure(TaskError::RobotNotReady, "observer readiness");
        }
        if (!e->holding_id.empty() || e->grasp_stage) {
            budget(5 * scene_budget + observation_budget + 5500ms);
            auto recovered = e->recover(
                failure(TaskError::ExecutionFailed, "explicit initialize physical recovery"));
            if (recovered.error == TaskError::RecoveryRequired)
                throw StepFailure(recovered.error, recovered.message);
        }
        // Configure the physical support in the same fresh observation/tool frame as the objects.
        budget(observation_budget);
        auto obs = e->observe(e->object_ids.front(), Manipulator::Left);
        budget(scene_budget);
        e->require(
            e->scene.add_collision_object(e->support_geometry(obs, Manipulator::Left)),
            TaskError::RobotNotReady, "physical support scene");
        for (const auto& id : e->object_ids)
            if (id != e->holding_id) {
                budget(observation_budget);
                obs = e->observe(id, Manipulator::Left);
                budget(scene_budget);
                e->require(
                    e->scene.add_collision_object(e->geometry(id, obs, Manipulator::Left)),
                    TaskError::RobotNotReady, "object scene");
            }
        if (remaining() == 0ms)
            throw StepFailure(TaskError::Timeout, "Robot initialize total deadline");
        {
            std::lock_guard<std::mutex> lock(e->mutex);
            e->state = RobotState::Ready;
            e->observer_healthy = true;
            e->observer_seen = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   Clock::now().time_since_epoch())
                                   .count();
        }
        return success();
    } catch (const StepFailure& error) {
        std::lock_guard<std::mutex> lock(e->mutex);
        if (e->state != RobotState::Error) e->state = RobotState::Uninitialized;
        return failure(error.error, error.what());
    } catch (const std::exception& error) {
        return failure(TaskError::InternalError, error.what());
    }
}
TaskHandle Robot::start(std::unique_ptr<RobotTask> task) {
    TaskHandle handle;
    handle.impl_ = std::make_shared<TaskHandle::Impl>();
    auto op = handle.impl_;
    auto e = impl_->engine;
    std::lock_guard<std::mutex> serial(e->cancellation_mutex);
    std::lock_guard<std::mutex> lock(e->mutex);
    const auto reject = [&](TaskError error, const std::string& message) {
        op->result = failure(error, message);
    };
    if (e->initializing) {
        reject(TaskError::RobotBusy, "Robot initialization is active");
        return handle;
    }
    if (e->live) {
        reject(
            e->state == RobotState::Error ? TaskError::PreviousOperationNotTerminated
                                          : TaskError::RobotBusy,
            "one top-level Task is already active");
        return handle;
    }
    if (!task || !e->validate(*task)) {
        reject(TaskError::InvalidTask, "invalid/unsupported Task input");
        return handle;
    }
    if (e->closing || e->state != RobotState::Ready || !e->ready()) {
        reject(
            e->state == RobotState::Error ? TaskError::RecoveryRequired : TaskError::RobotNotReady,
            "initialize Robot readiness/recovery first");
        return handle;
    }
    auto timeout = task->timeout().value_or(e->task_timeout);
    op->deadline = Clock::now() + timeout;
    op->step_deadline = op->deadline;
    op->terminal_timeout = e->terminal_timeout;
    e->current = op;
    e->live = true;
    e->state = RobotState::Executing;
    std::weak_ptr<Impl::Engine> weak = e;
    std::weak_ptr<TaskHandle::Impl> weak_op = op;
    op->cancel = [weak, weak_op] {
        auto self = weak.lock();
        auto operation = weak_op.lock();
        return self && operation
                   ? self->request_cancel(operation)
                   : failure(TaskError::CancelFailed, "Robot worker resource lease missing");
    };
    // std::function requires a copyable capture; shared ownership is internal only.
    std::shared_ptr<RobotTask> owned(std::move(task));
    e->worker = e->node->create_wall_timer(
        1ms,
        [weak, op, owned] {
            auto self = weak.lock();
            if (!self) return;
            bool expected = false;
            if (!op->dispatched.compare_exchange_strong(expected, true)) return;
            TaskResult result;
            try {
                self->run_task(*owned, op);
                result = success();
            } catch (const StepFailure& error) {
                auto code = error.error;
                if (code != TaskError::RecoveryRequired && op->cancel_requested)
                    code = op->timed_out ? TaskError::Timeout : TaskError::Canceled;
                try {
                    result = self->recover(failure(code, error.what()));
                } catch (const std::exception& recovery_error) {
                    result = failure(
                        TaskError::RecoveryRequired,
                        std::string("physical recovery: ") + recovery_error.what());
                }
            } catch (const std::exception& error) {
                try {
                    result = self->recover(failure(TaskError::InternalError, error.what()));
                } catch (...) {
                    result =
                        failure(TaskError::RecoveryRequired, "exception during physical recovery");
                }
            }
            self->finish(op, std::move(result));
        },
        e->worker_group);
    return handle;
}
TaskResult Robot::execute(const RobotTask& task) {
    try {
        return start(task.clone()).wait();
    } catch (const std::exception& error) {
        return failure(TaskError::InvalidTask, std::string("Task clone: ") + error.what());
    }
}
Robot::PreflightResult Robot::preflight(const RobotTask& task) {
    auto e = impl_->engine;
    std::lock_guard<std::mutex> serial(e->cancellation_mutex);
    std::lock_guard<std::mutex> lock(e->mutex);
    if (e->live || e->initializing)
        return {false, Manipulator::Auto, TaskError::RobotBusy, "robot is busy"};
    if (e->closing || e->state != RobotState::Ready || !e->ready())
        return {false, Manipulator::Auto, TaskError::RobotNotReady, "robot is not ready"};
    if (!e->validate(task))
        return {false, Manipulator::Auto, TaskError::InvalidTask, "invalid/unsupported task input"};
    if (const auto* pick = dynamic_cast<const PickTask*>(&task)) return e->preflight_pick(*pick);
    return {false, Manipulator::Auto, TaskError::InvalidTask,
            "preflight currently requires one PickTask"};
}
TaskResult Robot::cancel() {
    std::shared_ptr<TaskHandle::Impl> op;
    {
        std::lock_guard<std::mutex> lock(impl_->engine->mutex);
        op = impl_->engine->current;
    }
    return op ? impl_->engine->request_cancel(op) : success();
}
TaskResult Robot::stop() { return cancel(); }
bool TaskHandle::valid() const noexcept { return static_cast<bool>(impl_); }
TaskState TaskHandle::state() const {
    if (!impl_) return TaskState::Failed;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->result       ? impl_->result->state
           : impl_->dispatched ? TaskState::Running
                               : TaskState::Pending;
}
std::optional<TaskResult> TaskHandle::result() const {
    if (!impl_) return std::nullopt;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->result;
}
TaskResult TaskHandle::cancel() {
    if (!impl_) return failure(TaskError::InvalidTask, "invalid TaskHandle");
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->result) return *impl_->result;
    }
    return impl_->cancel ? impl_->cancel() : failure(TaskError::InvalidTask, "Task has no owner");
}
TaskResult TaskHandle::wait() {
    if (!impl_) return failure(TaskError::InvalidTask, "invalid TaskHandle");
    std::unique_lock<std::mutex> lock(impl_->mutex);
    if (!impl_->result && !impl_->changed.wait_until(
                              lock, impl_->deadline, [&] { return impl_->result.has_value(); })) {
        impl_->timed_out = true;
        impl_->cancel_requested = true;
        lock.unlock();
        return cancel();
    }
    return *impl_->result;
}
}  // namespace mfr3duo_robot
