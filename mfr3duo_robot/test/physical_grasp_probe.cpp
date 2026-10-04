#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <mutex>
#include <deque>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <Eigen/Geometry>
#include "grasp_observer.hpp"
#include "grasp_recovery.hpp"
#include <mfr3duo_control/control.hpp>
#include <mfr3duo_moveit/move_group.hpp>
#include <mfr3duo_moveit/planning_scene_interface.hpp>
using namespace std::chrono_literals;
namespace {
void require(const mfr3duo_moveit::Result& result, const char* label) {
    if (!result) throw std::runtime_error(std::string(label) + ": " + result.message);
}
void require(const mfr3duo_control::Result& result, const char* label) {
    if (!result) throw std::runtime_error(std::string(label) + ": " + result.message);
}
template <typename T>
void require(const T& result, const char* label) {
    if (!static_cast<bool>(result)) throw std::runtime_error(label);
}
Eigen::Isometry3d transform(const geometry_msgs::msg::Pose& pose) {
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    result.linear() =
        Eigen::Quaterniond(
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            .toRotationMatrix();
    return result;
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
geometry_msgs::msg::PoseStamped relative_goal(
    const mfr3duo_robot::GraspObservation& obs, const Eigen::Isometry3d& world_target) {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "left_fr3v2_1_hand_tcp";
    target.pose = message(transform(obs.tool_pose.pose).inverse() * world_target);
    return target;
}
moveit_msgs::msg::CollisionObject collision(
    const std::string& id, const std::vector<double>& dimensions,
    const geometry_msgs::msg::PoseStamped& pose) {
    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.header = pose.header;
    object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions.assign(dimensions.begin(), dimensions.end());
    object.primitives = {box};
    object.primitive_poses = {pose.pose};
    return object;
}
class InvalidObserver final : public mfr3duo_robot::GraspObserver {
public:
    mfr3duo_robot::GraspObservation observe(std::string_view, mfr3duo_robot::Manipulator) override {
        return mfr3duo_robot::GraspObservation();
    }
};
}  // namespace
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("physical_grasp_probe");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    std::thread spin([&] { executor.spin(); });
    struct SpineSample {
        std::mutex mutex;
        double target = 0, position = 0, velocity = 1;
        std::chrono::steady_clock::time_point received{};
    };
    auto spine_sample = std::make_shared<SpineSample>();
    auto spine_subscription =
        node->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/spine_controller/controller_state", 10,
            [spine_sample](const control_msgs::msg::JointTrajectoryControllerState& sample) {
                const auto& desired =
                    sample.reference.positions.empty() ? sample.desired : sample.reference;
                const auto& actual =
                    sample.feedback.positions.empty() ? sample.actual : sample.feedback;
                if (desired.positions.empty() || actual.positions.empty() ||
                    actual.velocities.empty())
                    return;
                std::lock_guard<std::mutex> lock(spine_sample->mutex);
                spine_sample->target = desired.positions[0];
                spine_sample->position = actual.positions[0];
                spine_sample->velocity = actual.velocities[0];
                spine_sample->received = std::chrono::steady_clock::now();
            });
    const auto settle_spine = [&] {
        std::deque<std::pair<std::chrono::steady_clock::time_point, double>> window;
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        auto next_print = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() < deadline) {
            {
                std::lock_guard<std::mutex> lock(spine_sample->mutex);
                const auto now = std::chrono::steady_clock::now();
                if (now - spine_sample->received < 300ms) {
                    window.emplace_back(now, spine_sample->position);
                    while (!window.empty() && now - window.front().first > 600ms)
                        window.pop_front();
                    double low = 1e9, high = -1e9;
                    for (const auto& sample : window) {
                        low = std::min(low, sample.second);
                        high = std::max(high, sample.second);
                    }
                    if (now >= next_print) {
                        std::cout << "SPINE_SETTLE target=" << spine_sample->target
                                  << " actual=" << spine_sample->position
                                  << " velocity=" << spine_sample->velocity
                                  << " range=" << high - low << std::endl;
                        next_print = now + 500ms;
                    }
                    if (window.size() > 1 && now - window.front().first >= 500ms &&
                        high - low < .0005 && std::abs(spine_sample->velocity) < .001)
                        return;
                }
            }
            // The lock is released before sleeping below.
            std::this_thread::sleep_for(20ms);
        }
        throw std::runtime_error("inactive spine did not settle before Cartesian stage");
    };
    int status = 0;
    {
        mfr3duo_control::Control control(node);
        mfr3duo_moveit::MoveGroup move(node);
        mfr3duo_moveit::PlanningSceneInterface scene(node);
        mfr3duo_robot::SimulationGraspObserver observer(node);
        bool attached = false;
        try {
            require(control.initialize(60s), "Control initialize");
            require(
                control.move_gripper(mfr3duo_control::Gripper::Right, .08, .05), "right Move open");
            const auto empty = control.grasp_gripper(mfr3duo_control::Gripper::Right, .04, .05, 20);
            require(
                empty.code == mfr3duo_control::ErrorCode::ExecutionFailed,
                "empty physical right Grasp must fail");
            require(
                control.move_gripper(mfr3duo_control::Gripper::Right, .08, .05),
                "right Move after failed grasp");
            std::cout
                << "RIGHT_GRIPPER_EMPTY_GRASP_PASS actual simulation opening and device failure\n";

            const auto initialized = move.initialize(60s);
            if (!initialized)
                std::cerr << "MOVE_INITIALIZE " << static_cast<int>(initialized.code) << ' '
                          << initialized.message << std::endl;
            require(initialized, "MoveGroup initialize");
            require(scene.initialize(30s), "scene initialize");
            auto observed = observer.observe("box", mfr3duo_robot::Manipulator::Left);
            require(observed.valid, "initial observation");
            const double initial_z = observed.object_pose.pose.position.z;
            auto table_world = Eigen::Isometry3d::Identity();
            table_world.translation() = Eigen::Vector3d(.8, .75, .46);
            const auto table_added = scene.add_collision_object(
                collision("grasp_table", {.24, .24, 1.0}, relative_goal(observed, table_world)));
            if (!table_added) {
                moveit_msgs::msg::CollisionObject actual;
                if (scene.get_object("grasp_table", actual)) {
                    std::cerr << "TABLE_QUERY frame=" << actual.header.frame_id
                              << " object_pose=" << actual.pose.position.x << ','
                              << actual.pose.position.y << ',' << actual.pose.position.z;
                    for (const auto& pose : actual.primitive_poses)
                        std::cerr << " primitive_pose=" << pose.position.x << ',' << pose.position.y
                                  << ',' << pose.position.z;
                    std::cerr << std::endl;
                }
            }
            require(table_added, "table scene");
            require(
                scene.add_collision_object(collision(
                    "box", {.04, .04, .05},
                    relative_goal(observed, transform(observed.object_pose.pose)))),
                "object scene");
            require(
                control.move_gripper(mfr3duo_control::Gripper::Left, .08, .05),
                "open before pregrasp");
            require(move.add_group(mfr3duo_moveit::RobotGroup::LeftArm), "left arm group");
            auto pregrasp = Eigen::Isometry3d::Identity();
            pregrasp.translation() = Eigen::Vector3d(.8, .75, initial_z + .12);
            pregrasp.linear() = Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitX())
                                    .toRotationMatrix();
            observed = observer.observe("box", mfr3duo_robot::Manipulator::Left);
            require(observed.valid, "fresh pregrasp observation");
            require(
                move.add_pose_target(
                    mfr3duo_moveit::RobotGroup::LeftArm, relative_goal(observed, pregrasp)),
                "pregrasp target");
            const auto planned = move.move();
            if (!planned)
                std::cerr << "PREGRASP " << static_cast<int>(planned.code) << ' ' << planned.message
                          << '\n';
            require(planned, "pregrasp move");
            require(move.clear_targets(), "clear pose target");
            settle_spine();
            require(move.set_max_velocity_scaling_factor(.05), "precision grasp velocity scale");
            require(
                move.set_max_acceleration_scaling_factor(.02),
                "precision grasp acceleration scale");
            const auto cartesian = [&](const Eigen::Isometry3d& target, const char* label) {
                const auto current = observer.observe("box", mfr3duo_robot::Manipulator::Left);
                require(current.valid, "fresh Cartesian observation");
                mfr3duo_moveit::CartesianPath path;
                auto result = move.compute_cartesian_path(
                    mfr3duo_moveit::RobotGroup::LeftArm, {relative_goal(current, target)}, .005,
                    path);
                std::cout << label << " fraction=" << path.fraction
                          << " code=" << static_cast<int>(result.code) << ' ' << result.message
                          << std::endl;
                require(result, label);
                require(path.fraction == 1., "full Cartesian path required");
                const auto executed = move.execute(path.plan);
                if (!executed) {
                    std::cerr << label << " execution code=" << static_cast<int>(executed.code)
                              << ' ' << executed.message << std::endl;
                    const auto joints = control.get_arm_joint_positions(mfr3duo_control::Arm::Left);
                    if (joints) {
                        std::cerr << "ARM actual=";
                        for (double value : joints.value) std::cerr << value << ',';
                        std::cerr << " goal=";
                        for (double value :
                             path.plan.trajectory().joint_trajectory.points.back().positions)
                            std::cerr << value << ',';
                        std::cerr << std::endl;
                    }
                }
                require(executed, label);
            };
            require(
                scene.set_grasp_contact_allowed("box", mfr3duo_moveit::RobotGroup::LeftArm, true),
                "allow selected fingers");
            auto grasp = pregrasp;
            grasp.translation().z() = initial_z;
            cartesian(grasp, "approach");
            auto close = control.grasp_gripper(mfr3duo_control::Gripper::Left, .04, .05, 20.);
            if (!close)
                std::cerr << "CLOSE " << static_cast<int>(close.code) << ' ' << close.message
                          << std::endl;
            require(close, "close action");
            auto closed = control.get_gripper_position(mfr3duo_control::Gripper::Left);
            require(closed, "closing state");
            require(closed.value > .005 && closed.value < .035, "actual blocked closing width");
            const auto until = std::chrono::steady_clock::now() + 200ms;
            while (std::chrono::steady_clock::now() < until) {
                observed = observer.observe("box", mfr3duo_robot::Manipulator::Left);
                require(observed.valid, "fresh target/tool observation");
                std::this_thread::sleep_for(20ms);
            }
            const auto held_relative =
                transform(observed.tool_pose.pose).inverse() * transform(observed.object_pose.pose);
            require(
                scene.add_collision_object(collision(
                    "box", {.04, .04, .05},
                    relative_goal(observed, transform(observed.object_pose.pose)))),
                "fresh grasped world pose");
            require(
                scene.attach_object("box", mfr3duo_moveit::RobotGroup::LeftArm),
                "attach after contact confirmation");
            attached = true;
            require(
                scene.set_grasp_contact_allowed("box", mfr3duo_moveit::RobotGroup::LeftArm, false),
                "exit temporary contact stage");
            auto lift = transform(observed.tool_pose.pose);
            lift.translation().z() += .10;
            cartesian(lift, "lift");
            const auto window = std::chrono::steady_clock::now() + 500ms;
            while (std::chrono::steady_clock::now() < window) {
                observed = observer.observe("box", mfr3duo_robot::Manipulator::Left);
                require(observed.valid, "lift target observation");
                const auto relative = transform(observed.tool_pose.pose).inverse() *
                                      transform(observed.object_pose.pose);
                const double drift = (relative.translation() - held_relative.translation()).norm();
                const double angle =
                    Eigen::AngleAxisd(relative.linear() * held_relative.linear().transpose())
                        .angle();
                std::cout << "held z=" << observed.object_pose.pose.position.z << " drift=" << drift
                          << " angle=" << angle << std::endl;
                require(
                    observed.object_pose.pose.position.z > initial_z + .08,
                    "physical object lifted");
                require(drift <= .005 && angle <= .05, "relative grasp stability");
                std::this_thread::sleep_for(50ms);
            }
            std::cout << "PHYSICAL_PICK_PASS Level B same-instance poses, lifted box, stable "
                         "relative pose\n";
            // Inject a failed higher-level step only after the owned trajectory is terminal.
            // Recovery consumes real same-instance observations, without opening the gripper.
            const auto stopped = move.stop();
            require(stopped, "confirmed stop before failure recovery");
            const auto object_geometry = collision(
                "box", {.04, .04, .05},
                relative_goal(observed, transform(observed.object_pose.pose)));
            const auto held = mfr3duo_robot::recover_grasp(
                node, scene, observer, "box", mfr3duo_robot::Manipulator::Left, held_relative,
                object_geometry, true, control, true);
            require(
                held.holding == mfr3duo_robot::PhysicalHolding::Held && held.scene_confirmed,
                "still-held recovery");
            bool retained = false;
            require(scene.is_attached("box", retained) && retained, "held attachment retained");
            const auto unverified = mfr3duo_robot::recover_grasp(
                node, scene, observer, "box", mfr3duo_robot::Manipulator::Left, held_relative,
                object_geometry, true, control, false);
            require(
                unverified.holding == mfr3duo_robot::PhysicalHolding::Unknown &&
                    !unverified.scene_confirmed,
                "static proximity without verified lift must remain unknown");
            InvalidObserver invalid_observer;
            const auto unknown = mfr3duo_robot::recover_grasp(
                node, scene, invalid_observer, "box", mfr3duo_robot::Manipulator::Left,
                held_relative, object_geometry, true, control, true);
            require(
                unknown.holding == mfr3duo_robot::PhysicalHolding::Unknown &&
                    !unknown.scene_confirmed,
                "missing observation requires explicit recovery");
            const auto unterminated = mfr3duo_robot::recover_grasp(
                node, scene, observer, "box", mfr3duo_robot::Manipulator::Left, held_relative,
                object_geometry, false, control, true);
            require(
                unterminated.holding == mfr3duo_robot::PhysicalHolding::Unknown &&
                    !unterminated.scene_confirmed,
                "unknown motion termination prevents scene recovery");
            require(
                scene.is_attached("box", retained) && retained,
                "unknown recovery must retain attachment");
            observed = observer.observe("box", mfr3duo_robot::Manipulator::Left);
            require(observed.valid, "unknown recovery did not release physical object");
            std::cout << "RECOVERY_HELD_UNKNOWN_PASS actual held observation; missing observation "
                         "and unknown motion preserve physical/scene state"
                      << std::endl;
            auto place = transform(observed.tool_pose.pose);
            place.translation().z() -= .08;
            cartesian(place, "place approach above support");
            require(
                control.move_gripper(mfr3duo_control::Gripper::Left, .08, .05), "release gripper");
            std::this_thread::sleep_for(500ms);
            observed = observer.observe("box", mfr3duo_robot::Manipulator::Left);
            require(observed.valid, "physical release confirmation");
            require(
                std::abs(observed.object_pose.pose.position.z - initial_z) < .01,
                "object on physical table");
            const auto released = mfr3duo_robot::recover_grasp(
                node, scene, observer, "box", mfr3duo_robot::Manipulator::Left, held_relative,
                object_geometry, true, control, true);
            require(
                released.holding == mfr3duo_robot::PhysicalHolding::Released &&
                    released.scene_confirmed,
                "released recovery restores actual world pose");
            bool remaining_attachment = true;
            require(
                scene.is_attached("box", remaining_attachment) && !remaining_attachment,
                "released recovery removed attachment");
            attached = false;
            std::cout << "RECOVERY_RELEASED_PASS actual released observation restores world"
                      << std::endl;
            auto retreat = transform(observed.tool_pose.pose);
            retreat.translation().z() += .12;
            cartesian(retreat, "retreat");
            bool world = false;
            require(scene.has_object("box", world) && world, "released world representation");
            std::cout << "PHYSICAL_PLACE_PASS Level B released on support, detached/world "
                         "restored, full retreat\n";
        } catch (const std::exception& e) {
            {
                std::lock_guard<std::mutex> lock(spine_sample->mutex);
                std::cerr << "SPINE_FAILURE target=" << spine_sample->target
                          << " actual=" << spine_sample->position
                          << " velocity=" << spine_sample->velocity << std::endl;
            }
            auto stopped = move.stop();
            auto observed = observer.observe("box", mfr3duo_robot::Manipulator::Left);
            std::cerr << "PHYSICAL_GRASP_FAIL " << e.what()
                      << " motion_stop=" << static_cast<int>(stopped.code)
                      << " attached=" << attached << " observation=" << observed.valid

                      << " tool=" << observed.tool_pose.pose.position.x << ','
                      << observed.tool_pose.pose.position.y << ','
                      << observed.tool_pose.pose.position.z
                      << " object=" << observed.object_pose.pose.position.x << ','
                      << observed.object_pose.pose.position.y << ','
                      << observed.object_pose.pose.position.z << std::endl;
            // No guessing or unconditional detach/open when physical state is unknown.
            status = 1;
        }
    }
    executor.cancel();
    spin.join();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return status;
}
