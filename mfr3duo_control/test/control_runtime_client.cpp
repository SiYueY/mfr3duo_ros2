#include <chrono>
#include <iostream>
#include <thread>

#include "mfr3duo_control/control.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char** argv) {
    using namespace mfr3duo_control;
    using namespace std::chrono_literals;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("control_runtime_client");
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3);
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    int status = 0;
    {
        Control control(node);
        const auto check = [&](const Result& result) {
            if (!result) {
                std::cerr << static_cast<int>(result.code) << ": " << result.message << '\n';
                status = 1;
            }
            return bool(result);
        };
        std::cout << "Control initialize" << std::endl;
        if (check(control.initialize(15s))) {
            std::cout << "Control initialized" << std::endl;
            for (const auto arm : {Arm::Left, Arm::Right}) {
                const auto state = control.get_arm_joint_positions(arm);
                if (!check(state.result)) break;
                auto positions = state.value;
                positions.front() += .02;
                std::cout << "Control arm " << static_cast<int>(arm) << std::endl;
                if (!check(control.command_arm_joint_position(arm, positions, 1s))) break;
            }
            std::cout << "Control spine" << std::endl;
            const auto spine = control.get_spine_position();
            if (check(spine.result)) check(control.command_spine_position(spine.value + .01, 1s));
            for (const auto gripper : {Gripper::Left, Gripper::Right}) {
                std::cout << "Control gripper " << static_cast<int>(gripper) << " open"
                          << std::endl;
                check(control.command_gripper(gripper, .04));
                std::cout << "Control gripper " << static_cast<int>(gripper) << " close"
                          << std::endl;
                check(control.command_gripper(gripper, .0));
                check(control.get_gripper_position(gripper).result);
                if (control.command_gripper(gripper, .02, 0.0).code != ErrorCode::InvalidArgument)
                    status = 1;
            }
            std::cout << "Control velocity and readiness" << std::endl;
            check(control.command_base_velocity({}));
            check(control.stop_base());
            if (!control.is_ready()) status = 1;
        }
    }
    std::cout << "Control cleanup completed" << std::endl;
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    if (!status) std::cout << "Control facade real MuJoCo arm/spine/gripper/state/velocity PASS\n";
    return status;
}
