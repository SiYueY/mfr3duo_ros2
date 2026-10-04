"""Build a downstream SDK consumer using only the installed exported targets."""
import os
from pathlib import Path
import subprocess
import tempfile

with tempfile.TemporaryDirectory(prefix='mfr3duo-control-consumer-') as directory:
    root = Path(directory)
    (root / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(control_consumer LANGUAGES CXX)
find_package(mfr3duo_control REQUIRED)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_link_libraries(consumer PRIVATE mfr3duo_control::mfr3duo_control mfr3duo_control::tmr_kinematics)
''')
    (root / 'main.cpp').write_text('''#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <mfr3duo_control/control.hpp>
#include <mfr3duo_control/tmr_kinematics.hpp>
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    {
        mfr3duo_control::Control control(rclcpp::Node::make_shared("installed_control_consumer"));
        if (control.initialize(std::chrono::milliseconds(10)).code != mfr3duo_control::ErrorCode::Timeout) result = 1;
        if (control.command_gripper(mfr3duo_control::Gripper::Left, .02, 0.0).code != mfr3duo_control::ErrorCode::InvalidArgument) result = 1;
        if (mfr3duo_control::TmrKinematics::shortest_angle(0.0) != 0.0) result = 1;
    }
    rclcpp::shutdown();
    return result;
}
''')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(10 + os.getpid() % 80))
    environment.setdefault('ROS_LOG_DIR', str(root / 'logs'))
    subprocess.run(['cmake', '-S', str(root), '-B', str(root / 'build')], check=True, timeout=45, env=environment)
    subprocess.run(['cmake', '--build', str(root / 'build')], check=True, timeout=45, env=environment)
    subprocess.run([str(root / 'build/consumer')], check=True, timeout=10, env=environment)
    print('PASS installed Control and kinematics exported target consumer')
