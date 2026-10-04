"""The public MoveGroup target must work without source-tree include paths."""
import os
from pathlib import Path
import subprocess
import tempfile

with tempfile.TemporaryDirectory(prefix='mfr3duo-moveit-consumer-') as directory:
    root = Path(directory)
    (root / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(moveit_consumer LANGUAGES CXX)
find_package(mfr3duo_moveit REQUIRED)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_link_libraries(consumer PRIVATE mfr3duo_moveit::mfr3duo_moveit)
''')
    (root / 'main.cpp').write_text('''#include <mfr3duo_moveit/move_group.hpp>
#include <type_traits>
static_assert(std::is_same_v<decltype(std::declval<mfr3duo_moveit::Plan>().trajectory()), const moveit_msgs::msg::RobotTrajectory&>);
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  {
    mfr3duo_moveit::MoveGroup move(rclcpp::Node::make_shared("installed_moveit_consumer"));
    if (!move.add_group(mfr3duo_moveit::RobotGroup::LeftArm)) result = 1;
    mfr3duo_moveit::Plan plan;
    if (plan.valid() || move.plan(plan).code != mfr3duo_moveit::ErrorCode::NotInitialized) result = 1;
  }
  rclcpp::shutdown(); return result;
}
''')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(10 + os.getpid() % 80))
    environment.setdefault('ROS_LOG_DIR', str(root / 'logs'))
    subprocess.run(['cmake', '-S', str(root), '-B', str(root / 'build')], check=True, timeout=45, env=environment)
    subprocess.run(['cmake', '--build', str(root / 'build')], check=True, timeout=45, env=environment)
    subprocess.run([str(root / 'build/consumer')], check=True, timeout=15, env=environment)
    print('PASS installed MoveGroup exported target consumer')
