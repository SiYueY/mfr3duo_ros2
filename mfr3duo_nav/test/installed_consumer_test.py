"""The public Navigator target must work without source-tree include paths."""
import os
from pathlib import Path
import subprocess
import tempfile

with tempfile.TemporaryDirectory(prefix='mfr3duo-nav-consumer-') as directory:
    root = Path(directory)
    (root / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(nav_consumer LANGUAGES CXX)
find_package(mfr3duo_nav REQUIRED)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_link_libraries(consumer PRIVATE mfr3duo_nav::mfr3duo_nav)
''')
    (root / 'main.cpp').write_text('''#include <mfr3duo_nav/navigator.hpp>
#include <type_traits>
static_assert(!std::is_copy_constructible_v<mfr3duo_nav::Navigator>);
static_assert(std::is_copy_constructible_v<mfr3duo_nav::NavigationHandle>);
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  {
    mfr3duo_nav::Navigator navigator(rclcpp::Node::make_shared("installed_nav_consumer"));
    geometry_msgs::msg::PoseStamped pose;
    if (navigator.get_current_pose(pose).code != mfr3duo_nav::ErrorCode::NotInitialized) result = 1;
    mfr3duo_nav::NavigationHandle handle;
    if (handle.valid()) result = 1;
  }
  rclcpp::shutdown(); return result;
}
''')
    environment = dict(os.environ, ROS_DOMAIN_ID=str(10 + os.getpid() % 80))
    environment.setdefault('ROS_LOG_DIR', str(root / 'logs'))
    subprocess.run(['cmake', '-S', str(root), '-B', str(root / 'build')], check=True, timeout=45, env=environment)
    subprocess.run(['cmake', '--build', str(root / 'build')], check=True, timeout=45, env=environment)
    subprocess.run([str(root / 'build/consumer')], check=True, timeout=15, env=environment)
    print('PASS installed Navigator exported target consumer')
