cmake_minimum_required(VERSION 3.16)
project(mfr3duo_hardware VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mfr3duo_mujoco CONFIG REQUIRED)

# Core hardware library. This target is the whole-robot C++ API and must stay
# free of any ROS dependency: it only links the MuJoCo backend privately.
add_library(mfr3duo_hardware SHARED src/robot_hardware.cpp)
target_compile_features(mfr3duo_hardware PUBLIC cxx_std_17)
target_include_directories(mfr3duo_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(mfr3duo_hardware PRIVATE MFR3DUO_HARDWARE_BUILDING_LIBRARY)
target_compile_options(mfr3duo_hardware PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mfr3duo_hardware PRIVATE mfr3duo_mujoco::mfr3duo_mujoco)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ROS 2 adapter library. It adapts RobotHardware to ros2_control and to ROS
# sensor topics; it is the only target that may depend on ROS 2.
add_library(mfr3duo_ros2_adapter SHARED
  src/ros2_control_adapter.cpp
  src/ros2_sensor_adapter.cpp)
target_compile_features(mfr3duo_ros2_adapter PUBLIC cxx_std_17)
target_include_directories(mfr3duo_ros2_adapter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(mfr3duo_ros2_adapter PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mfr3duo_ros2_adapter mfr3duo_hardware)
ament_target_dependencies(mfr3duo_ros2_adapter
  hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs)

pluginlib_export_plugin_description_file(hardware_interface mfr3duo_hardware.xml)

install(TARGETS mfr3duo_hardware mfr3duo_ros2_adapter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
# Only the official public headers are installed. The ROS adapters keep their
# headers inside src/ and are not part of the user-facing SDK.
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ros2_control/ DESTINATION share/${PROJECT_NAME}/ros2_control)

if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME grasp_observation_runtime COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/grasp_observation_runtime_test.py)
  set_tests_properties(grasp_observation_runtime PROPERTIES TIMEOUT 90 RUN_SERIAL TRUE)

  # MuJoCo exports its bundled tinyxml2 symbols. Every test that links MuJoCo and
  # then loads the ROS 2 middleware must prefer the system tinyxml2 instead; the
  # production plugin is loaded after ROS initializes its middleware and is not
  # affected.
  find_file(MFR3DUO_SYSTEM_TINYXML2 NAMES libtinyxml2.so.9
    PATHS /lib /usr/lib PATH_SUFFIXES ${CMAKE_LIBRARY_ARCHITECTURE} REQUIRED)

  add_executable(mfr3duo_hardware_robot_hardware_test test/robot_hardware_test.cpp)
  target_compile_features(mfr3duo_hardware_robot_hardware_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_hardware_robot_hardware_test PRIVATE mfr3duo_hardware)
  add_test(NAME mfr3duo_hardware_robot_hardware_test
    COMMAND mfr3duo_hardware_robot_hardware_test)
  set_tests_properties(mfr3duo_hardware_robot_hardware_test PROPERTIES TIMEOUT 300)

  add_executable(mfr3duo_hardware_concurrency_test
    test/robot_hardware_concurrency_test.cpp)
  target_compile_features(mfr3duo_hardware_concurrency_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_hardware_concurrency_test PRIVATE mfr3duo_hardware)
  add_test(NAME mfr3duo_hardware_concurrency_test
    COMMAND mfr3duo_hardware_concurrency_test)
  set_tests_properties(mfr3duo_hardware_concurrency_test PROPERTIES TIMEOUT 300)

  add_executable(mfr3duo_hardware_interface_test test/interface_test.cpp)
  target_compile_features(mfr3duo_hardware_interface_test PRIVATE cxx_std_17)
  target_include_directories(mfr3duo_hardware_interface_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(mfr3duo_hardware_interface_test mfr3duo_ros2_adapter)
  ament_target_dependencies(mfr3duo_hardware_interface_test hardware_interface rclcpp)
  add_test(NAME mfr3duo_hardware_interface_test COMMAND mfr3duo_hardware_interface_test)
  set_tests_properties(mfr3duo_hardware_interface_test PROPERTIES
    ENVIRONMENT "LD_PRELOAD=${MFR3DUO_SYSTEM_TINYXML2}")

  add_test(NAME mfr3duo_hardware_runtime_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/runtime_integration.py)
  set_tests_properties(mfr3duo_hardware_runtime_integration PROPERTIES TIMEOUT 90)

  add_executable(mfr3duo_control_cycle_benchmark test/control_cycle_benchmark.cpp)
  target_compile_features(mfr3duo_control_cycle_benchmark PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_control_cycle_benchmark PRIVATE mfr3duo_hardware)
  # One simulation per process: a second one is unreliable on software OpenGL.
  add_test(NAME mfr3duo_control_cycle_benchmark_1000hz
    COMMAND mfr3duo_control_cycle_benchmark 1)
  add_test(NAME mfr3duo_control_cycle_benchmark_500hz
    COMMAND mfr3duo_control_cycle_benchmark 2)
  set_tests_properties(mfr3duo_control_cycle_benchmark_1000hz
    mfr3duo_control_cycle_benchmark_500hz PROPERTIES TIMEOUT 300)

  add_executable(mfr3duo_ros2_adapter_cycle_benchmark
    test/ros2_control_adapter_cycle_benchmark.cpp)
  target_compile_features(mfr3duo_ros2_adapter_cycle_benchmark PRIVATE cxx_std_17)
  target_include_directories(mfr3duo_ros2_adapter_cycle_benchmark
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(mfr3duo_ros2_adapter_cycle_benchmark mfr3duo_ros2_adapter)
  ament_target_dependencies(mfr3duo_ros2_adapter_cycle_benchmark hardware_interface rclcpp)
  add_test(NAME mfr3duo_ros2_adapter_cycle_benchmark_1000hz
    COMMAND mfr3duo_ros2_adapter_cycle_benchmark 0.001)
  add_test(NAME mfr3duo_ros2_adapter_cycle_benchmark_500hz
    COMMAND mfr3duo_ros2_adapter_cycle_benchmark 0.002)
  set_tests_properties(mfr3duo_ros2_adapter_cycle_benchmark_1000hz
    mfr3duo_ros2_adapter_cycle_benchmark_500hz PROPERTIES
    TIMEOUT 300
    ENVIRONMENT "LD_PRELOAD=${MFR3DUO_SYSTEM_TINYXML2}")

  add_executable(mfr3duo_hardware_long_running_test test/long_running_test.cpp)
  target_compile_features(mfr3duo_hardware_long_running_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_hardware_long_running_test PRIVATE mfr3duo_hardware)
  add_test(NAME mfr3duo_hardware_long_running_test COMMAND mfr3duo_hardware_long_running_test)
  set_tests_properties(mfr3duo_hardware_long_running_test PROPERTIES TIMEOUT 300)

  add_executable(mfr3duo_mujoco_link_test test/link_mujoco.cpp)
  target_compile_features(mfr3duo_mujoco_link_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_mujoco_link_test
    PRIVATE mfr3duo_mujoco::mfr3duo_mujoco)
  add_test(NAME mfr3duo_mujoco_link_test COMMAND mfr3duo_mujoco_link_test)
endif()

ament_package()
