cmake_minimum_required(VERSION 3.16)
project(mfr3duo_hardware VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mfr3duo_mujoco CONFIG REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(mfr3duo_mujoco_system SHARED
  src/mujoco_system.cpp src/mujoco_sensor_bridge.cpp)
target_compile_features(mfr3duo_mujoco_system PUBLIC cxx_std_17)
target_include_directories(mfr3duo_mujoco_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mfr3duo_mujoco_system mfr3duo_mujoco::mfr3duo_mujoco)
ament_target_dependencies(mfr3duo_mujoco_system
  hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs)
pluginlib_export_plugin_description_file(hardware_interface mfr3duo_hardware.xml)
install(TARGETS mfr3duo_mujoco_system LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ros2_control/ DESTINATION share/${PROJECT_NAME}/ros2_control)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME mfr3duo_hardware_runtime_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/runtime_integration.py)
  set_tests_properties(mfr3duo_hardware_runtime_integration PROPERTIES TIMEOUT 90)
  add_executable(mfr3duo_control_cycle_benchmark test/control_cycle_benchmark.cpp)
  target_compile_features(mfr3duo_control_cycle_benchmark PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_control_cycle_benchmark mfr3duo_mujoco::mfr3duo_mujoco)
  add_executable(mfr3duo_mujoco_system_cycle_benchmark
    test/mujoco_system_cycle_benchmark.cpp)
  target_compile_features(mfr3duo_mujoco_system_cycle_benchmark PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_mujoco_system_cycle_benchmark mfr3duo_mujoco_system)
  ament_target_dependencies(mfr3duo_mujoco_system_cycle_benchmark hardware_interface rclcpp)
  add_executable(mfr3duo_hardware_long_running_test test/long_running_test.cpp)
  target_compile_features(mfr3duo_hardware_long_running_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_hardware_long_running_test mfr3duo_mujoco::mfr3duo_mujoco)
  add_test(NAME mfr3duo_hardware_long_running_test COMMAND mfr3duo_hardware_long_running_test)
  set_tests_properties(mfr3duo_hardware_long_running_test PROPERTIES TIMEOUT 240)
  add_executable(mfr3duo_hardware_interface_test test/interface_test.cpp)
  target_compile_features(mfr3duo_hardware_interface_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_hardware_interface_test mfr3duo_mujoco_system)
  ament_target_dependencies(mfr3duo_hardware_interface_test hardware_interface rclcpp)
  add_test(NAME mfr3duo_hardware_interface_test
    COMMAND mfr3duo_hardware_interface_test)
  # This test links MuJoCo before Fast DDS is loaded. MuJoCo exports its bundled
  # tinyxml2 symbols, which otherwise intercept Fast DDS's system tinyxml2 calls.
  # The production plugin is loaded after ROS initializes its middleware.
  find_file(MFR3DUO_SYSTEM_TINYXML2 NAMES libtinyxml2.so.9
    PATHS /lib /usr/lib PATH_SUFFIXES ${CMAKE_LIBRARY_ARCHITECTURE} REQUIRED)
  set_tests_properties(mfr3duo_hardware_interface_test PROPERTIES
    ENVIRONMENT "LD_PRELOAD=${MFR3DUO_SYSTEM_TINYXML2}")
  add_executable(mfr3duo_mujoco_link_test test/link_mujoco.cpp)
  target_compile_features(mfr3duo_mujoco_link_test PRIVATE cxx_std_17)
  target_link_libraries(mfr3duo_mujoco_link_test
    PRIVATE mfr3duo_mujoco::mfr3duo_mujoco)
  add_test(NAME mfr3duo_mujoco_link_test COMMAND mfr3duo_mujoco_link_test)
endif()
ament_package()
