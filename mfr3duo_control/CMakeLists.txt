cmake_minimum_required(VERSION 3.16)
project(mfr3duo_control VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(mfr3duo_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(mfr3duo_control SHARED src/control.cpp)
target_compile_features(mfr3duo_control PUBLIC cxx_std_17)
target_compile_options(mfr3duo_control PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(mfr3duo_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mfr3duo_control rclcpp rclcpp_action control_msgs mfr3duo_msgs controller_manager_msgs
  geometry_msgs trajectory_msgs sensor_msgs std_msgs urdf)
install(TARGETS mfr3duo_control EXPORT export_mfr3duo_control
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin INCLUDES DESTINATION include)
ament_export_targets(export_mfr3duo_control HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action control_msgs mfr3duo_msgs controller_manager_msgs
  geometry_msgs trajectory_msgs sensor_msgs std_msgs urdf)

add_library(tmr_kinematics SHARED src/tmr_kinematics.cpp src/tmr_odometry.cpp)
target_compile_features(tmr_kinematics PUBLIC cxx_std_17)
target_compile_options(tmr_kinematics PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(tmr_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(mfr3duo_control_plugins SHARED src/tmr_controller.cpp src/gripper_controller.cpp)
target_compile_features(mfr3duo_control_plugins PUBLIC cxx_std_17)
target_compile_options(mfr3duo_control_plugins PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mfr3duo_control_plugins tmr_kinematics)
ament_target_dependencies(mfr3duo_control_plugins
  controller_interface hardware_interface geometry_msgs pluginlib rclcpp rclcpp_lifecycle realtime_tools rclcpp_action control_msgs mfr3duo_msgs nav_msgs tf2_msgs diagnostic_msgs)
pluginlib_export_plugin_description_file(controller_interface mfr3duo_control_plugins.xml)

install(TARGETS tmr_kinematics EXPORT export_tmr_kinematics
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS mfr3duo_control_plugins
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_tmr_kinematics HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  add_executable(tmr_odometry_test test/tmr_odometry_test.cpp)
  target_link_libraries(tmr_odometry_test tmr_kinematics)
  add_test(NAME tmr_odometry_test COMMAND tmr_odometry_test)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME odometry_runtime_normal COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/odometry_runtime_test.py 500)
  add_test(NAME odometry_runtime_slow COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/odometry_runtime_test.py 1000 slow)
  set_tests_properties(odometry_runtime_normal odometry_runtime_slow
    PROPERTIES TIMEOUT 130 RUN_SERIAL TRUE)
  add_executable(gripper_controller_test test/gripper_controller_test.cpp)
  target_include_directories(gripper_controller_test PRIVATE src)
  target_link_libraries(gripper_controller_test mfr3duo_control_plugins)
  ament_target_dependencies(gripper_controller_test controller_interface hardware_interface
    rclcpp rclcpp_lifecycle rclcpp_action control_msgs mfr3duo_msgs realtime_tools)
  add_test(NAME gripper_controller_test COMMAND gripper_controller_test)
  set_tests_properties(gripper_controller_test PROPERTIES TIMEOUT 30)
  add_executable(control_action_test test/control_action_test.cpp)
  target_link_libraries(control_action_test mfr3duo_control)
  ament_target_dependencies(control_action_test rclcpp rclcpp_action control_msgs mfr3duo_msgs
    controller_manager_msgs sensor_msgs std_msgs geometry_msgs)
  add_test(NAME control_action_test COMMAND control_action_test)
  set_tests_properties(control_action_test PROPERTIES TIMEOUT 30)

  add_executable(tmr_kinematics_test test/tmr_kinematics_test.cpp)
  target_link_libraries(tmr_kinematics_test tmr_kinematics)
  add_test(NAME tmr_kinematics_test COMMAND tmr_kinematics_test)
  add_executable(tmr_controller_test test/tmr_controller_test.cpp)
  target_include_directories(tmr_controller_test PRIVATE src)
  target_link_libraries(tmr_controller_test mfr3duo_control_plugins tmr_kinematics)
  ament_target_dependencies(tmr_controller_test
    controller_interface hardware_interface geometry_msgs rclcpp rclcpp_lifecycle realtime_tools)
  add_test(NAME tmr_controller_test COMMAND tmr_controller_test)
  set_tests_properties(tmr_controller_test PROPERTIES TIMEOUT 30)
  add_executable(control_runtime_client test/control_runtime_client.cpp)
  target_link_libraries(control_runtime_client mfr3duo_control)
  ament_target_dependencies(control_runtime_client rclcpp trajectory_msgs)
  foreach(rate 500 1000)
    add_test(NAME control_runtime_${rate}hz
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/runtime_integration.py ${rate} $<TARGET_FILE:control_runtime_client>)
    set_tests_properties(control_runtime_${rate}hz PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
  endforeach()
  add_test(NAME control_installed_consumer
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/installed_consumer_test.py)
  set_tests_properties(control_installed_consumer PROPERTIES TIMEOUT 100)
  add_test(NAME control_launch_contract
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/launch_contract_test.py)
endif()
ament_package()
