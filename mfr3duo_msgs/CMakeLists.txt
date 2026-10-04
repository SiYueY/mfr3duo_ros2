cmake_minimum_required(VERSION 3.16)
project(mfr3duo_msgs VERSION 0.1.0 LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspEpsilon.msg" "action/Move.action" "action/Grasp.action")
ament_export_dependencies(rosidl_default_runtime)
install(FILES LICENSE NOTICE.md README.md upstream.json DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 COMPONENTS Interpreter REQUIRED)
  add_test(NAME msgs_definition_contract COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/definition_contract.py)
endif()
ament_package()
