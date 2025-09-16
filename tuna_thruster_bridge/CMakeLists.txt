cmake_minimum_required(VERSION 3.8)
project(tuna_thruster_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(thruster_relay_node src/thruster_relay_node.cpp)
ament_target_dependencies(thruster_relay_node rclcpp std_msgs)


add_executable(joint_state_to_thrust_converter src/joint_state_to_thrust_converter.cpp)
ament_target_dependencies(joint_state_to_thrust_converter
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  thruster_relay_node 
  joint_state_to_thrust_converter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
