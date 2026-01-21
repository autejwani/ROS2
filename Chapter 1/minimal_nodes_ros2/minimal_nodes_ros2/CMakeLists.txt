cmake_minimum_required(VERSION 3.8)
project(minimal_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executables
add_executable(minimal_publisher src/minimal_publisher.cpp)
add_executable(sleepy_minimal_publisher src/sleepy_minimal_publisher.cpp)
add_executable(minimal_subscriber src/minimal_subscriber.cpp)
add_executable(minimal_simulator src/minimal_simulator.cpp)
add_executable(minimal_controller src/minimal_controller.cpp)

# Specify libraries to link each executable against
target_link_libraries(minimal_publisher PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

target_link_libraries(sleepy_minimal_publisher PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

target_link_libraries(minimal_subscriber PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

target_link_libraries(minimal_simulator PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

target_link_libraries(minimal_controller PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# Install executables
install(TARGETS
  minimal_publisher
  sleepy_minimal_publisher
  minimal_subscriber
  minimal_simulator
  minimal_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
