cmake_minimum_required(VERSION 3.20)
project(my_robot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# =============================================
add_executable(robot_node src/robot_node.cpp)
target_link_libraries(robot_node
    rclcpp::rclcpp
    std_msgs::std_msgs
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
)

add_executable(status_listener src/status_listener.cpp)
target_link_libraries(status_listener
    rclcpp::rclcpp
    std_msgs::std_msgs
    geometry_msgs::geometry_msgs
)

add_executable(imu_listener src/imu_listener.cpp)
target_link_libraries(imu_listener
    rclcpp::rclcpp
    std_msgs::std_msgs
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
)

install(
    TARGETS robot_node
    TARGETS status_listener
    TARGETS imu_listener
    DESTINATION lib/${PROJECT_NAME}
)
# =============================================
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

ament_package()
