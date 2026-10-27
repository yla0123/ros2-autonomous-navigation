cmake_minimum_required(VERSION 3.20)
project(my_robot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# =============================================
add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
    nav_msgs::nav_msgs
)

add_executable(lidar_node 
    src/lidar_node.cpp
    src/astar_planner.cpp)
target_link_libraries(lidar_node
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
    nav_msgs::nav_msgs
)
target_include_directories(lidar_node
  PRIVATE
  include
)

add_executable(path_follower_node src/path_follower_node.cpp)
target_link_libraries(path_follower_node
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    std_msgs::std_msgs
)

add_executable(evaluation_node src/evaluation_node.cpp)
target_link_libraries(evaluation_node
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    std_msgs::std_msgs
)

install(
    TARGETS odometry_node lidar_node path_follower_node evaluation_node
    DESTINATION lib/${PROJECT_NAME}
)

install(    
    DIRECTORY config launch worlds
    DESTINATION share/${PROJECT_NAME}
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
