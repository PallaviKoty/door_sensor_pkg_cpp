cmake_minimum_required(VERSION 3.5)
project(door_sensor_pkg_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(agv_publisher src/AGV_Publisher.cpp)
ament_target_dependencies(agv_publisher rclcpp std_msgs)

add_executable(rmf_agv_subscriber src/RMF_AGV_Subscriber.cpp)
ament_target_dependencies(rmf_agv_subscriber rclcpp std_msgs)

add_executable(sensor_status_pub src/sensor_status_pub.cpp)
ament_target_dependencies(sensor_status_pub rclcpp std_msgs)

install(TARGETS
  agv_publisher
  rmf_agv_subscriber
  sensor_status_pub
  DESTINATION lib/${PROJECT_NAME}
)


#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
#  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
