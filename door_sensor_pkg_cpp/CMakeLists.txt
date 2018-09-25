cmake_minimum_required(VERSION 3.5)

project(door_sensor_pkg_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -lwiringPi -lwiringPiDev")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_time REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(wiringPi REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(door_sensor_pkg_cpp
  "msg/Command.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# include_directories(include)
include_directories(include ${WIRINGPI_INCLUDE_DIRS})


add_executable(agv_publisher src/AGV_Publisher.cpp)
ament_target_dependencies(agv_publisher rclcpp std_msgs ros2_time)
rosidl_target_interfaces(agv_publisher door_sensor_pkg_cpp "rosidl_typesupport_cpp")

add_executable(dry_contact_sensor_wrap src/DryContactSensorWrap.cpp)
# ament_target_dependencies(dry_contact_sensor_wrap rclcpp std_msgs ros2_time)
ament_target_dependencies(dry_contact_sensor_wrap rclcpp std_msgs ros2_time wiringPi)
rosidl_target_interfaces(dry_contact_sensor_wrap door_sensor_pkg_cpp "rosidl_typesupport_cpp")

add_executable(sensor_status_sub_sim src/sensor_status_sub_sim.cpp)
ament_target_dependencies(sensor_status_sub_sim rclcpp std_msgs)

install(TARGETS
  agv_publisher
  dry_contact_sensor_wrap
  sensor_status_sub_sim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
