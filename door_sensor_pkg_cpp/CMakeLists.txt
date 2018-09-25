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


add_executable(agv_command_publish_simu src/AgvCommandPublishSimu.cpp)
ament_target_dependencies(agv_command_publish_simu rclcpp std_msgs ros2_time)
rosidl_target_interfaces(agv_command_publish_simu door_sensor_pkg_cpp "rosidl_typesupport_cpp")

add_executable(dry_contact_sensor_wrap src/DryContactSensorWrap.cpp)
# ament_target_dependencies(dry_contact_sensor_wrap rclcpp std_msgs ros2_time)
ament_target_dependencies(dry_contact_sensor_wrap rclcpp std_msgs ros2_time wiringPi)
rosidl_target_interfaces(dry_contact_sensor_wrap door_sensor_pkg_cpp "rosidl_typesupport_cpp")

add_executable(status_subscriber_simu src/StatusSubscriberSimu.cpp)
ament_target_dependencies(status_subscriber_simu rclcpp std_msgs)

install(TARGETS
  agv_command_publish_simu
  dry_contact_sensor_wrap
  status_subscriber_simu
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
