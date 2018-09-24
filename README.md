# door_sensor_pkg_cpp

This package is built on Ubuntu 18.04 and uses ROS Bouncy version.
This package offers the following features:
1. It listens for any `open door` command from the AGV and sends a high low pulse to the GPIO pins on the Pi. 
2. It reads the status of the door from the GPIO pins and publishes it over the ROS2 topic so that the AGV can subscribe to this topic to know the status of the door.

`door_sensor_pkg_cpp` has 4 nodes: 2 on the AGV and 2 on the SoC (Raspberry Pi). Here, the `agv_publisher` and `sensor_status_sub_sim` nodes are on the AGV while `rmf_agv_subscriber` and `sensor_status_pub` are on the Raspberry Pi.

For compiling the code, use the following command:
```colcon build --symlink-install --packages-select door_sensor_pkg_cpp --cmake-clean-cache```

To run the AGV Simulator (publisher of `door open` command):
```ros2 run door_sensor_pkg_cpp agv_publisher```
We need to give the command to open the door. 1- Open door and 0-close door

To run the rmf_agv_subscriber (fetches the command and writes data to the Pi - GPIO pin 17)
```ros2 run door_sensor_pkg_cpp rmf_agv_subscriber```

To read the data from the sensor, 
```ros2 run door_sensor_pkg_cpp sensor_status_pub```
