# door_sensor_pkg_cpp
For compiling the code, use the following command:
```colcon build --symlink-install --packages-select door_sensor_pkg_cpp --cmake-clean-cache```

To run the AGV Simulator (publisher of `door open` command):
```ros2 run door_sensor_pkg_cpp agv_publisher```
We need to give the command to open the door. 1- Open door and 0-close door

To run the rmf_agv_subscriber (fetches the command and writes data to the Pi - GPIO pin 17)
```ros2 run door_sensor_pkg_cpp rmf_agv_subscriber```

To read the data from the sensor, 
```ros2 run door_sensor_pkg_cpp sesnor_status_pub```
