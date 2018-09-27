
# Dry Contact Sensor ROS2 wrapper (Assa Abloy Door Sensor)

The AGV send the `open door` command to the ROS2 wrapper. The wrapper reads the command from the AGV and manipulates the GPIO pins on the Raspberry Pi (The Pi actuates the actions on dry contact sensor which closes/opens the door)

### Getting Started

This package offers the following features:
1. It listens for any `open door` command from the AGV and sends a high low pulse to the GPIO pins on the Pi. 
2. It reads the status of the door from the GPIO pins and publishes it over the ROS2 topic so that the AGV can subscribe to this topic to know the status of the door.
`door_sensor_pkg_cpp` has 3 nodes: 2 on the AGV and 1 on the SoC (Raspberry Pi). Here, the `agv_command_publish_simu` and `status_subscriber_simu` nodes are on the AGV while `dry_contact_sensor_wrap` are on the Raspberry Pi. The client need to refer `client` as reference to make use of `dry_contact_sensor_wrap`.

### Prerequisites

This package is a ROS2 bouncy package running on Ubuntu. To run this package, you will need the following:
1. Raspberry Pi 3B+ with Ubuntu/Raspbian installed in it
2. WiringPi library for Raspberry Pi 3B+
3. Gertboard (for testing)

### Installing

WiringPi 2.46 library need to be installed to manipulate GPIO pins on the Raspberry Pi.
To install WiringPi do the following:
```
cd /tmp
wget https://unicorn.drogon.net/wiringpi-2.46-1.deb
sudo dpkg -i wiringpi-2.46-1.deb
```
After this, clone `door_sensor_pkg_cpp` repository to your ROS2 workspace
```
cd my_ros2_ws/src
git clone https://github.com/PallaviKoty/door_sensor_pkg_cpp.git
```
## Running the code

For compiling the code, use the following command:
```
colcon build --symlink-install --packages-select door_sensor_pkg_cpp --cmake-clean-cache
```

Source the setup files using the command,
```
source install/setup.*
```

To run the AGV Simulator (publisher of `door open` command):
```
ros2 run door_sensor_pkg_cpp agv_command_publish_simu
```

The `dry_contact_sensor_wrap` file does the following:
1. Fetches the command from the AGV and writes data to the Pi - GPIO pin 17
2. Reads the status of the door from the GPIO pin and publishes it to the AGV 

This wrap is used in `client.cpp` file.

To run the `client`,
```
ros2 run door_sensor_pkg_cpp client
```

To run the AGV Status subscriber, 
```
ros2 run door_sensor_pkg_cpp status_subscriber_simu
```
