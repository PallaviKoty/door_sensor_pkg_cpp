/*
  Created by : Pallavi
  Created on : 26-09-2018
  Description: This is the format any client application using the dry contact sensor wrap need to follow
*/

#include "rclcpp/rclcpp.hpp"
#include "door_sensor_pkg_cpp/DryContactSensorWrap.hpp"

int main(int argc, char *argv[])
{
  call_door_dry_contact_sensor_wrap(argc, argv);
  return 0;
}