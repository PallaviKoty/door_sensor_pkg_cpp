/*
  Created by : Pallavi
  Created on : 03-09-2018
  Description: This file creates a subscriber node that reads the door status
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"

using std::placeholders::_1;

class SensorStatusSubscriber : public rclcpp::Node
{
public:
  SensorStatusSubscriber()
  : Node("sensor_status_sub_sim")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int8>("sensor_status_topic", std::bind(&SensorStatusSubscriber::sensor_status_sub_topic_callback, this, _1));
  }

private:
  // When there is some message published on the ROS2 topic this function will subscribe and prints out the status of the door
  void sensor_status_sub_topic_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
    if(msg->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "Door is open");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Door is closed");
    }
   }
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStatusSubscriber>());
  rclcpp::shutdown();
  return 0;
}