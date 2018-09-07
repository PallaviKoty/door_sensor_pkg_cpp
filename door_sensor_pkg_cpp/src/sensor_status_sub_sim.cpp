/*
  Created by : Pallavi
  Created on : 03-09-2018
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
// #include "door_sensor_pkg_cpp/msg/command.hpp"

using std::placeholders::_1;

class SensorStatusSubscriber : public rclcpp::Node
{
public:
  SensorStatusSubscriber()
  : Node("sensor_status_sub_sim")
  {
    // subscription_ = this->create_subscription<std_msgs::msg::Int8>("agv_door_command_topic", std::bind(&RMFAGVSubscriber::agv_door_command_topic_callback, this, _1));
    subscription_ = this->create_subscription<std_msgs::msg::Int8>("sensor_status_topic", std::bind(&SensorStatusSubscriber::sensor_status_sub_topic_callback, this, _1));
  }

private:
  // void agv_door_command_topic_callback(const std_msgs::msg::Int8::SharedPtr msg)
  void sensor_status_sub_topic_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
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