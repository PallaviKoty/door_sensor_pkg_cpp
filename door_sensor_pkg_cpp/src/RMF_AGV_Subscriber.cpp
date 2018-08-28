/*
  Created by : Pallavi
  Created on : 28-08-2018
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
// #include <wiringPi.h>

using std::placeholders::_1;

#define GPIO_OUTPIN 0

class RMFAGVSubscriber : public rclcpp::Node
{
public:
  RMFAGVSubscriber()
  : Node("rmf_agv_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int8>(
      "agv_door_command_topic", std::bind(&RMFAGVSubscriber::agv_door_command_topic_callback, this, _1));
  }

private:
  void agv_door_command_topic_callback(const std_msgs::msg::Int8::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->data);
    if (msg->data == 1)
    {
      RCLCPP_INFO(this->get_logger(), "Writing %d to GPIO pins", msg->data);
      // https://bob.cs.sonoma.edu/IntroCompOrg-RPi/sec-cgpio.html
      // if(wiringPiSetup() == -1)
      // {
      //   RCLCPP_INFO(this->get_logger(), "setup wiringPi failed")
      // }
      // RCLCPP_INFO(this->get_logger(), "linker LedPin : GPIO %d(wiringPi pin)", GPIO_OUTPIN)
      // pinMode(GPIO_OUTPIN, OUTPUT);
      // digitalWrite(LedPin, LOW);

    }
  }
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RMFAGVSubscriber>());
  rclcpp::shutdown();
  return 0;
}