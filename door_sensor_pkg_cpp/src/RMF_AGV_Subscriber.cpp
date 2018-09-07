/*
  Created by : Pallavi
  Created on : 28-08-2018
*/
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "ros2_time/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/executor.hpp"
#include <sstream>
#include <chrono>
// #include "std_msgs/msg/int8.hpp"
#include "door_sensor_pkg_cpp/msg/command.hpp"
// #include <wiringPi.h>
using namespace std;

using std::placeholders::_1;

#define GPIO_OUTPIN 0

int timerFlag= 0;

class RMFAGVSubscriber : public rclcpp::Node
{
public:
  RMFAGVSubscriber()
  : Node("rmf_agv_subscriber")
  {
    // subscription_ = this->create_subscription<std_msgs::msg::Int8>("agv_door_command_topic", std::bind(&RMFAGVSubscriber::agv_door_command_topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Value of subscription_ before create_subscription : %d", subscription_);
    subscription_ = this->create_subscription<door_sensor_pkg_cpp::msg::Command>("agv_door_command_topic", std::bind(&RMFAGVSubscriber::agv_door_command_topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Value of subscription_ after create_subscription : %d", subscription_);
    sub_timer_ = this->create_wall_timer(
      15000ms, std::bind(&RMFAGVSubscriber::sub_timer_callback, this));
  }

private:

  void sub_timer_callback()
  {
    if(timerFlag == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Timeout already");
      gpioresetpin();
    }
    else if (timerFlag == 1)
    {
      RCLCPP_INFO(this->get_logger(), "Not timeout");
      timerFlag = 0;
    }

  }
  // void agv_door_command_topic_callback(const std_msgs::msg::Int8::SharedPtr msg)
  void agv_door_command_topic_callback(const door_sensor_pkg_cpp::msg::Command::SharedPtr msg)
  {
    timerFlag = 1;
    // cout << "Received a signal just now : " << exectime.toSec() <<"."<< exectime.toNSec() <<endl;
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->signalcommand);
    gpiosetup(msg);

  }

  void gpiosetpin()
  {
    // digitalWrite(GPIO_OUTPIN, HIGH);
    RCLCPP_INFO(this->get_logger(), "LED Pin Set")
  }

  void gpioresetpin()
  {
    // digitalWrite(GPIO_OUTPIN, LOW);
    RCLCPP_INFO(this->get_logger(), "LED Pin reset")
  }

  void gpiosetup(const door_sensor_pkg_cpp::msg::Command::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Writing %d to GPIO pins", msg->signalcommand);
    // https://bob.cs.sonoma.edu/IntroCompOrg-RPi/sec-cgpio.html
    
    // if(wiringPiSetup() == -1)
    // {
    //   RCLCPP_INFO(this->get_logger(), "setup wiringPi failed")
    // }
    // pinMode(GPIO_OUTPIN, OUTPUT);
    if(msg->signalcommand == 1)
    {
      RCLCPP_INFO(this->get_logger(), "linker LedPin : GPIO %d(wiringPi pin)", GPIO_OUTPIN)
      gpiosetpin();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "linker LedPin : GPIO %d(wiringPi pin)", GPIO_OUTPIN)
      gpioresetpin();

    }
  }
  rclcpp::Subscription<door_sensor_pkg_cpp::msg::Command>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr sub_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RMFAGVSubscriber>()); 
  rclcpp::shutdown();
  return 0;
}