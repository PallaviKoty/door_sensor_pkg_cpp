/*
  Created by : Pallavi
  Created on : 28-08-2018
  Description: This file subscribes to the ROS2 topic and if there is an open door command over the topic, it sets the GPIO pin on the Pi
*/
#include <iostream>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "ros2_time/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/executor.hpp"
#include <sstream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "door_sensor_pkg_cpp/msg/command.hpp"
#include <wiringPi.h>
using namespace std;

using std::placeholders::_1;

#define GPIO_OUTPIN 0

class RMFAGVSubscriber : public rclcpp::Node
{
public:
  int timerFlag= 0;
  int timeoutPeriod = 15;
  int cyclePeriodms = 200;
  RMFAGVSubscriber()
  : Node("rmf_agv_subscriber")
  {
    gpiosetup();
    subscription_ = this->create_subscription<door_sensor_pkg_cpp::msg::Command>("agv_door_command_topic", std::bind(&RMFAGVSubscriber::agv_door_command_topic_callback, this, _1));
    sub_timer_ = this->create_wall_timer(500ms, std::bind(&RMFAGVSubscriber::sub_timer_callback, this));
  }

private:
  // This functions sends a HIGH LOW pulse to the GPIO pin
  void gpiosetpin()
  {
    digitalWrite(GPIO_OUTPIN, HIGH);
    RCLCPP_INFO(this->get_logger(), "LED Pin set")
    std::this_thread::sleep_for(std::chrono::milliseconds(cyclePeriodms));
    digitalWrite(GPIO_OUTPIN,LOW);
    RCLCPP_INFO(this->get_logger(), "LED Pin reset")
    std::this_thread::sleep_for(std::chrono::milliseconds(cyclePeriodms));
  }
  
  //This function resets the GPIO pin when nothing is published on the ROS2 topic even after timeout
  void gpioresetpin()
  {
    digitalWrite(GPIO_OUTPIN, LOW);
    RCLCPP_INFO(this->get_logger(), "LED Pin reset")
  }
  //This function is called every 500ms which checks for timeout and sets/resets the GPIO pins
  void sub_timer_callback()
  {
    if(count < countervalue)
    {
      //timerflag is reset whenever any message is published over ROS2 topic
      if(timerFlag ==1)
      {
        timerFlag = 0;
        count = 0;
      }
      gpiosetpin();
      count++;
    }
    else if(count == countervalue)
    {
      RCLCPP_INFO(this->get_logger(), "Timeout");
      gpioresetpin();
    }
    else 
    {
      //RCLCPP_INFO(this->get_logger(), "Waiting for subscription");
    }
  }
  // Initial wiring setup needs to done before using wiringPi functions/APIs
  void gpiosetup()
  {
    
    if(wiringPiSetup() == -1)
    {
     RCLCPP_INFO(this->get_logger(), "setup wiringPi failed")
    }
    pinMode(GPIO_OUTPIN, OUTPUT);
  }
  
  // This function is called when there is any message is published over the ROS2 topic
  void agv_door_command_topic_callback(const door_sensor_pkg_cpp::msg::Command::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->signalcommand);
    timerFlag = 1;
    count = 0;
    
    gpiosetup();

  }
  rclcpp::Subscription<door_sensor_pkg_cpp::msg::Command>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr sub_timer_;
  int countervalue = (timeoutPeriod*1000/2)/cyclePeriodms;
  int count = countervalue+1;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RMFAGVSubscriber>()); 
  rclcpp::shutdown();
  return 0;
}