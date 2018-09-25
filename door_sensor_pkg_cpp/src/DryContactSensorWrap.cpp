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
#include "std_msgs/msg/int8.hpp"
#include <wiringPi.h>
using namespace std;

using std::placeholders::_1;

#define GPIO_OUTPIN 0
#define INPUT_PIN 1

class DryContactSensorWrap : public rclcpp::Node
{
public:
  int timer_flag= 0;
  int timeout_period_sec = 15;
  int half_cycle_period_ms = 200;
  DryContactSensorWrap()
  : Node("dry_contact_sensor_wrap")
  {
    gpiosetup();
    command_subscription_ = this->create_subscription<door_sensor_pkg_cpp::msg::Command>("door_command_topic", std::bind(&DryContactSensorWrap::door_command_topic_callback, this, _1));
    gpio_write_timer_ = this->create_wall_timer(500ms, std::bind(&DryContactSensorWrap::gpio_write_timer_callback, this));
    door_status_publisher_ = this->create_publisher<std_msgs::msg::Int8>("door_sensor_status_topic");
    status_publish_timer_ = this->create_wall_timer(1000ms, std::bind(&DryContactSensorWrap::status_publish_timer_callback, this));
  }

private:

  // Initial wiring setup needs to done before using wiringPi functions/APIs
  void gpiosetup()
  {
    RCLCPP_INFO(this->get_logger(), "setup wiringPi")
    if(wiringPiSetup() == -1)
    {
     RCLCPP_INFO(this->get_logger(), "setup wiringPi failed")
    }
    pinMode(GPIO_OUTPIN, OUTPUT);
  }

  // This function is called when there is any message is published over the ROS2 topic
  void door_command_topic_callback(const door_sensor_pkg_cpp::msg::Command::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->signalcommand);
    timer_flag = 1;
    count = 0;
  }

  // This functions sends a HIGH LOW pulse to the GPIO pin
  void gpiosetpin()
  {
    digitalWrite(GPIO_OUTPIN, HIGH);
    RCLCPP_INFO(this->get_logger(), "LED Pin set")
    std::this_thread::sleep_for(std::chrono::milliseconds(half_cycle_period_ms));
    digitalWrite(GPIO_OUTPIN,LOW);
    RCLCPP_INFO(this->get_logger(), "LED Pin reset")
    std::this_thread::sleep_for(std::chrono::milliseconds(half_cycle_period_ms));
  }
  
  //This function resets the GPIO pin when nothing is published on the ROS2 topic even after timeout
  void gpioresetpin()
  {
    digitalWrite(GPIO_OUTPIN, LOW);
    RCLCPP_INFO(this->get_logger(), "LED Pin reset")
  }

  //This function is called every 500ms which checks for timeout and sets/resets the GPIO pins
  void gpio_write_timer_callback()
  {
    if(count < countervalue)
    {
      //timer_flag is reset whenever any message is published over ROS2 topic
      if(timer_flag ==1)
      {
        timer_flag = 0;
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

  void status_publish_timer_callback()
  {
    auto door_sensor_status = std_msgs::msg::Int8();
    if(digitalRead(INPUT_PIN) == LOW)
    {
      RCLCPP_INFO(this->get_logger(), "The door is open") //LOW is pushed
      door_sensor_status.data = 1;
      
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "The door is closed") //HIGH is released
      door_sensor_status.data = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing sensor status: '%d'", door_sensor_status.data)
    door_status_publisher_->publish(door_sensor_status);
  }

  rclcpp::Subscription<door_sensor_pkg_cpp::msg::Command>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr gpio_write_timer_;
  rclcpp::TimerBase::SharedPtr status_publish_timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr door_status_publisher_;
  int countervalue = (timeout_period_sec*1000/2)/half_cycle_period_ms;
  int count = countervalue+1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DryContactSensorWrap>()); 
  rclcpp::shutdown();
  return 0;
}