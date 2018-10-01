/*
  Created by : Pallavi
  Created on : 25-09-2018
  Description: This file subscribes to the ROS2 topic and if there is an open door command over the topic, it sets the GPIO pin on the Pi
*/

#ifndef DRYCONTACTSENSORWRAP_HPP
#define DRYCONTACTSENSORWRAP_HPP

#include <iostream>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/executor.hpp"
#include <sstream>
#include <memory>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "door_sensor_pkg_cpp/msg/command.hpp"
#include "std_msgs/msg/int8.hpp"

#ifndef WIRINGPI
// #define WIRINGPI
#endif

#ifdef WIRINGPI
#include <wiringPi.h>
#endif

using namespace std;

using std::placeholders::_1;

#define GPIO_OUTPIN 0
#define INPUT_PIN 1

class DryContactSensorWrap : public rclcpp::Node
{
  public:
    DryContactSensorWrap();
    void param_initialize();
    virtual ~DryContactSensorWrap();

  private:
    bool timer_flag = false;
    int timeout_period_sec = 0;
    int half_cycle_period_ms = 0;
    int count, countervalue = 0;
    bool timeout = false;
    rclcpp::Subscription<door_sensor_pkg_cpp::msg::Command>::SharedPtr command_subscription_;
    rclcpp::TimerBase::SharedPtr gpio_write_timer_;
    rclcpp::TimerBase::SharedPtr status_publish_timer_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr door_status_publisher_;
  
    //Subscribe to the topic for commands
    void door_command_topic_callback(const door_sensor_pkg_cpp::msg::Command::SharedPtr msg);
    void gpio_write_timer_callback();

    //WiringPi Libraries
    void gpiosetup();
    void gpiosetpin();
    void gpioresetpin();

    //Publish door status to the AGV
    void status_publish_timer_callback();
};

void call_door_contact_sensor_wrap(int argc, char *argv[]);

#endif