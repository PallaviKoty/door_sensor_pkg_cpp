/*
  Created by : Pallavi
  Created on : 25-09-2018
  Description: This file subscribes to the ROS2 topic and if there is an open door command over the topic, it sets the GPIO pin on the Pi
*/

#include "door_sensor_pkg_cpp/DryContactSensorWrap.hpp"

using namespace std;

DryContactSensorWrap::DryContactSensorWrap() : Node("dry_contact_sensor_wrap")
{
#ifdef WIRINGPI
  gpiosetup();
#endif
  command_subscription_ = this->create_subscription<door_sensor_pkg_cpp::msg::Command>("door_command_topic", std::bind(&DryContactSensorWrap::door_command_topic_callback, this, _1));
  gpio_write_timer_ = this->create_wall_timer(500ms, std::bind(&DryContactSensorWrap::gpio_write_timer_callback, this));
  door_status_publisher_ = this->create_publisher<std_msgs::msg::Int8>("door_sensor_status_topic");
  status_publish_timer_ = this->create_wall_timer(1000ms, std::bind(&DryContactSensorWrap::status_publish_timer_callback, this));
}


void DryContactSensorWrap::param_initialize()
{
  this->get_parameter_or("timer_flag_param", timer_flag, false);
  this->get_parameter_or("timeout_period_sec_param", timeout_period_sec, 0);
  this->get_parameter_or("half_cycle_period_ms_param", half_cycle_period_ms, 0);
 
  countervalue = (timeout_period_sec * 500) / half_cycle_period_ms;
  count = countervalue + 1;
}

DryContactSensorWrap::~DryContactSensorWrap() {}

// This function is called when there is any message is published over the ROS2 topic
void DryContactSensorWrap::door_command_topic_callback(const door_sensor_pkg_cpp::msg::Command::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received: '%d'", msg->signalcommand);
  timer_flag = true;
  count = 0;
}

//This function is called every 500ms which checks for timeout and sets/resets the GPIO pins
void DryContactSensorWrap::gpio_write_timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "countervalue = %d", countervalue);
  RCLCPP_INFO(this->get_logger(), "count = %d", count);
  if (count < countervalue)
  {
    //timer_flag is reset whenever any message is published over ROS2 topic
    if (timer_flag)
    {
      timer_flag = false;
      count = 0;
    }
    gpiosetpin();
    count++;
  }
  else if (count == countervalue)
  {
    RCLCPP_INFO(this->get_logger(), "Timeout");
    timeout = true;
    gpioresetpin();
  }
  else
  {
    //RCLCPP_INFO(this->get_logger(), "Waiting for subscription");
  }
}

// Initial wiring setup needs to done before using wiringPi functions/APIs
void DryContactSensorWrap::gpiosetup()
{
  RCLCPP_INFO(this->get_logger(), "setup wiringPi");
#ifdef WIRINGPI
  if (wiringPiSetup() == -1)
  {
    RCLCPP_INFO(this->get_logger(), "setup wiringPi failed");
  }
  pinMode(GPIO_OUTPIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
  pullUpDnControl(INPUT_PIN, PUD_UP);
#endif
}

// This functions sends a HIGH LOW pulse to the GPIO pin
void DryContactSensorWrap::gpiosetpin()
{
#ifdef WIRINGPI
  digitalWrite(GPIO_OUTPIN, HIGH);
#endif
  RCLCPP_INFO(this->get_logger(), "LED Pin set");
  std::this_thread::sleep_for(std::chrono::milliseconds(half_cycle_period_ms));
#ifdef WIRINGPI
  digitalWrite(GPIO_OUTPIN, LOW);
#endif
  RCLCPP_INFO(this->get_logger(), "LED Pin reset");
  std::this_thread::sleep_for(std::chrono::milliseconds(half_cycle_period_ms));
}

//This function resets the GPIO pin when nothing is published on the ROS2 topic even after timeout
void DryContactSensorWrap::gpioresetpin()
{
#ifdef WIRINGPI
  digitalWrite(GPIO_OUTPIN, LOW);
#endif
  RCLCPP_INFO(this->get_logger(), "LED Pin reset");
}

void DryContactSensorWrap::status_publish_timer_callback()
{
  auto door_sensor_status = std_msgs::msg::Int8();
#ifdef WIRINGPI
  if (digitalRead(INPUT_PIN) == LOW && (!timeout) && (!(count > countervalue)))
  {
    RCLCPP_INFO(this->get_logger(), "The door is open"); //LOW is pushed
    door_sensor_status.data = 1;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "The door is closed"); //HIGH is released
    door_sensor_status.data = 0;
  }
#endif
  RCLCPP_INFO(this->get_logger(), "Publishing sensor status: '%d'", door_sensor_status.data);
  door_status_publisher_->publish(door_sensor_status);
}

void call_door_dry_contact_sensor_wrap(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DryContactSensorWrap>();
  node->param_initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
