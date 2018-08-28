/*
  Created by : Pallavi
  Created on : 28-08-2018
*/

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
// #include <wiringPi.h>

using namespace std::chrono_literals;

#define INPUT_PIN 1

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SensorStatusPublisher : public rclcpp::Node
{
public:
  SensorStatusPublisher()
  : Node("sensor_status_pub"), status_(0)
  {
    status_publisher_ = this->create_publisher<std_msgs::msg::Int8>("sensor_status_topic");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SensorStatusPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    // if(wiringPisetup() == -1)
    // {
    //   RCLCPP_INFO(this->get_logger(), "setup wiringPi failed")
    // }
    // RCLCPP_INFO(this->get_logger(), "Status Read Pin : GPIO %d(wiringPi pin)", INPUT_PIN)
    // pinMode(INPUT_PIN, INPUT)
    // if(digitalRead(INPUT_PIN) == HIGH)
    // {
    //   sensor_status.data = 1;
    // }
    // else
    // {
    //   sensor_status.data = 0;
    // }
    auto sensor_status = std_msgs::msg::Int8();
    if (status_ == 1)
    {
      sensor_status.data = status_;
      status_--;
    }
    else 
    {
      status_ =0;
      sensor_status.data = status_;
      status_++;
    }
    RCLCPP_INFO(this->get_logger(), "Sensor status: '%d'", sensor_status.data)
    status_publisher_->publish(sensor_status);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_publisher_;
  size_t status_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}