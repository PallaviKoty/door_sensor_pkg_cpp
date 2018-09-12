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
  : Node("sensor_status_pub")
  {
    // if(wiringPiSetup() == -1)
    // {
    //   RCLCPP_INFO(this->get_logger(), "setup wiringPi failed");
    // }
    // pinMode(INPUT_PIN, INPUT);
    // pullUpDnControl (INPUT_PIN, PUD_UP) ;
    status_publisher_ = this->create_publisher<std_msgs::msg::Int8>("sensor_status_topic");
    timer_ = this->create_wall_timer(1000ms, std::bind(&SensorStatusPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto sensor_status = std_msgs::msg::Int8();
    // if(digitalRead(INPUT_PIN) == LOW)
    // {
    //   RCLCPP_INFO(this->get_logger(), "Read HIGH, the door is open") //LOW is pushed
    //   sensor_status.data = 1;
      
    // }
    // else
    // {
    //   RCLCPP_INFO(this->get_logger(), "Read LOW, the door is closed") //HIGH is released
    //   sensor_status.data = 0;
    // }

    RCLCPP_INFO(this->get_logger(), "Publishing sensor status: '%d'", sensor_status.data)
    status_publisher_->publish(sensor_status);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}