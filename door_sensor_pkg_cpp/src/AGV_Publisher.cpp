/*
  Created by : Pallavi
  Created on : 28-08-2018
*/
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "door_sensor_pkg_cpp/msg/command.hpp"

using namespace std;

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AGVPublisher : public rclcpp::Node
{
public:
  AGVPublisher()
  : Node("agv_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<door_sensor_pkg_cpp::msg::Command>("agv_door_command_topic");
    timer_ = this->create_wall_timer(500ms, std::bind(&AGVPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = door_sensor_pkg_cpp::msg::Command();
    message.signalcommand=0;
    if(count_<100) 
    {
    message.signalcommand =1;
    count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.signalcommand)
    publisher_->publish(message);
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<door_sensor_pkg_cpp::msg::Command>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AGVPublisher>());
  rclcpp::shutdown();
  return 0;
}