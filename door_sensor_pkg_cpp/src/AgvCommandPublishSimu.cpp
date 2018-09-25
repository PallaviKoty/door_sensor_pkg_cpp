/*
  Created by : Pallavi
  Created on : 28-08-2018
  Description: This file creates an agv_publisher node. This is a simulator script that simulates the AGV publishings to open the door.

*/
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "door_sensor_pkg_cpp/msg/command.hpp"

using namespace std;

using namespace std::chrono_literals;

/* This creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class AGVPublisher : public rclcpp::Node
{
public:
  AGVPublisher()
  : Node("agv_publisher"),count_(0)
  {
    // command_publisher_ = this->create_publisher<std_msgs::msg::Int8>("agv_door_command_topic");
    command_publisher_ = this->create_publisher<door_sensor_pkg_cpp::msg::Command>("door_command_topic");
    publish_timer_ = this->create_wall_timer(
      1000ms, std::bind(&AGVPublisher::publish_timer_callback, this));
  }

private:
  // This callback function is called every 1 sec and HIGH is published over "agv_door_command_topic" topic. 
  void publish_timer_callback()
  {
    // auto message = std_msgs::msg::Int8();
    auto message = door_sensor_pkg_cpp::msg::Command();
    // cout << "Hey AGV here, I want the door to be opened if I send HIGH" << endl;
    // cin >> count_;
    // if ((count_!=1))
    // {
    //   RCLCPP_INFO(this->get_logger(), "Bad choice : Please enter 1 to open the door")
    //   cin >> count_;   
    // }
    message.signalcommand = 0;
    if(count_ < 100)
    {
    message.signalcommand = 1;
    count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d', count : '%d'", message.signalcommand, count_)
    command_publisher_->publish(message);
    }
  }
  rclcpp::TimerBase::SharedPtr publish_timer_;
  // rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr command_publisher_;
  rclcpp::Publisher<door_sensor_pkg_cpp::msg::Command>::SharedPtr command_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AGVPublisher>());
  rclcpp::shutdown();
  return 0;
}