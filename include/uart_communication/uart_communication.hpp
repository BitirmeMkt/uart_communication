#ifndef UART_COMMUNICATION_HPP_
#define UART_COMMUNICATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <serial/serial.h>

namespace uart_communication
{
class UartCommunication : public rclcpp::Node
{
public:
  explicit UartCommunication(const rclcpp::NodeOptions & options);

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);

  void timer_callback();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uart_component

#endif  // UART_COMMUNICATION_HPP_
