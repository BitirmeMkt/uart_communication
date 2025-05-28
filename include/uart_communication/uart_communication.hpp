#ifndef UART_COMMUNICATION_HPP_
#define UART_COMMUNICATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <serial/serial.h>
#include "std_msgs/msg/float32.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace uart_communication
{
class UartCommunication : public rclcpp::Node
{
public:
  explicit UartCommunication(const rclcpp::NodeOptions & options);

private:

  void sender_timer_callback();

  void receiver_timer_callback();

  bool read_parameters();

  void receiver_callback(const std_msgs::msg::Float32::ConstSharedPtr &steering_angle_msg,
                         const std_msgs::msg::Float32::ConstSharedPtr &throttle_msg);

  void send_to_stm(const std::string &message);
  
  rclcpp::TimerBase::SharedPtr sender_timer_;
  rclcpp::TimerBase::SharedPtr receiver_timer_;

  std::unique_ptr<serial::Serial> stm;

  std::string serial_port_;

  int serial_baudrate_;

  int sender_timer_value_;

  int receiver_timer_value_;

  std::string steering_angle_topic_;

  std::string throttle_topic_;

  std::string speed_publisher_topic_;

  message_filters::Subscriber<std_msgs::msg::Float32> steering_angle_subscriber_;
  message_filters::Subscriber<std_msgs::msg::Float32> throttle_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_publisher_;;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<std_msgs::msg::Float32, std_msgs::msg::Float32>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

};

}  // namespace uart_component

#endif  // UART_COMMUNICATION_HPP_
