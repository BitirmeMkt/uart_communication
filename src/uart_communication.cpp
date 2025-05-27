#include "uart_communication/uart_communication.hpp"

using namespace std::chrono_literals;

namespace uart_communication
{

  UartCommunication::UartCommunication(const rclcpp::NodeOptions &options)
      : Node("uart_communication", options)
  {
    RCLCPP_INFO(this->get_logger(), "UART Communication Node has been started.");
    publisher_ = this->create_publisher<std_msgs::msg::String>("uart_out", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "uart_in", 10,
        std::bind(&UartCommunication::topic_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), std::bind(&UartCommunication::timer_callback, this));
  }

  void UartCommunication::topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
  }

  void UartCommunication::timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered.");
    try
    {
      // 1. Seri portu aç
      serial::Serial stm("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));
      if (stm.isOpen())
      {
        RCLCPP_INFO(this->get_logger(), "STM seri portu acildi: %s", stm.getPort().c_str());
        // 2. Mesaj oluştur ve gönder
        std::string tx_msg = "-15.2,30.0\n";
        stm.write(tx_msg);
        RCLCPP_INFO(this->get_logger(), "STM'ye veri gonderildi: %s", tx_msg.c_str());
        // 3. Cevap bekle
        std::string rx_line = stm.readline(100, "\n"); // STM sonunda '\n' gönderiyorsa
        if (!rx_line.empty())
        {
          RCLCPP_INFO(this->get_logger(), "STM'den gelen veri: %s", rx_line.c_str());

          try
          {
            // Satırın sonundaki '\n' veya olası '\r' karakterlerini temizle
            rx_line.erase(std::remove(rx_line.begin(), rx_line.end(), '\n'), rx_line.end());
            rx_line.erase(std::remove(rx_line.begin(), rx_line.end(), '\r'), rx_line.end());

            float throttle = std::stof(rx_line);
            RCLCPP_INFO(this->get_logger(), "Throttle: %.2f", throttle);
          }
          catch (const std::exception &e)
          {
            RCLCPP_WARN(this->get_logger(), "Float'a çevirirken hata oluştu: %s", e.what());
          }
        }

        // 100ms bekle
      }
      else
      {
        std::cerr << "Port açılamadı.\n";
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << "Hata: " << e.what() << std::endl;
    }
  }

} // namespace uart_communication

RCLCPP_COMPONENTS_REGISTER_NODE(uart_communication::UartCommunication)
