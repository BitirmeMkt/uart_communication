#include "uart_communication/uart_communication.hpp"

using namespace std::chrono_literals;

namespace uart_communication
{

  UartCommunication::UartCommunication(const rclcpp::NodeOptions &options)
      : Node("uart_communication", options)
  {
    RCLCPP_INFO(this->get_logger(), "UART Communication Node has been started.");

    if (read_parameters())
    {
      RCLCPP_INFO(this->get_logger(), "Parameters has been read");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Parameters has not been read");
    }

    // steering_angle_subscriber_.subscribe(this, steering_angle_topic_.c_str());
    // throttle_subscriber_.subscribe(this, throttle_topic_.c_str());
    // sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10000), steering_angle_subscriber_, throttle_subscriber_);
    // sync_->registerCallback(std::bind(&UartCommunication::receiver_callback, this, std::placeholders::_1, std::placeholders::_2));

    steering_angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        steering_angle_topic_.c_str(), 10, std::bind(&UartCommunication::steering_callback, this, std::placeholders::_1));

    throttle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        throttle_topic_.c_str(), 10, std::bind(&UartCommunication::throttle_callback, this, std::placeholders::_1));

    speed_publisher_ = this->create_publisher<std_msgs::msg::Float32>(speed_publisher_topic_.c_str(), 10);

    stm = std::make_unique<serial::Serial>(serial_port_, serial_baudrate_, serial::Timeout::simpleTimeout(1000));

    if (stm->isOpen())
    {
      RCLCPP_INFO(this->get_logger(), "STM seri portu acildi: %s", stm->getPort().c_str());
      sender_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(sender_timer_value_), std::bind(&UartCommunication::sender_timer_callback, this));
      // receiver_timer_ = this->create_wall_timer(
      //     std::chrono::milliseconds(receiver_timer_value_), std::bind(&UartCommunication::receiver_timer_callback, this));
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", stm->getPort().c_str());
    }
  }

  bool UartCommunication::read_parameters()
  {
    this->declare_parameter<std::string>("serial_port", "/dev/default");
    this->declare_parameter<int>("serial_baudrate", 0);
    this->declare_parameter<int>("sender_timer_value", 0);
    this->declare_parameter<int>("receiver_timer_value", 0);
    this->declare_parameter<std::string>("steering_angle_topic", "/default_steering_angle");
    this->declare_parameter<std::string>("throttle_topic", "/default_throttle");
    this->declare_parameter<std::string>("speed_publisher_topic", "/default_speed");

    serial_port_ = this->get_parameter("serial_port").as_string();
    serial_baudrate_ = this->get_parameter("serial_baudrate").as_int();
    sender_timer_value_ = this->get_parameter("sender_timer_value").as_int();
    receiver_timer_value_ = this->get_parameter("receiver_timer_value").as_int();
    steering_angle_topic_ = this->get_parameter("steering_angle_topic").as_string();
    throttle_topic_ = this->get_parameter("throttle_topic").as_string();
    speed_publisher_topic_ = this->get_parameter("speed_publisher_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "serial_port: %s ", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "serial_baudrate: %d ", serial_baudrate_);
    RCLCPP_INFO(this->get_logger(), "sender_timer_value: %d ", sender_timer_value_);
    RCLCPP_INFO(this->get_logger(), "receiver_timer_value: %d ", receiver_timer_value_);
    RCLCPP_INFO(this->get_logger(), "steering_angle_topic: %s ", steering_angle_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "throttle_topic: %s", throttle_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "speed_publisher_topic: %s", speed_publisher_topic_.c_str());

    return true;
  }

  void UartCommunication::steering_callback(const std_msgs::msg::Float32::ConstSharedPtr &steering_angle_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Steering callback triggered.");
    steering_angle = steering_angle_msg->data;
    steering_ready = true;
  }

  void UartCommunication::throttle_callback(const std_msgs::msg::Float32::ConstSharedPtr &throttle_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Throttle callback triggered.");
    throttle = throttle_msg->data;
    throttle_ready = true;
  }

  void UartCommunication::receiver_callback(const std_msgs::msg::Float32::ConstSharedPtr &steering_angle_msg, const std_msgs::msg::Float32::ConstSharedPtr &throttle_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Receiver callback triggered.");
    float steering_angle = steering_angle_msg->data;
    float throttle = throttle_msg->data;
    // Gelen verileri string formatına dönüştür ve send to stm fonksiyonuna gönder
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1) << steering_angle << "," << throttle << "\n";
    std::string message = ss.str();
    send_to_stm(message);
  }

  void UartCommunication::send_to_stm(const std::string &message)
  {
    try
    {
      stm->write(message);
      RCLCPP_INFO(this->get_logger(), "Sended to STM by send_to_stm function with message: %s", message.c_str());
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error: " << e.what() << std::endl;
    }
  }

  void UartCommunication::sender_timer_callback()
  {
    if (!steering_ready && !throttle_ready)
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for both steering and throttle data.");
      return;
    }
    if (!steering_ready && throttle_ready)
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for steering data.");
      return;
    }
    if (steering_ready && !throttle_ready)
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for throttle data.");
      return;
    }

    

    RCLCPP_INFO(this->get_logger(), "Timer callback triggered.");
    try
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(1) << steering_angle << "," << throttle << "\n";
      std::string message = ss.str();
      send_to_stm(message);
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error: " << e.what() << std::endl;
    }
  }

  void UartCommunication::receiver_timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Receiver timer callback triggered.");
    // 3. Cevap bekle
    std::string rx_line = stm->readline(100, "\n"); // STM sonunda '\n' gönderiyorsa
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
        // Send the throttle value to the speed publisher
        auto speed_msg = std_msgs::msg::Float32();
        speed_msg.data = throttle;
        speed_publisher_->publish(speed_msg);
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(), "Float'a çevirirken hata oluştu: %s", e.what());
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "STM'den veri alınamadı veya boş veri alındı.");
    }
  }

} // namespace uart_communication

RCLCPP_COMPONENTS_REGISTER_NODE(uart_communication::UartCommunication)
