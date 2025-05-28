cmake_minimum_required(VERSION 3.8)
project(uart_communication)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(serial REQUIRED)
find_package(message_filters REQUIRED)

add_library(uart_communication SHARED
  src/uart_communication.cpp)

target_include_directories(uart_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(uart_communication
  rclcpp
  std_msgs
  rclcpp_components
  serial
  message_filters)

rclcpp_components_register_nodes(uart_communication
  "uart_communication::UartCommunication")

install(TARGETS uart_communication
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
