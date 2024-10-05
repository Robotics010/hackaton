#include "ign_converter/ign_converter.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simulator::IgnitionConverter>());
  rclcpp::shutdown();
  return 0;
}