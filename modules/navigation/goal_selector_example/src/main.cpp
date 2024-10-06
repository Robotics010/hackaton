#include <memory>

#include "goal_selector/goal_selector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation::GoalSelector>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
