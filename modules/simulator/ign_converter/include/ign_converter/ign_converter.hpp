#ifndef SIMULATOR__IGN_CONVERTER_HPP_
#define SIMULATOR__IGN_CONVERTER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "goal_selector_msgs/msg/catch.hpp"

namespace simulator
{

class IgnitionConverter : public rclcpp::Node
{
public:
  using Catch = goal_selector_msgs::msg::Catch;
  IgnitionConverter();

private:
  void sim_poses_callback(const geometry_msgs::msg::PoseStamped & sim_pose);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sim_poses_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;

  void catch_callback(const Catch & catch_msg);
  rclcpp::Subscription<Catch>::SharedPtr catch_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr catch_converted_pub_;

  void release_callback(const Catch & release_msg);
  rclcpp::Subscription<Catch>::SharedPtr release_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr release_converted_pub_;
};

}  // namespace simulator

#endif  // SIMULATOR__IGN_CONVERTER_HPP_
