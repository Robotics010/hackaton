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
  void sim_blue_poses_callback(const geometry_msgs::msg::PoseStamped & sim_blue_pose);
  void sim_yellow_poses_callback(const geometry_msgs::msg::PoseStamped & sim_yellow_pose);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sim_blue_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sim_yellow_poses_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr blue_robot_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr yellow_robot_pose_pub_;

  void blue_catch_callback(const Catch & blue_catch_msg);
  rclcpp::Subscription<Catch>::SharedPtr blue_catch_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr blue_catch_converted_pub_;

  void blue_release_callback(const Catch & blue_release_msg);
  rclcpp::Subscription<Catch>::SharedPtr blue_release_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr blue_release_converted_pub_;

  void yellow_catch_callback(const Catch & yellow_catch_msg);
  rclcpp::Subscription<Catch>::SharedPtr yellow_catch_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yellow_catch_converted_pub_;

  void yellow_release_callback(const Catch & yellow_release_msg);
  rclcpp::Subscription<Catch>::SharedPtr yellow_release_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr yellow_release_converted_pub_;
};

}  // namespace simulator

#endif  // SIMULATOR__IGN_CONVERTER_HPP_
