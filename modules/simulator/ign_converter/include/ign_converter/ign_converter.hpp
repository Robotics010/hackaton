#ifndef SIMULATOR__IGN_CONVERTER_HPP_
#define SIMULATOR__IGN_CONVERTER_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace simulator
{

class IgnitionConverter : public rclcpp::Node
{
public:
  IgnitionConverter();

private:
  void sim_poses_callback(const geometry_msgs::msg::PoseStamped & sim_pose);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sim_poses_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;
};

}  // namespace simulator

#endif  // SIMULATOR__IGN_CONVERTER_HPP_
