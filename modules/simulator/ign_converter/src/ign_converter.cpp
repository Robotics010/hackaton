#include "ign_converter/ign_converter.hpp"

using std::placeholders::_1;

namespace simulator
{

IgnitionConverter::IgnitionConverter() : Node("ignition_converter")
{
  sim_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/computer/perception/poses_from_sim", 10,
    std::bind(&IgnitionConverter::sim_poses_callback, this, _1));
  robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/computer/perception/robot/pose", 10);
}

void IgnitionConverter::sim_poses_callback(const geometry_msgs::msg::PoseStamped & sim_pose)
{
  if (sim_pose.header.frame_id == "car_world") {
    geometry_msgs::msg::PoseWithCovarianceStamped robot_pose;
    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = sim_pose.header.stamp;
    robot_pose.pose.pose = sim_pose.pose;
    robot_pose_pub_->publish(robot_pose);
  }
}

}  // namespace simulator