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

  catch_sub_ = this->create_subscription<Catch>(
    "/robot/control/catch", 10,
    std::bind(&IgnitionConverter::catch_callback, this, _1));
  catch_converted_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/robot/control/catch_converted", 10);
  
  release_sub_ = this->create_subscription<Catch>(
    "/robot/control/release", 10,
    std::bind(&IgnitionConverter::release_callback, this, _1));
  release_converted_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/robot/control/release_converted", 10);
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

void IgnitionConverter::catch_callback(const Catch & catch_msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IgnitionConverter::catch_callback()");

  auto string_msg = std_msgs::msg::String();
  // string_msg.header.stamp = rclcpp::Time();
  switch (catch_msg.object_type)
  {
  case Catch::COLUMN:
    string_msg.data = "COLUMN";
    break;
  case Catch::PLATFORM:
    string_msg.data = "PLATFORM";
    break;
  
  default:
    string_msg.data = "UNDEFINED";
    break;
  }
  catch_converted_pub_->publish(string_msg);
}

void IgnitionConverter::release_callback(const Catch & release_msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IgnitionConverter::release_callback()");

  auto string_msg = std_msgs::msg::String();
  // string_msg.header.stamp = rclcpp::Time();
  switch (release_msg.object_type)
  {
  case Catch::COLUMN:
    string_msg.data = "COLUMN";
    break;
  case Catch::PLATFORM:
    string_msg.data = "PLATFORM";
    break;

  default:
    string_msg.data = "UNDEFINED";
    break;
  }
  release_converted_pub_->publish(string_msg);
}

}  // namespace simulator