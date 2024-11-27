#include "ign_converter/ign_converter.hpp"

using std::placeholders::_1;

namespace simulator
{

IgnitionConverter::IgnitionConverter() : Node("ignition_converter")
{
  sim_blue_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/simulator/ground_truth/blue_poses_from_sim", 10,
    std::bind(&IgnitionConverter::sim_blue_poses_callback, this, _1));
  sim_yellow_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/simulator/ground_truth/yellow_poses_from_sim", 10,
    std::bind(&IgnitionConverter::sim_yellow_poses_callback, this, _1));
  blue_robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/computer/perception/blue_robot/pose", 10);
  yellow_robot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/computer/perception/yellow_robot/pose", 10);

  blue_catch_sub_ = this->create_subscription<Catch>(
    "/blue_robot/control/catch", 10,
    std::bind(&IgnitionConverter::blue_catch_callback, this, _1));
  blue_catch_converted_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/blue_robot/control/catch_converted", 10);
  
  blue_release_sub_ = this->create_subscription<Catch>(
    "/blue_robot/control/release", 10,
    std::bind(&IgnitionConverter::blue_release_callback, this, _1));
  blue_release_converted_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/blue_robot/control/release_converted", 10);

  yellow_catch_sub_ = this->create_subscription<Catch>(
    "/yellow_robot/control/catch", 10,
    std::bind(&IgnitionConverter::yellow_catch_callback, this, _1));
  yellow_catch_converted_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/yellow_robot/control/catch_converted", 10);
  
  yellow_release_sub_ = this->create_subscription<Catch>(
    "/yellow_robot/control/release", 10,
    std::bind(&IgnitionConverter::yellow_release_callback, this, _1));
  yellow_release_converted_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/yellow_robot/control/release_converted", 10);
}

void IgnitionConverter::sim_blue_poses_callback(const geometry_msgs::msg::PoseStamped & sim_blue_pose)
{
  if (sim_blue_pose.header.frame_id == "car_world") {
    geometry_msgs::msg::PoseWithCovarianceStamped blue_robot_pose;
    blue_robot_pose.header.frame_id = "map";
    blue_robot_pose.header.stamp = sim_blue_pose.header.stamp;
    blue_robot_pose.pose.pose = sim_blue_pose.pose;
    blue_robot_pose_pub_->publish(blue_robot_pose);
  }
}

void IgnitionConverter::sim_yellow_poses_callback(const geometry_msgs::msg::PoseStamped & sim_yellow_pose)
{
  if (sim_yellow_pose.header.frame_id == "car_world") {
    geometry_msgs::msg::PoseWithCovarianceStamped yellow_robot_pose;
    yellow_robot_pose.header.frame_id = "map";
    yellow_robot_pose.header.stamp = sim_yellow_pose.header.stamp;
    yellow_robot_pose.pose.pose = sim_yellow_pose.pose;
    yellow_robot_pose_pub_->publish(yellow_robot_pose);
  }
}

void IgnitionConverter::blue_catch_callback(const Catch & blue_catch_msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IgnitionConverter::blue_catch_callback()");

  auto string_msg = std_msgs::msg::String();
  // string_msg.header.stamp = rclcpp::Time();
  switch (blue_catch_msg.object_type)
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
  blue_catch_converted_pub_->publish(string_msg);
}

void IgnitionConverter::blue_release_callback(const Catch & blue_release_msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IgnitionConverter::blue_release_callback()");

  auto string_msg = std_msgs::msg::String();
  // string_msg.header.stamp = rclcpp::Time();
  switch (blue_release_msg.object_type)
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
  blue_release_converted_pub_->publish(string_msg);
}

void IgnitionConverter::yellow_catch_callback(const Catch & yellow_catch_msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IgnitionConverter::yellow_catch_callback()");

  auto string_msg = std_msgs::msg::String();
  // string_msg.header.stamp = rclcpp::Time();
  switch (yellow_catch_msg.object_type)
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
  yellow_catch_converted_pub_->publish(string_msg);
}

void IgnitionConverter::yellow_release_callback(const Catch & yellow_release_msg) {
  RCLCPP_INFO_STREAM(this->get_logger(), "IgnitionConverter::yellow_release_callback()");

  auto string_msg = std_msgs::msg::String();
  // string_msg.header.stamp = rclcpp::Time();
  switch (yellow_release_msg.object_type)
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
  yellow_release_converted_pub_->publish(string_msg);
}

}  // namespace simulator