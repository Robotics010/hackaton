#include "goal_selector/goal_selector.hpp"

#include <chrono>
#include <thread>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;
using std::placeholders::_1, std::placeholders::_2;

namespace navigation
{

GoalSelector::GoalSelector() : Node("goal_selector"), state_(INITIALIZING) {

  this->declare_parameter("global_frame", std::string("map"));
  global_frame_ = this->get_parameter("global_frame").as_string();

  this->declare_parameter("robot_base_frame", std::string("base_link"));
  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();

  this->declare_parameter("transform_tolerance", 0.1);
  transform_tolerance_ = this->get_parameter("transform_tolerance").as_double();

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);
  
  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/output/goal_pose", 10);
  goal_completed_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/input/goal_completed", 10, std::bind(&GoalSelector::goal_completed_callback, this, _1));

  catch_pub_ = this->create_publisher<Catch>("~/output/catch", 10);
  release_pub_ = this->create_publisher<Catch>("~/output/release", 10);

  execution_timer_ =
    this->create_wall_timer(100ms, std::bind(&GoalSelector::tick_execution, this));
}

void GoalSelector::tick_execution() { // state_machine = конечный автомат
  // RCLCPP_INFO_STREAM(this->get_logger(), "GoalSelector::tick_execution()");

  switch (state_)
  {
  case INITIALIZING:
    state_ = initializing();
    break;

  case GO_CENTER:
    state_ = go_column();
    break;
  
  case WAITING_CENTER:
    state_ = waiting_column();
    break;
  
  case GO_FINISH:
    state_ = go_finish();
    break;

  case WAITING_FINISH:
    state_ = waiting_finish();
    break;

  case IDLE:
  default:
    state_ = idle();
    break;
  }
}

State GoalSelector::initializing() {
  if (!get_current_pose()) {
    return INITIALIZING;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "going center");
  return GO_CENTER;
}

State GoalSelector::go_column() {

  geometry_msgs::msg::PoseStamped new_goal;
  new_goal.header.stamp = rclcpp::Time();
  new_goal.header.frame_id = "map";
  new_goal.pose.position.x = -0.55;
  new_goal.pose.position.y = -0.05-0.150-0.25/2;
  new_goal.pose.position.z = 0;
  new_goal.pose.orientation.x = 0.0;
  new_goal.pose.orientation.y = 0.0;
  new_goal.pose.orientation.z = 0.7071068;
  new_goal.pose.orientation.w = 0.7071068;
  goal_pose_pub_->publish(new_goal);
  last_goal_ = new_goal;
  last_goal_completed_ = false;
  RCLCPP_INFO_STREAM(this->get_logger(),
                    "send goal, pose, x: " << new_goal.pose.position.x
                    << ", y: " << new_goal.pose.position.y
                    << ", z: " << new_goal.pose.position.z
                    << ", orientation, qx: " << new_goal.pose.orientation.x
                    << ", qy: " << new_goal.pose.orientation.y
                    << ", qz: " << new_goal.pose.orientation.z
                    << ", qw: " << new_goal.pose.orientation.w);
  
  return WAITING_CENTER;
}

State GoalSelector::waiting_column() {
  if (!get_current_pose()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No current pose available...");
  }

  if (!last_goal_completed_) {
    return WAITING_CENTER;
  }

  get_clock()->sleep_for(rclcpp::Duration(2.5, 0));

  auto catch_msg = Catch();
  catch_msg.header.stamp = rclcpp::Time();
  catch_msg.object_type = Catch::COLUMN;
  catch_pub_->publish(catch_msg);
  
  RCLCPP_INFO_STREAM(this->get_logger(), "going finish");
  return GO_FINISH;
}

State GoalSelector::go_finish() {
  geometry_msgs::msg::PoseStamped new_goal;
  new_goal.header.stamp = rclcpp::Time();
  new_goal.header.frame_id = "map";
  new_goal.pose.position.x = -1.275-0.15;
  new_goal.pose.position.y = -0.125;
  new_goal.pose.position.z = 0;
  new_goal.pose.orientation.x = 0.0;
  new_goal.pose.orientation.y = 0.0;
  new_goal.pose.orientation.z = 1.0;
  new_goal.pose.orientation.w = 0.0;
  goal_pose_pub_->publish(new_goal);
  last_goal_ = new_goal;
  RCLCPP_INFO_STREAM(this->get_logger(),
                    "send goal, pose, x: " << new_goal.pose.position.x
                    << ", y: " << new_goal.pose.position.y
                    << ", z: " << new_goal.pose.position.z
                    << ", orientation, qx: " << new_goal.pose.orientation.x
                    << ", qy: " << new_goal.pose.orientation.y
                    << ", qz: " << new_goal.pose.orientation.z
                    << ", qw: " << new_goal.pose.orientation.w);
  
  return WAITING_FINISH;
}

State GoalSelector::waiting_finish() {
  if (!get_current_pose()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "No current pose available...");
  }

  if (!last_goal_completed_) {
    return WAITING_FINISH;
  }

  get_clock()->sleep_for(rclcpp::Duration(2.5, 0));

  auto release_msg = Catch();
  release_msg.header.stamp = rclcpp::Time();
  release_msg.object_type = Catch::COLUMN;
  release_pub_->publish(release_msg);
  
  RCLCPP_INFO_STREAM(this->get_logger(), "idle");
  return IDLE;
}

State GoalSelector::idle() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return IDLE;
}

bool GoalSelector::get_current_pose() {
  tf2::toMsg(tf2::Transform::getIdentity(), current_pose_.pose);
  current_pose_.header.frame_id = robot_base_frame_;
  current_pose_.header.stamp = rclcpp::Time();

  try {
    current_pose_ = tf_->transform(current_pose_, global_frame_,
                                   tf2::durationFromSec(transform_tolerance_));
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "No Transform available from " << global_frame_
                        << " to " << robot_base_frame_ << ": " << ex.what());
    return false;
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Connectivity Error from " << global_frame_
                        << " to " << robot_base_frame_ << ": " << ex.what());
    return false;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Extrapolation Error from " << global_frame_
                        << " to " << robot_base_frame_ << ": " << ex.what());
    return false;
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Transform timeout with tolerance " << transform_tolerance_
                        << ": " << ex.what());
    return false;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Failed to transform from " << global_frame_
                        << " to " << robot_base_frame_ << ": " << ex.what());
    return false;
  }
  return true;
}

void GoalSelector::goal_completed_callback(const geometry_msgs::msg::PoseStamped & pose) {
  if (last_goal_.pose == pose.pose) {
    last_goal_completed_ = true;
    RCLCPP_INFO(this->get_logger(), "goal_completed!");
  }
}

bool GoalSelector::is_close(const geometry_msgs::msg::Point& one,
                            const geometry_msgs::msg::Point& another) const {
  const double close_enough = 0.2;

  const double diff = std::sqrt(
    (std::pow(one.x - another.x, 2.0) + std::pow(one.y - another.y, 2.0)));
  
  RCLCPP_DEBUG_STREAM(this->get_logger(), "diff: " << diff);

  if (diff < close_enough) {
    return true;
  }
  return false;
}

}  // namespace navigation
