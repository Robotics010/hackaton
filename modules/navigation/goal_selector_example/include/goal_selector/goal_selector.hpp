#ifndef GOAL_SELECTOR__GOAL_SELECTOR_HPP_
#define GOAL_SELECTOR__GOAL_SELECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "goal_selector_msgs/msg/catch.hpp"

namespace navigation
{

enum State {
  INITIALIZING,
  GO_CENTER,
  WAITING_CENTER,
  GO_FINISH,
  WAITING_FINISH,
  IDLE,
};

class GoalSelector : public rclcpp::Node {
 public:
  using Catch = goal_selector_msgs::msg::Catch;
  GoalSelector();
 private:
  void tick_execution();
  
  State initializing();
  State go_column();
  State waiting_column();
  State go_finish();
  State waiting_finish();
  State idle();

  bool is_close(const geometry_msgs::msg::Point& one,
                const geometry_msgs::msg::Point& another) const;

  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr execution_timer_;
  State state_;

  void goal_completed_callback(const geometry_msgs::msg::PoseStamped & pose);
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_completed_sub_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool last_goal_completed_;

  bool get_current_pose();
  geometry_msgs::msg::PoseStamped current_pose_;

  rclcpp::Publisher<Catch>::SharedPtr catch_pub_;
  rclcpp::Publisher<Catch>::SharedPtr release_pub_;
};

}  // namespace navigation

#endif  // GOAL_SELECTOR__GOAL_SELECTOR_HPP_
