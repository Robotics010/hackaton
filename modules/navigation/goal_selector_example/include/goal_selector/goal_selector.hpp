#ifndef GOAL_SELECTOR__GOAL_SELECTOR_HPP_
#define GOAL_SELECTOR__GOAL_SELECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
  GoalSelector();
 private:
  void tick_execution();
  
  State initializing();
  State go_center();
  State waiting_center();
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

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  geometry_msgs::msg::PoseStamped last_goal_;

  bool get_current_pose();
  geometry_msgs::msg::PoseStamped current_pose_;
};

}  // namespace navigation

#endif  // GOAL_SELECTOR__GOAL_SELECTOR_HPP_
