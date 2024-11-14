/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <string>
#include <sstream>
#include <utility>
#include <cmath>
#include <algorithm>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/common/Profiler.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "GameManager.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

void GameManager::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager &_ecm,
                                EventManager &/*_eventMgr*/)
{
  igndbg << "GameManager: Configure()" << std::endl;

  column_positions_.reserve(40);

  std::string defaultPosesInfoTopic{"/world/car_world/pose/info"};
  this->poses_info_topic = _sdf->Get<std::string>("poses_info_topic", defaultPosesInfoTopic).first;
  
  this->node.Subscribe(
    this->poses_info_topic, &GameManager::OnPosesInfoUpdate, this);
  
  igndbg << "GameManager subscribing to messages on "
          << "[" << this->poses_info_topic << "]" << std::endl;

  std::string defaultRobotCatchTopic{"/model/big_robotic/catch"};
  this->robot_catch_topic = _sdf->Get<std::string>("robot_catch_topic", defaultRobotCatchTopic).first;
  
  if (!this->node.Subscribe(this->robot_catch_topic, &GameManager::OnRobotCatch, this)) {
    ignerr << "GameManager Error subscribing to topic "
          << "[" << this->robot_catch_topic << "]" << std::endl;
    return;
  }
  
  igndbg << "GameManager subscribing to messages on "
          << "[" << this->robot_catch_topic << "]" << std::endl;

  std::string defaultRobotReleaseTopic{"/model/big_robotic/release"};
  this->robot_release_topic = _sdf->Get<std::string>("robot_release_topic", defaultRobotReleaseTopic).first;
  
  if (!this->node.Subscribe(this->robot_release_topic, &GameManager::OnRobotRelease, this)) {
    ignerr << "GameManager Error subscribing to topic "
          << "[" << this->robot_release_topic << "]" << std::endl;
    return;
  }

  igndbg << "GameManager subscribing to messages on "
          << "[" << this->robot_release_topic << "]" << std::endl;
  
  std::stringstream ss;
  for (int i = 1; i <= 40; i++) {
    ss.str(std::string(""));
    ss << "/column_" << i << "/attach";
    attach_column_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;

    ss.str(std::string(""));
    ss << "/column_" << i << "/detach";
    detach_column_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  }

  for (int i = 1; i <= 20; i++) {
    ss.str(std::string(""));
    ss << "/platform_" << i << "/attach";
    attach_platform_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  
    ss.str(std::string(""));
    ss << "/platform_" << i << "/detach";
    detach_platform_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  }

  ignmsg << "GameManager: validConfig=true" << std::endl;
  this->validConfig = true;
}

void GameManager::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("GameManager::PreUpdate");
  if (!this->validConfig) {
    return;
  }

}

void GameManager::OnPosesInfoUpdate(const msgs::Pose_V &_msg)
{
  const int column_offset = std::string("column_").length();
  const int platform_offset = std::string("platform_").length();

  for (const auto& pose : _msg.pose()) {
    if (pose.name() == "big_robotic") {
      robot_position_ = pose.position();
      robot_orientation_ = pose.orientation();
    } else if (pose.name().find("column_") != std::string::npos) {
      int column_id = std::stoi(pose.name().substr(column_offset, pose.name().size() - column_offset));
      column_positions_[column_id] = pose.position();
    } else if (pose.name().find("platform_") != std::string::npos) {
      int platform_id = std::stoi(pose.name().substr(platform_offset, pose.name().size() - platform_offset));
      platform_positions_[platform_id] = pose.position();
    } else {
      continue;
    }
  }
}

void GameManager::OnRobotCatch(const msgs::StringMsg &_msg)
{
  igndbg << "GameManager: OnRobotCatch()" << std::endl;

  if (_msg.data() != "COLUMN" && _msg.data() != "PLATFORM") {
    igndbg << "attach not_supported: '" << _msg.data() << "'" << std::endl;
    return;
  }

  const double catch_radius = 0.150;
  const double body_offset = 0.25 / 2 + catch_radius;

  gz::math::Quaterniond robot_quaternion(
    robot_orientation_.w(),
    robot_orientation_.x(),
    robot_orientation_.y(),
    robot_orientation_.z());
  gz::math::Vector3d robot_euler(robot_quaternion.Euler());

  const double initial_vector_x = 1.0 * body_offset;
  const double initial_vector_y = 0.0;

  const double projected_offset_x = initial_vector_x * std::cos(robot_euler.Z()) - initial_vector_y * std::sin(robot_euler.Z());
  const double projected_offset_y = initial_vector_y * std::cos(robot_euler.Z()) + initial_vector_x * std::sin(robot_euler.Z());

  const double catch_x = robot_position_.x() + projected_offset_x;
  const double catch_y = robot_position_.y() + projected_offset_y;

  igndbg << "robot_position"
        << ", x: " << robot_position_.x()
        << ", y: " << robot_position_.y()
        << std::endl;
  igndbg << "catch"
        << ", x: " << catch_x
        << ", y: " << catch_y
        << std::endl;

  if (_msg.data() == "COLUMN") {
    auto closest_column_ids = get_closest_column_ids_sorted(catch_x, catch_y, catch_radius);
    if (!closest_column_ids.empty()) {
      int closest_id = closest_column_ids[0];
      msgs::Empty empty_msg;
      this->attach_column_pubs_[closest_id].Publish(empty_msg);
      column_catched_ = true;
      column_id_catched_ = closest_id;
      igndbg << "attach column " << closest_id << std::endl;
    }
  } else if (_msg.data() == "PLATFORM") {
    auto highest_platform_ids = get_highest_platform_ids_sorted(catch_x, catch_y, catch_radius);
    if (!highest_platform_ids.empty()) {
      int highest_id = highest_platform_ids[0];
      msgs::Empty empty_msg;
      this->attach_platform_pubs_[highest_id].Publish(empty_msg);
      platform_catched_ = true;
      platform_id_catched_ = highest_id;
      igndbg << "attach platform " << highest_id << std::endl;
    }
  }
}

void GameManager::OnRobotRelease(const msgs::StringMsg &_msg)
{
  igndbg << "GameManager: OnRobotRelease()" << std::endl;
  msgs::Empty empty_msg;
  if (_msg.data() == "COLUMN" && column_catched_) {
    column_catched_ = false;
    this->detach_column_pubs_[column_id_catched_].Publish(empty_msg);
    igndbg << "detach column " << column_id_catched_ << std::endl;
  } else if (_msg.data() == "PLATFORM" && platform_catched_) {
    platform_catched_ = false;
    this->detach_platform_pubs_[platform_id_catched_].Publish(empty_msg);
    igndbg << "detach platform " << platform_id_catched_ << std::endl;
  } else {
    igndbg << "detach not_supported: '" << _msg.data() << "'" << std::endl;
  }
}

std::vector<int> GameManager::get_closest_column_ids_sorted(const double& x,
                                                            const double& y,
                                                            const double& radius_m) {
  typedef std::pair<int,double> ColumnPair;
  std::vector<ColumnPair> closest_columns;
  double x_min, x_max, y_min, y_max;
  x_min = x - radius_m;
  x_max = x + radius_m;
  y_min = y - radius_m;
  y_max = y + radius_m;

  igndbg << "get_closest_column_ids_sorted()"
         << ", x_min: " << x_min
         << ", x_max: " << x_max
         << ", y_min: " << y_min
         << ", y_max: " << y_max
         << std::endl;
  for (size_t id = 1; id <= column_positions_.size(); id++) {
    const auto& column_position = column_positions_[id];
    if ((column_position.x() > x_min) && (column_position.x() < x_max)
          && (column_position.y() > y_min) && (column_position.y() < y_max)) {
        const double euclidean_distance = std::hypot(
          x - column_position.x(), y - column_position.y());
        closest_columns.push_back(ColumnPair(id, euclidean_distance));
      }
  }

  std::sort(closest_columns.begin(), closest_columns.end(), [](auto &left, auto &right) {
    return left.second < right.second;
  });

  std::vector<int> closest_column_ids;
  for (const auto& column : closest_columns) {
    igndbg << "closest_columns: "
           << column.first << ") " << column.second
           << std::endl;
    closest_column_ids.push_back(column.first);
  }
  return closest_column_ids;
}

std::vector<int> GameManager::get_highest_platform_ids_sorted(const double& x,
                                                              const double& y,
                                                              const double& radius_m) {
  typedef std::pair<int,double> PlatformPair;
  std::vector<PlatformPair> highest_platforms;
  double x_min, x_max, y_min, y_max;
  x_min = x - radius_m;
  x_max = x + radius_m;
  y_min = y - radius_m;
  y_max = y + radius_m;

  for (size_t id = 1; id <= platform_positions_.size(); id++) {
    const auto& platform_position = platform_positions_[id];
    if ((platform_position.x() > x_min) && (platform_position.x() < x_max)
          && (platform_position.y() > y_min) && (platform_position.y() < y_max)) {
      highest_platforms.push_back(PlatformPair(id, platform_position.z()));
    }
  }

  std::sort(highest_platforms.begin(), highest_platforms.end(), [](auto &left, auto &right) {
    return left.second > right.second;
  });

  std::vector<int> highest_platform_ids;
  for (const auto& platform : highest_platforms) {
    igndbg << "highest_platform: "
           << platform.first << ") " << platform.second
           << std::endl;
    highest_platform_ids.push_back(platform.first);
  }
  return highest_platform_ids;
}

IGNITION_ADD_PLUGIN(GameManager,
                    ignition::gazebo::System,
                    GameManager::ISystemConfigure,
                    GameManager::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(GameManager,
  "ignition::gazebo::systems::GameManager")
