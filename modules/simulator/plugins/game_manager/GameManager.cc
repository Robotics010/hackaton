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
  platform_positions_.reserve(20);

  std::string defaultPosesInfoTopic{"/world/car_world/pose/info"};
  this->poses_info_topic = _sdf->Get<std::string>("poses_info_topic", defaultPosesInfoTopic).first;

  this->node.Subscribe(
    this->poses_info_topic, &GameManager::OnPosesInfoUpdate, this);
  
  igndbg << "GameManager subscribing to messages on "
          << "[" << this->poses_info_topic << "]" << std::endl;

  std::string blue_robot_catch_topic{"/model/blue/catch"};
  
  if (!this->node.Subscribe(blue_robot_catch_topic, &GameManager::OnBlueRobotCatch, this)) {
    ignerr << "GameManager Error subscribing to topic "
          << "[" << blue_robot_catch_topic << "]" << std::endl;
    return;
  }
  
  igndbg << "GameManager subscribing to messages on "
          << "[" << blue_robot_catch_topic << "]" << std::endl;

  std::string blue_robot_release_topic{"/model/blue/release"};
  
  if (!this->node.Subscribe(blue_robot_release_topic, &GameManager::OnBlueRobotRelease, this)) {
    ignerr << "GameManager Error subscribing to topic "
          << "[" << blue_robot_release_topic << "]" << std::endl;
    return;
  }

  igndbg << "GameManager subscribing to messages on "
          << "[" << blue_robot_release_topic << "]" << std::endl;
  
  std::stringstream ss;
  for (int i = 1; i <= 40; i++) {
    ss.str(std::string(""));
    ss << "/blue/column_" << i << "/attach";
    blue_attach_column_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;

    ss.str(std::string(""));
    ss << "/blue/column_" << i << "/detach";
    blue_detach_column_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  }

  for (int i = 1; i <= 20; i++) {
    ss.str(std::string(""));
    ss << "/blue/platform_" << i << "/attach";
    blue_attach_platform_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  
    ss.str(std::string(""));
    ss << "/blue/platform_" << i << "/detach";
    blue_detach_platform_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  }

  std::string yellow_robot_catch_topic{"/model/yellow/catch"};
  
  if (!this->node.Subscribe(yellow_robot_catch_topic, &GameManager::OnYellowRobotCatch, this)) {
    ignerr << "GameManager Error subscribing to topic "
          << "[" << yellow_robot_catch_topic << "]" << std::endl;
    return;
  }
  
  igndbg << "GameManager subscribing to messages on "
          << "[" << yellow_robot_catch_topic << "]" << std::endl;

  std::string yellow_robot_release_topic{"/model/yellow/release"};
  
  if (!this->node.Subscribe(yellow_robot_release_topic, &GameManager::OnYellowRobotRelease, this)) {
    ignerr << "GameManager Error subscribing to topic "
          << "[" << yellow_robot_release_topic << "]" << std::endl;
    return;
  }

  igndbg << "GameManager subscribing to messages on "
          << "[" << yellow_robot_release_topic << "]" << std::endl;
  
  for (int i = 1; i <= 40; i++) {
    ss.str(std::string(""));
    ss << "/yellow/column_" << i << "/attach";
    yellow_attach_column_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;

    ss.str(std::string(""));
    ss << "/yellow/column_" << i << "/detach";
    yellow_detach_column_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  }

  for (int i = 1; i <= 20; i++) {
    ss.str(std::string(""));
    ss << "/yellow/platform_" << i << "/attach";
    yellow_attach_platform_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
    igndbg << "GameManager advertising messages on " << "[" << ss.str() << "]" << std::endl;
  
    ss.str(std::string(""));
    ss << "/yellow/platform_" << i << "/detach";
    yellow_detach_platform_pubs_[i] = this->node.Advertise<msgs::Empty>(ss.str());
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
    if (pose.name() == "blue") {
      blue_robot_position_ = pose.position();
      blue_robot_orientation_ = pose.orientation();
    } else if (pose.name() == "yellow") {
      yellow_robot_position_ = pose.position();
      yellow_robot_orientation_ = pose.orientation();
    }else if (pose.name().find("column_") != std::string::npos) {
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

void GameManager::OnBlueRobotCatch(const msgs::StringMsg &_msg)
{
  igndbg << "GameManager: OnBlueRobotCatch()" << std::endl;

  if (_msg.data() != "COLUMN" && _msg.data() != "PLATFORM") {
    igndbg << "attach not_supported: '" << _msg.data() << "'" << std::endl;
    return;
  }

  const double catch_radius = 0.150;
  const double body_offset = 0.25 / 2 + catch_radius;

  gz::math::Quaterniond robot_quaternion(
    blue_robot_orientation_.w(),
    blue_robot_orientation_.x(),
    blue_robot_orientation_.y(),
    blue_robot_orientation_.z());
  gz::math::Vector3d robot_euler(robot_quaternion.Euler());

  const double initial_vector_x = 1.0 * body_offset;
  const double initial_vector_y = 0.0;

  const double projected_offset_x = initial_vector_x * std::cos(robot_euler.Z()) - initial_vector_y * std::sin(robot_euler.Z());
  const double projected_offset_y = initial_vector_y * std::cos(robot_euler.Z()) + initial_vector_x * std::sin(robot_euler.Z());

  const double catch_x = blue_robot_position_.x() + projected_offset_x;
  const double catch_y = blue_robot_position_.y() + projected_offset_y;

  igndbg << "blue_position"
        << ", x: " << blue_robot_position_.x()
        << ", y: " << blue_robot_position_.y()
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
      this->blue_attach_column_pubs_[closest_id].Publish(empty_msg);
      blue_column_catched_ = true;
      blue_column_id_catched_ = closest_id;
      igndbg << "attach column " << closest_id << std::endl;
    }
  } else if (_msg.data() == "PLATFORM") {
    auto highest_platform_ids = get_highest_platform_ids_sorted(catch_x, catch_y, catch_radius);
    if (!highest_platform_ids.empty()) {
      int highest_id = highest_platform_ids[0];
      msgs::Empty empty_msg;
      this->blue_attach_platform_pubs_[highest_id].Publish(empty_msg);
      blue_platform_catched_ = true;
      blue_platform_id_catched_ = highest_id;
      igndbg << "attach platform " << highest_id << std::endl;
    }
  }
}

void GameManager::OnBlueRobotRelease(const msgs::StringMsg &_msg)
{
  igndbg << "GameManager: OnBlueRobotRelease()" << std::endl;
  msgs::Empty empty_msg;
  if (_msg.data() == "COLUMN" && blue_column_catched_) {
    blue_column_catched_ = false;
    this->blue_detach_column_pubs_[blue_column_id_catched_].Publish(empty_msg);
    igndbg << "detach column " << blue_column_id_catched_ << std::endl;
  } else if (_msg.data() == "PLATFORM" && blue_platform_catched_) {
    blue_platform_catched_ = false;
    this->blue_detach_platform_pubs_[blue_platform_id_catched_].Publish(empty_msg);
    igndbg << "detach platform " << blue_platform_id_catched_ << std::endl;
  } else {
    igndbg << "detach not_supported: '" << _msg.data() << "'" << std::endl;
  }
}

void GameManager::OnYellowRobotCatch(const msgs::StringMsg &_msg)
{
  igndbg << "GameManager: OnYellowRobotCatch()" << std::endl;

  if (_msg.data() != "COLUMN" && _msg.data() != "PLATFORM") {
    igndbg << "attach not_supported: '" << _msg.data() << "'" << std::endl;
    return;
  }

  const double catch_radius = 0.150;
  const double body_offset = 0.25 / 2 + catch_radius;

  gz::math::Quaterniond robot_quaternion(
    yellow_robot_orientation_.w(),
    yellow_robot_orientation_.x(),
    yellow_robot_orientation_.y(),
    yellow_robot_orientation_.z());
  gz::math::Vector3d robot_euler(robot_quaternion.Euler());

  const double initial_vector_x = 1.0 * body_offset;
  const double initial_vector_y = 0.0;

  const double projected_offset_x = initial_vector_x * std::cos(robot_euler.Z()) - initial_vector_y * std::sin(robot_euler.Z());
  const double projected_offset_y = initial_vector_y * std::cos(robot_euler.Z()) + initial_vector_x * std::sin(robot_euler.Z());

  const double catch_x = yellow_robot_position_.x() + projected_offset_x;
  const double catch_y = yellow_robot_position_.y() + projected_offset_y;

  igndbg << "yellow_position"
        << ", x: " << yellow_robot_position_.x()
        << ", y: " << yellow_robot_position_.y()
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
      this->yellow_attach_column_pubs_[closest_id].Publish(empty_msg);
      yellow_column_catched_ = true;
      yellow_column_id_catched_ = closest_id;
      igndbg << "attach column " << closest_id << std::endl;
    }
  } else if (_msg.data() == "PLATFORM") {
    auto highest_platform_ids = get_highest_platform_ids_sorted(catch_x, catch_y, catch_radius);
    if (!highest_platform_ids.empty()) {
      int highest_id = highest_platform_ids[0];
      msgs::Empty empty_msg;
      this->yellow_attach_platform_pubs_[highest_id].Publish(empty_msg);
      yellow_platform_catched_ = true;
      yellow_platform_id_catched_ = highest_id;
      igndbg << "attach platform " << highest_id << std::endl;
    }
  }
}

void GameManager::OnYellowRobotRelease(const msgs::StringMsg &_msg)
{
  igndbg << "GameManager: OnYellowRobotRelease()" << std::endl;
  msgs::Empty empty_msg;
  if (_msg.data() == "COLUMN" && yellow_column_catched_) {
    yellow_column_catched_ = false;
    this->yellow_detach_column_pubs_[yellow_column_id_catched_].Publish(empty_msg);
    igndbg << "detach column " << yellow_column_id_catched_ << std::endl;
  } else if (_msg.data() == "PLATFORM" && yellow_platform_catched_) {
    yellow_platform_catched_ = false;
    this->yellow_detach_platform_pubs_[yellow_platform_id_catched_].Publish(empty_msg);
    igndbg << "detach platform " << yellow_platform_id_catched_ << std::endl;
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
