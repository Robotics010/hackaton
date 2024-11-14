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

#ifndef IGNITION_GAZEBO_SYSTEMS_DETACHABLEJOINT_HH_
#define IGNITION_GAZEBO_SYSTEMS_DETACHABLEJOINT_HH_

#include <ignition/msgs/empty.pb.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"

namespace ignition 
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

namespace systems
{

class IGNITION_GAZEBO_VISIBLE GameManager
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
 public:
    GameManager() = default;
    void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;
    void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) final;


    transport::Node node;

 private:

    bool validConfig{false};

    void OnPosesInfoUpdate(const msgs::Pose_V &_msg);
    std::string poses_info_topic;
    msgs::Vector3d robot_position_;
    msgs::Quaternion robot_orientation_;
    std::unordered_map<int, msgs::Vector3d> column_positions_;
    std::unordered_map<int, msgs::Vector3d> platform_positions_;

    void OnRobotCatch(const msgs::StringMsg &_msg);
    std::string robot_catch_topic;
    bool column_catched_ {false};
    int column_id_catched_;
    bool platform_catched_ {false};
    int platform_id_catched_;

    void OnRobotRelease(const msgs::StringMsg &_msg);
    std::string robot_release_topic;

    std::vector<int> get_closest_column_ids_sorted(
      const double& x, const double& y, const double& radius_m);
    std::vector<int> get_highest_platform_ids_sorted(
      const double& x, const double& y, const double& radius_m);

    std::unordered_map<int, transport::Node::Publisher> attach_column_pubs_;
    std::unordered_map<int, transport::Node::Publisher> detach_column_pubs_;
    std::unordered_map<int, transport::Node::Publisher> attach_platform_pubs_;
    std::unordered_map<int, transport::Node::Publisher> detach_platform_pubs_;
};
} // namespace systems

}
} // namespace gazebo
} // ignition

#endif
