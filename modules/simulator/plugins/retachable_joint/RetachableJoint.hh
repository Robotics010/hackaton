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
  /// \brief A system that initially attaches two models via a fixed joint and
  /// allows for the models to get detached during simulation via a topic.
  ///
  /// Parameters:
  ///
  /// <parent_link>: Name of the link in the parent model to be used in
  /// creating a fixed joint with a link in the child model.
  ///
  /// <child_model>: Name of the model to which this model will be connected
  ///
  /// <child_link>: Name of the link in the child model to be used in
  /// creating a fixed joint with a link in the parent model.
  ///
  /// <detach_topic> (optional): Topic name to be used for detaching connections
  ///
  /// <suppress_child_warning> (optional): If true, the system
  /// will not print a warning message if a child model does not exist yet.
  /// Otherwise, a warning message is printed. Defaults to false.

class IGNITION_GAZEBO_VISIBLE RetachableJoint
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
 public:
    RetachableJoint() = default;
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
    bool attached{false};

    Model model;
    std::string childModelName;
    std::string childLinkName;
    Entity parentLinkEntity{kNullEntity};
    Entity childLinkEntity{kNullEntity};
    Entity detachableJointEntity{kNullEntity};

    void OnAttachRequest(const msgs::Empty &_msg);
    void OnDetachRequest(const msgs::Empty &_msg);
    std::string attach_topic;
    std::string detach_topic;
    std::atomic<bool> attachRequested{false};
    std::atomic<bool> detachRequested{false};
};
} // namespace systems

}
} // namespace gazebo
} // ignition

#endif
