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

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/common/Profiler.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/DetachableJoint.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "RetachableJoint.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

void RetachableJoint::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager &_ecm,
                                EventManager &/*_eventMgr*/)
{
  ignmsg << "RetachableJoint: Configure()" << std::endl;
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm)) {
    ignerr << "RetachableJoint should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  if (!_sdf->HasElement("parent_link")) {
    ignerr << "'parent_link' is a required parameter for RetachableJoint. "
              "Failed to initialize.\n";
    return;
  }
  auto parentLinkName = _sdf->Get<std::string>("parent_link");
  this->parentLinkEntity = this->model.LinkByName(_ecm, parentLinkName);

  if (kNullEntity == this->parentLinkEntity) {
      ignerr << "Link with name " << parentLinkName
              << " not found in model " << this->model.Name(_ecm)
              << ". Make sure the parameter 'parent_link' has the "
              << "correct value. Failed to initialize.\n";
      return;
  }

  if (!_sdf->HasElement("child_model")) {
    ignerr << "'child_model' is a required parameter for RetachableJoint."
              "Failed to initialize.\n";
    return;
  }
  this->childModelName = _sdf->Get<std::string>("child_model");

  if (!_sdf->HasElement("child_link")) {
    ignerr << "'child_link' is a required parameter for RetachableJoint."
              "Failed to initialize.\n";
    return;
  }
  this->childLinkName = _sdf->Get<std::string>("child_link");

  std::string defaultAttachTopic{"/model/" + this->model.Name(_ecm) +
                             "/detachable_joint/attach"};
  this->attach_topic = _sdf->Get<std::string>("attach_topic", defaultAttachTopic).first;
  
  std::string defaultDetachTopic{"/model/" + this->model.Name(_ecm) +
                             "/detachable_joint/detach"};
  this->detach_topic = _sdf->Get<std::string>("detach_topic", defaultDetachTopic).first;

  // this->suppressChildWarning =
  //     _sdf->Get<bool>("suppress_child_warning", this->suppressChildWarning)
  //         .first;

  this->node.Subscribe(
    this->attach_topic, &RetachableJoint::OnAttachRequest, this);
  
  ignmsg << "RetachableJoint subscribing to messages on "
          << "[" << this->attach_topic << "]" << std::endl;

  this->node.Subscribe(
    this->detach_topic, &RetachableJoint::OnDetachRequest, this);

  ignmsg << "RetachableJoint subscribing to messages on "
          << "[" << this->detach_topic << "]" << std::endl;

  ignmsg << "RetachableJoint: validConfig=true" << std::endl;
  this->validConfig = true;
}

void RetachableJoint::PreUpdate(
  const ignition::gazebo::UpdateInfo &/*_info*/,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  // ignmsg << "RetachableJoint: PreUpdate()" << std::endl;
  IGN_PROFILE("RetachableJoint::PreUpdate");
  if (!this->validConfig) {
    return;
  }

  if (!this->attached) {
    if (this->attachRequested) {
      // Entity modelEntity{kNullEntity};

      // if ("__model__" == this->childModelName)
      // {
      //   modelEntity = this->model.Entity();
      // }
      // else
      // {
      Entity modelEntity = _ecm.EntityByComponents(
          components::Model(), components::Name(this->childModelName));
      // }

      if (kNullEntity == modelEntity) {
          ignwarn << "Child Link " << this->childLinkName
                  << " could not be found.\n";
          return;
      }

      // if (kNullEntity != modelEntity) {
      this->childLinkEntity = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(modelEntity),
          components::Name(this->childLinkName));

      if (kNullEntity == this->childLinkEntity) {
        ignwarn << "Child Link " << this->childLinkName
                << " could not be found.\n";
        return;
      }

      // if (kNullEntity != this->childLinkEntity)
      // {
        // Attach the models
        // We do this by creating a detachable joint entity.
      this->detachableJointEntity = _ecm.CreateEntity();

      _ecm.CreateComponent(
          this->detachableJointEntity,
          components::DetachableJoint({this->parentLinkEntity,
                                        this->childLinkEntity, "fixed"}));

      

      this->attached = true;
      this->attachRequested = false;
      ignmsg << "RetachableJoint: attached = true" << std::endl;
      // }
        // else
        // {
        // }
      // }
      // else if (!this->suppressChildWarning)
      // {
      //   ignwarn << "Child Model " << this->childModelName
      //           << " could not be found.\n";
      // }
    }
  }

  if (this->attached) {
    if (this->detachRequested && (kNullEntity != this->detachableJointEntity))
    {
      ignmsg << "RetachableJoint: Removing entity" << std::endl;
      // Detach the models
      igndbg << "Removing entity: " << this->detachableJointEntity << std::endl;
      _ecm.RequestRemoveEntity(this->detachableJointEntity);
      this->detachableJointEntity = kNullEntity;
      this->detachRequested = false;
    }
  }
}

void RetachableJoint::OnAttachRequest(const msgs::Empty &)
{
  this->attachRequested = true;
}

void RetachableJoint::OnDetachRequest(const msgs::Empty &)
{
  this->detachRequested = true;
}

IGNITION_ADD_PLUGIN(RetachableJoint,
                    ignition::gazebo::System,
                    RetachableJoint::ISystemConfigure,
                    RetachableJoint::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RetachableJoint,
  "ignition::gazebo::systems::RetachableJoint")
