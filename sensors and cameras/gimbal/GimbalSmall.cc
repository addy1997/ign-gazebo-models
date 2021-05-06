/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

/*
 * Modified by Adwait Naik, 16 April 2021.
 *
*/

#include <string>
#include <vector>
#include <chrono>
#include <mutex>

#include <ignition/transport/Node.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>

#include <ignition/msgs.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/PID.hh>


#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "GimbalSmall.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::GimbalSmallPrivate
{
	  /// \brief Callback when a command string is received.
  	/// \param[in] _msg Mesage containing the command string
    public: void OnStringMsg(const msgs::StringMsg &_msg);

    /// \brief node for ignition communication
    public: transport::Node node;

    /// \brief model interface
    public: Model model{kNullEntity};

    /// \brief Publisher to the gimbal status topic
    public: transport::Node::Publisher pub;

    /// \brief PID controller for the gimbal 
    public: math::PID pid;

    /// \brief last udpate simTime
    public: std::chrono::steady_clock::duration lastUpdateTime{0};

    /// \brief command that updates the gimbal tilt angle
    public: double command = IGN_PI_2;

    /// \brief entity to the gimbal joint
    public: Entity tiltJoint{kNullEntity};

    /// \brief joint name
    public: std::vector<std::string>jointName;

    /// \brief scopedjointName
    public: std::vector<std::string>scopedjointName;

    /// \brief mutex to protect the command
    public: std::mutex JointCmdMutex;
    

};

//////////////////////////////////////////////////////////////////

GimbalSmall::GimbalSmall()
  : dataPtr(std::make_unique<GimbalSmallPrivate>())

{
	this->dataPtr->pid.Init(1,0,0,0,0,1.0,-1.0);

}
//////////////////////////////////////////////////////////////////

void GimbalSmall::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
	  this->dataPtr->model = Model(_entity);

  	if (!this->dataPtr->model.Valid(_ecm))
  	{
    	ignerr << "GimbalSmall2d plugin should be attached to a model entity. "
           	   << "Failed to initialize." << std::endl;
    	return;

  	}

    // Ugly, but needed because the sdf::Element::GetElement is not a const
  	// function and _sdf is a const shared pointer to a const sdf::Element.
    auto ptr = const_cast<sdf::Element*>(_sdf.get());

    // Get params from SDF
  	sdf::ElementPtr sdfElem = ptr->GetElement("tilt_joint");
  	if (sdfElem)
  	{
  		this->dataPtr->jointName.push_back(sdfElem->Get<std::string>());
  		sdfElem = sdfElem->GetNextElement("tilt_joint");
  	
  	}else{

  		this->dataPtr->scopedjointName.push_back(sdfElem->Get<std::string>());	
  	}

  	if (!sdfElem)
  	{

  		ignerr << "GimbalSmall::Load ERROR! Can't get joint" << std::endl;

  	}

}

/////////////////////////////////////////////////////////////////////////////

void GimbalSmall::Init()
{
	IGN_PROFILE("GimbalSmall::Init");

	ignition::gazebo::UpdateInfo _info;

	EntityComponentManager _ecm;

  sdf::ElementPtr _sdf;

	std::vector<std::string> topics;

  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }

  topics.push_back(std::string("~/") + this->dataPtr->model.Name(_ecm) + "/gimbal_tilt_cmd");

  auto topic = validTopic(topics);

  if (topic.empty())
  {
    ignerr << "GimbalSmall2d plugin received invalid model name "
           << "Failed to initialize." << std::endl;
    return;
  }
 
  this->dataPtr->node.Subscribe(topic, &GimbalSmallPrivate::OnStringMsg, this->dataPtr.get());

  //other topic
  std::vector<std::string> gimbtopics;

  gimbtopics.push_back("~/"+this->dataPtr->model.Name(_ecm) +"/gimbal_tilt_status");
  
  auto topic_ = validTopic(gimbtopics);

  this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::StringMsg>(topic_);
    
}

/////////////////////////////////////////////////////////////////////////////////

void GimbalSmallPrivate::OnStringMsg(const msgs::StringMsg &_msg)
{
	IGN_PROFILE("GimbalSmallPrivate::OnStringMsg");

  //use mutext to protect the command
	this->command = atof(_msg.data().c_str());

} 

//////////////////////////////////////////////////////////////////////////////////

void GimbalSmall::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{

	IGN_PROFILE("GimbalSmall::PreUpdate");
  IGN_PROFILE_BEGIN("Update");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    	ignwarn << "Detected jump back in time ["
        	    << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        		  << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // Create joint position component if one doesn't exist
  auto jointPos = _ecm.Component<components::JointPosition>(this->dataPtr->tiltJoint);

  if (jointPos == nullptr)
    return;

  double error;
  {

    std::lock_guard<std::mutex> lock(this->dataPtr->JointCmdMutex);
    error = jointPos->Data().at(0) - this->dataPtr->command;

  }

  //auto force = this->dataPtr->pid.Update(error, _info.dt);
  auto force = _ecm.Component<components::JointForceCmd>(this->dataPtr->tiltJoint);

  if (force==nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->tiltJoint,
        components::JointForceCmd({this->dataPtr->command}));
  }
  else
  {
    force->Data()[0] += this->dataPtr->command;
  }

  //_ecm.SetComponentData(this->dataPtr->tiltJoint, components::JointForceCmd(force));
   
  static int i = 1000;
  if(++i > 100)
  {
  		i=0;
  		std::stringstream ss;
  		ss << jointPos;
  		msgs::StringMsg m;
  		m.set_data(ss.str());
  		this->dataPtr->pub.Publish(m);
  }
  
  IGN_PROFILE_END();
}


IGNITION_ADD_PLUGIN(GimbalSmall,
                    ignition::gazebo::System,
                    GimbalSmall::ISystemConfigure,
                    GimbalSmall::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(GimbalSmall,
                          "ignition::gazebo::systems::GimbalSmall")
