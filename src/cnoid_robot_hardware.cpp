/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Yohei Kakiuchi (JSK lab.)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Yohei Kakiuchi
*/

#include "cnoid_robot_hardware.h"
//#include <urdf/model.h>

#include <thread>

namespace cnoid_robot_hardware
{

bool CnoidRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)// add joint list
{
  // reading paramerters

  //prev_ref_positions_.resize(number_of_angles_);
#if 0
  std::string model_str;
  if (!root_nh.getParam("robot_description", model_str)) {
    ROS_ERROR("Failed to get model from robot_description");
    return false;
  }
  urdf::Model model;
  if (!model.initString(model_str)) {
    ROS_ERROR("Failed to parse robot_description");
    return false;
  }
#endif

#if 0
  ROS_WARN("model: %d %d", model.joints_.size(),
           model.links_.size());
  for (std::map<std::string, urdf::JointSharedPtr>::iterator joint = model.joints_.begin();joint != model.joints_.end(); joint++)
    {
      ROS_WARN("j: %s, %s", joint->first.c_str(),
               joint->second->name.c_str());
    }
#endif

  // set number of angles from model
  number_of_angles_ = use_joints.size();
  // joint_names_.resize(number_of_angles_);
  joint_types_.resize(number_of_angles_);
  joint_lower_limits_.resize(number_of_angles_);
  joint_upper_limits_.resize(number_of_angles_);
  joint_effort_limits_.resize(number_of_angles_);
  joint_control_methods_.resize(number_of_angles_);
  //pid_controllers_.resize(number_of_angles_);
  joint_position_.resize(number_of_angles_);
  joint_velocity_.resize(number_of_angles_);
  joint_effort_.resize(number_of_angles_);
  joint_effort_command_.resize(number_of_angles_);
  joint_position_command_.resize(number_of_angles_);
  joint_velocity_command_.resize(number_of_angles_);
#ifdef USE_PR2_CONTROLLER
  pr2_controllers_loaded_.resize(number_of_angles_, false);
#endif

  // Initialize values
  for(unsigned int j = 0; j < number_of_angles_; j++) {
    std::string jointname = use_joints[j];
    cnoid::Link* joint = cnoid_body->link(jointname);
    // Add data from transmission
    joint_position_[j]         = joint->q(); // initialize
    joint_position_command_[j] = joint->q(); //
    joint_velocity_[j]         = joint->dq();
    joint_velocity_command_[j] = joint->dq();
    joint_effort_[j]           = joint->u();  // N/m for continuous joints
    joint_effort_command_[j]   = joint->u();

    ROS_INFO("joint: %s / [initial] q: %f, dq: %f, u: %f / [gain] P: %f, D: %f",
             jointname.c_str(), joint->q(), joint->dq(), joint->u(),
             p_gain[j], d_gain[j]);
#ifndef USE_PR2_CONTROLLER
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        jointname, &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    joint_control_methods_[j] = POSITION;
    hardware_interface::JointHandle joint_handle =
      hardware_interface::JointHandle(js_interface_.getHandle(jointname),
                                      &joint_position_command_[j]);
    pj_interface_.registerHandle(joint_handle);

    joint_limits_interface::JointLimits limits;
    //const bool urdf_limits_ok = joint_limits_interface::getJointLimits(model.getJoint(jointname), limits);
    //if (!urdf_limits_ok) {
    //ROS_WARN("urdf limits of joint %s is not defined", jointname.c_str());
    //}
    limits.min_position = joint->q_upper();
    limits.max_position = joint->q_lower();
    limits.has_position_limits = true;
    limits.max_velocity = joint->dq_upper();
    limits.has_velocity_limits = true;
    //limits.max_acceleration = ;
    //limits.has_acceleration_limits = ;
    //limits.max_effort = ;
    //limits.has_effort_limits = ;
    // Register handle in joint limits interface
    joint_limits_interface::PositionJointSaturationHandle
      limits_handle(joint_handle, // We read the state and read/write the command
                    limits);       // Limits spec
    pj_sat_interface_.registerHandle(limits_handle);
#endif
  }

#ifndef USE_PR2_CONTROLLER
  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  //registerInterface(&vj_interface_);
  //registerInterface(&ej_interface_);
#else
  std::string robot_description_string;
  TiXmlDocument robot_description_xml;
  if (root_nh.getParam("robot_description", robot_description_string))
    robot_description_xml.Parse(robot_description_string.c_str());
  else
    {
      ROS_ERROR("Could not load the robot description from the parameter server");
      return false;
    }
  TiXmlElement *robot_description_root = robot_description_xml.FirstChildElement("robot");
  if (!robot_description_root)
    {
      ROS_ERROR("Could not parse the robot description");
      return false;
    }
  // Constructs the actuators by parsing custom xml.
  TiXmlElement *xit = NULL;
  for (xit = robot_description_root->FirstChildElement("transmission"); xit;
       xit = xit->NextSiblingElement("transmission"))
    {
      std::string type(xit->Attribute("type"));

      if ( type == "pr2_mechanism_model/SimpleTransmission" )
        {
          std::string joint_name = xit->FirstChildElement("joint")->Attribute("name");
          if (std::find(use_joints.begin(), use_joints.end(), joint_name) != use_joints.end())
            {
              std::string actuator_name = xit->FirstChildElement("actuator")->Attribute("name");
              actuators_[actuator_name] = new pr2_hardware_interface::Actuator(actuator_name);
            }
        }
      else if ( type == "pr2_mechanism_model/WristTransmission" )
        {
          std::string flex_joint_name = xit->FirstChildElement("flexJoint")->Attribute("name");
          std::string roll_joint_name = xit->FirstChildElement("rollJoint")->Attribute("name");
          if (std::find(use_joints.begin(), use_joints.end(), flex_joint_name) != use_joints.end()
              && std::find(use_joints.begin(), use_joints.end(), roll_joint_name) != use_joints.end())
            {
              std::string right_actuator_name = xit->FirstChildElement("rightActuator")->Attribute("name");
              std::string left_actuator_name = xit->FirstChildElement("leftActuator")->Attribute("name");
              actuators_[right_actuator_name] = new pr2_hardware_interface::Actuator(right_actuator_name);
              actuators_[left_actuator_name] = new pr2_hardware_interface::Actuator(left_actuator_name);
            }
        }
      else if ( type == "pr2_mechanism_model/PR2GripperTransmission" )
        {
          std::string joint_name = xit->FirstChildElement("gap_joint")->Attribute("name");
          if (std::find(use_joints.begin(), use_joints.end(), joint_name) != use_joints.end())
            {
              std::string actuator_name = xit->FirstChildElement("actuator")->Attribute("name");
              actuators_[actuator_name] = new pr2_hardware_interface::Actuator(actuator_name);
            }
        }
      else
        {
          ROS_WARN("Transmission type %s is not supported by CnoidRobotHW.  ", type.c_str());
        }
    }
#endif

  return true;
}

void CnoidRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  // copy choreonoid body from ...
  for(int i = 0; i < use_joints.size(); ++i) {
    cnoid::Link* joint = cnoid_body->link(use_joints[i]);
    joint_position_[i] = joint->q();
    joint_velocity_[i] = joint->dq();
    joint_effort_[i]   = joint->u();
#ifdef USE_PR2_CONTROLLER
    pr2_mechanism_model::JointState* js = state_->getJointState(joint->name());
    if(!js)ROS_WARN("Joint %s is not found in URDF", joint->name().c_str());
    js->position_ = joint_position_[i];
    js->velocity_ = joint_velocity_[i];
    js->measured_effort_ = joint_effort_[i];
#endif
  }

#ifdef USE_PR2_CONTROLLER
  state_->propagateJointPositionToActuatorPosition();
#endif
  return;
}

void CnoidRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  // write choreonoid body to ...
  /// command ???
  // flipper: 4
  // jaco:    6
  // hand:    3
  //ROS_INFO("tm: %f", time.toSec());
#ifdef USE_PR2_CONTROLLER
  current_time_ = time;
  state_->propagateActuatorEffortToJointEffort();
#endif

  for(int i = 0; i < use_joints.size(); ++i) {
    cnoid::Link* joint = cnoid_body->link(use_joints[i]);
    double tq;

    double pgain = p_gain[i];
    //double igain = i_gain[i];
    double dgain = d_gain[i];

    tq = pgain*(joint_position_command_[i] - joint->q()) - dgain*joint->dq();

#ifdef USE_PR2_CONTROLLER
    // Overwrite torque if pr2_controllers have been loaded.
    pr2_mechanism_model::JointState* js = state_->getJointState(joint->name());
    if(!js)ROS_WARN("Joint %s is not found in URDF", joint->name().c_str());
    if( js->commanded_effort_ != 0 ) pr2_controllers_loaded_[i] = true;
    if(pr2_controllers_loaded_[i]) tq = js->commanded_effort_;
#endif

    //ROS_INFO("%f = %f %f %f", tq, joint_position_command_[i], joint->q(), joint->dq());
    joint->u() = tq;
    //joint->u() = 0.0;
  }
  return;
}
}
