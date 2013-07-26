/**
 * \file fullbody_teleop.cpp
 * \brief
 *
 * \author Andrew Price
 * \date July 19, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab Georgia Institute of Technology
 * Director: Mike Stilman http://www.golems.org
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/GripperCommandAction.h>

#include <interactive_markers/interactive_marker_server.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include <hubo_robot_msgs/JointTrajectoryAction.h>
#include "hubo_motion_ros/drchubo_joint_names.h"
#include "hubo_motion_ros/PoseConverter.h"

#include "joystick_integrator/JoystickIntegrator.h"

#include <RobotKin/Robot.h>

ros::Subscriber gJoySubscriber;
ros::ServiceClient gIKinClient;
ros::ServiceClient gFKinClient;
ros::Publisher gStatePublisher;
ros::Publisher gRPosePublisher;

volatile bool mouseInUse = false;
Eigen::AngleAxisf prevAA;

sensor_msgs::JointState planState;
sensor_msgs::Joy prevJoy;
JoystickIntegrator joyInt("/Body_TSY");
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> gIntServer;

RobotKin::Robot hubo;

bool gripperStateClosed = true;

void joyCallback(const sensor_msgs::JoyPtr joy)
{
	if (mouseInUse)
	{ return; }
	// Update the pose based on the joystick
	joyInt.spacenavUpdate(joy);
	gRPosePublisher.publish(joyInt.currentPose);
	return;

	// Call IK to get the joint states
	moveit_msgs::GetPositionIKRequest req;
	req.ik_request.group_name = "right_arm";
	req.ik_request.pose_stamped = joyInt.currentPose;
	req.ik_request.robot_state.joint_state = planState;

	moveit_msgs::GetPositionIKResponse resp;
	gIKinClient.call(req, resp);

	// Check for valid solution and update the full plan
	if (resp.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
	{
		// Assign all of the new solution joints while preserving the existing ones
		for (int i = 0; i < resp.solution.joint_state.name.size(); i++)
		{
			// Locate the index of the solution joint in the plan state
			for (int j = 0; j < planState.name.size(); j++)
			{
				if (resp.solution.joint_state.name[i] == planState.name[j])
				{
					planState.position[j] = resp.solution.joint_state.position[i];
					if (resp.solution.joint_state.velocity.size() > i)
						planState.velocity[j] = resp.solution.joint_state.velocity[i];
					if (resp.solution.joint_state.effort.size() > i)
						planState.effort[j] = resp.solution.joint_state.effort[i];
				}
			}
		}

		// Time and Frame stamps
		planState.header.frame_id = "/Body_TSY";
		planState.header.stamp = ros::Time::now();

		gStatePublisher.publish(planState);
	}

	// Check for button presses
    if (prevJoy.buttons.size() > 0 && joy->buttons.size() > 0)
    {
        if (prevJoy.buttons[0] == 0 && joy->buttons[0] != 0)
        {
            hubo_robot_msgs::JointTrajectoryGoal goal;
            trajectory_msgs::JointTrajectoryPoint tPoint;
            for (int i = 0; i < planState.name.size(); i++)
            {
                goal.trajectory.joint_names.push_back(planState.name[i]);
                tPoint.positions.push_back(planState.position[i]);
            }
            goal.trajectory.points.push_back(tPoint);
            actionlib::SimpleActionClient<hubo_robot_msgs::JointTrajectoryAction> ac("/hubo_trajectory_server_joint", true);
            ac.waitForServer();
            ac.sendGoal(goal);
            bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
        }

		if (prevJoy.buttons[1] == 0 && joy->buttons[1] != 0)
		{
			control_msgs::GripperCommandGoal goal;
			if (gripperStateClosed)
			{
				goal.command.position = -1.0;
			}
			else
			{
				goal.command.position = 1.0;
			}
			actionlib::SimpleActionClient<control_msgs::GripperCommandAction> ac("/hubo_trajectory_server_gripper", true);
			ac.waitForServer();
			ac.sendGoal(goal);
			bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
		}
    }

	prevJoy = *joy;
	gRPosePublisher.publish(joyInt.currentPose);
}



void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
	<< " / control '" << feedback->control_name << "'";

  switch ( feedback->event_type )
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
	  mouseInUse = true;
	  break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
  {
	  // Compute FK for end effectors that have changed
	  // Call IK to get the joint states
	  moveit_msgs::GetPositionFKRequest req;
	  req.fk_link_names.push_back("RightArm");
	  req.robot_state.joint_state = planState;
	  req.header.stamp = ros::Time::now();

	  moveit_msgs::GetPositionFKResponse resp;
	  gFKinClient.call(req, resp);

	  std::cerr << "Response: " << resp.pose_stamped[0] << std::endl;

	  if (resp.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
	  {
		  joyInt.currentPose = resp.pose_stamped[0];
	  }


	  mouseInUse = false;
	  break;
  }

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
  {
	  Eigen::AngleAxisf aa;
	  aa = Eigen::Quaternionf(feedback->pose.orientation.w,
							  feedback->pose.orientation.x,
							  feedback->pose.orientation.y,
							  feedback->pose.orientation.z);

	  RobotKin::Joint& targetJoint = hubo.joint(feedback->marker_name);
	  float angle;
	  // Get sign of angle
	  if (aa.axis().dot(targetJoint.getJointAxis().cast<float>()) < 0)
	  {
		  angle = -aa.angle();
	  }
	  else
	  {
		  angle = aa.angle();
	  }

	  // Trim angle to joint limits
	  if (angle > targetJoint.max())
	  {
		  angle = targetJoint.max();
	  }
	  else if (angle < targetJoint.min())
	  {
		  angle = targetJoint.min();
	  }

	  // Locate the index of the solution joint in the plan state
	  for (int j = 0; j < planState.name.size(); j++)
	  {
		  if (feedback->marker_name == planState.name[j])
		  {
			  planState.position[j] = angle;
		  }
	  }

	  prevAA = aa;

	  // Time and Frame stamps
	  planState.header.frame_id = "/Body_TSY";
	  planState.header.stamp = ros::Time::now();

	  gStatePublisher.publish(planState);
	  break;
  }
  }

  gIntServer->applyChanges();
}

void makeJointMarker(std::string jointName)
{
	// RobotKin provides a reasonably easy way to access parsed URDF
	RobotKin::Joint& targetJoint = hubo.joint(jointName);
	RobotKin::Joint& parentJoint = targetJoint.parentJoint();

	// The marker must be created in the parent frame so you don't get feedback when you move it
	visualization_msgs::InteractiveMarker marker;
	if (parentJoint.name() == "invalid")
	{
		marker.header.frame_id = "/Body_Torso";
	}
	else
	{
		marker.header.frame_id = "/Body_" + parentJoint.name();//jointName;
	}

	marker.scale = .125;
	marker.name = jointName;

	geometry_msgs::Pose controlPose = hubo_motion_ros::toPose( targetJoint.respectToFixed().cast<float>());
	marker.pose = controlPose;

	visualization_msgs::InteractiveMarkerControl control;

	Eigen::Quaternionf jointAxis;
	jointAxis.setFromTwoVectors(Eigen::Vector3f::UnitX(), targetJoint.getJointAxis().cast<float>());

	control.orientation.w = jointAxis.w();
	control.orientation.x = jointAxis.x();
	control.orientation.y = jointAxis.y();
	control.orientation.z = jointAxis.z();

	control.always_visible = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

	marker.controls.push_back(control);

	gIntServer->insert(marker);
	gIntServer->setCallback(marker.name, &processFeedback);
}

int main(int argc, char** argv)
{
	ROS_INFO("Started fullbody_teleop.");
	ros::init(argc, argv, "fullbody_teleop");

	ros::NodeHandle m_nh;

	std::string robotDescription;
	if (!m_nh.getParam("/robot_description", robotDescription))
	{
		ROS_FATAL("Parameter for robot description not provided");
	}
	hubo.loadURDFString(robotDescription);

	//ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

	gIntServer.reset( new interactive_markers::InteractiveMarkerServer("joint_controls","",false) );

	for (int i = 0; i < hubo.linkages().size(); i++)
	{
		RobotKin::Linkage* linkage = hubo.linkages()[i];
		std::cerr << linkage->name();
		for (int j = 0; j < linkage->joints().size(); j++)
		{
			RobotKin::Joint* joint = linkage->joints()[j];
			if (joint->name()[1] == 'F') { continue; }
			if (joint->name() == "TSY") { continue; }
			makeJointMarker(joint->name());
		}
	}
	//makeJointMarker("NKY");
	//makeJointMarker("NK1");
	//makeJointMarker("NK2");

	ros::Duration(0.1).sleep();

	for (int i = 0; i < HUBO_JOINT_COUNT; i++)
	{
		if (DRCHUBO_URDF_JOINT_NAMES[i] != "")
		{
			planState.name.push_back(DRCHUBO_URDF_JOINT_NAMES[i]);
			planState.position.push_back(0);
			planState.velocity.push_back(0);
			planState.effort.push_back(0);
		}
	}

    for (int side = 0; side < 2; side++)
        for (int i = 1; i <= 3; i++)
            for (int j = 1; j <= 3; j++)
            {
                std::string sideStr = (side == 0) ? "R" : "L";
                planState.name.push_back(sideStr + "F" + std::to_string(i) + std::to_string(j));
                planState.position.push_back(0);
                planState.velocity.push_back(0);
                planState.effort.push_back(0);
            }

	gIntServer->applyChanges();

	gJoySubscriber = m_nh.subscribe("joy_in", 1, &joyCallback);
	gIKinClient = m_nh.serviceClient<moveit_msgs::GetPositionIK>("/hubo/kinematics/ik_service");
	gFKinClient = m_nh.serviceClient<moveit_msgs::GetPositionFK>("/hubo/kinematics/fk_service");
	gStatePublisher = m_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	gRPosePublisher = m_nh.advertise<geometry_msgs::PoseStamped>("rh_pose", 1);

	ros::spin();

	gIntServer.reset();
	return 0;
}
