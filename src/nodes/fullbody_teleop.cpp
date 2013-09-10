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

#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <hubo_robot_msgs/JointTrajectory.h>
#include <hubo_motion_ros/ExecuteGripperAction.h>
#include <control_msgs/GripperCommandAction.h>

#include <interactive_markers/interactive_marker_server.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


#include <hubo_robot_msgs/JointTrajectoryAction.h>
#include "hubo_motion_ros/drchubo_joint_names.h"
#include "hubo_motion_ros/PoseConverter.h"
#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/TeleopCmd.h"
#include "hubo_motion_ros/TeleopPoseNudge.h"

#include "joystick_integrator/JoystickIntegrator.h"

#include <urdf/model.h>


ros::Subscriber gJoySubscriber;
ros::Subscriber gCmdSubscriber;
ros::Subscriber gNudgeSubscriber;
ros::Subscriber gStateSubscriber;
ros::ServiceClient gIKinClient;
ros::ServiceClient gFKinClient;
ros::Publisher gStatePublisher;
ros::Publisher gRPosePublisher;
ros::Publisher gTextPublisher;
ros::Timer gTimer;

volatile bool mouseInUse = false;
Eigen::AngleAxisf prevAA;

sensor_msgs::JointState planState;
sensor_msgs::Joy prevJoy;

sensor_msgs::JointState lastPlanState;
geometry_msgs::PoseStamped lastPose[2];
Eigen::Isometry3d offsetPose;

//JoystickIntegrator joyInt("/Plan_RAR");
JoystickIntegrator joyInt("/planRightFoot");
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> gIntServer;

urdf::Model huboModel;
std::string baseFrame;

int armSide;
enum { DUAL_ARM = 2 };



void dualHelper()
{
    moveit_msgs::GetPositionFKRequest req;
    req.fk_link_names.push_back("RightArm");

    moveit_msgs::GetPositionFKResponse resp;
    gFKinClient.call(req, resp);

    Eigen::Isometry3d rightPose;
    Eigen::Quaterniond rightQuat(resp.pose_stamped[0].pose.orientation.w,
                                 resp.pose_stamped[0].pose.orientation.x,
                                 resp.pose_stamped[0].pose.orientation.y,
                                 resp.pose_stamped[0].pose.orientation.z);
    Eigen::Vector3d rightTrans(resp.pose_stamped[0].pose.position.x,
                               resp.pose_stamped[0].pose.position.y,
                               resp.pose_stamped[0].pose.position.z);
    rightPose.translate(rightTrans);
    rightPose.rotate(rightQuat);
    
    Eigen::Isometry3d leftPose = rightPose * offsetPose.inverse();
    Eigen::Vector3d leftTrans = leftPose.translation();
    Eigen::Quaterniond leftQuat(leftPose.rotation());
    
    
    moveit_msgs::GetPositionIKRequest ikreq;

    
    ikreq.ik_request.group_name = "left_arm";

    ikreq.ik_request.pose_stamped = joyInt.currentPose;
    
    ikreq.ik_request.pose_stamped.pose.position.x = leftTrans.x();
    ikreq.ik_request.pose_stamped.pose.position.y = leftTrans.y();
    ikreq.ik_request.pose_stamped.pose.position.z = leftTrans.z();
    
    ikreq.ik_request.pose_stamped.pose.orientation.w = leftQuat.w();
    ikreq.ik_request.pose_stamped.pose.orientation.x = leftQuat.x();
    ikreq.ik_request.pose_stamped.pose.orientation.y = leftQuat.y();
    ikreq.ik_request.pose_stamped.pose.orientation.z = leftQuat.z();
    
    ikreq.ik_request.robot_state.joint_state = planState;

    moveit_msgs::GetPositionIKResponse ikresp;
    gIKinClient.call(ikreq, ikresp);

    {
        // Assign all of the new solution joints while preserving the existing ones
        for (int i = 0; i < ikresp.solution.joint_state.name.size(); i++)
        {
            // Locate the index of the solution joint in the plan state
            for (int j = 0; j < planState.name.size(); j++)
            {
                if (ikresp.solution.joint_state.name[i] == planState.name[j])
                {
                    planState.position[j] = ikresp.solution.joint_state.position[i];
                    if (ikresp.solution.joint_state.velocity.size() > i)
                        planState.velocity[j] = ikresp.solution.joint_state.velocity[i];
                    if (ikresp.solution.joint_state.effort.size() > i)
                        planState.effort[j] = ikresp.solution.joint_state.effort[i];
                }
            }
        }

        // Time and Frame stamps
        planState.header.frame_id = baseFrame;
        planState.header.stamp = ros::Time::now();

        gStatePublisher.publish(planState);
    }
}

void jointStateCallback( const sensor_msgs::JointStateConstPtr &state )
{
    for(int i=0; i<state->name.size(); i++)
    {
        if(DRCHUBO_JOINT_NAME_TO_LIMB.find(state->name[i]) !=
                                            DRCHUBO_JOINT_NAME_TO_LIMB.end())
        {
            for(int legSide=0; legSide<2; legSide++)
            {
                if(DRCHUBO_JOINT_NAME_TO_LIMB.at(state->name[i]) == legSide+2)
                {
                    for(int j=0; j<planState.name.size(); j++)
                    {
                        if(state->name[i] == planState.name[j])
                        {
                            planState.position[j] = state->position[i];
                            lastPlanState.position[j] = state->position[i];
                        }
                    }
                }
            }
        }
        else if(state->name[i] == "TSY")
        {
            for(int j=0; j<planState.name.size(); j++)
            {
                if(state->name[i] == planState.name[j])
                {
                    planState.position[j] = state->position[i];
                    lastPlanState.position[j] = state->position[i];
                }
            }
        }
    }
}

void publishJointResults()
{
    // Call IK to get the joint states
    moveit_msgs::GetPositionIKRequest req;

    if(armSide==RIGHT || armSide==DUAL_ARM)
        req.ik_request.group_name = "right_arm";
    else if(armSide==LEFT)
        req.ik_request.group_name = "left_arm";
    // TODO: Handle dual arm as well


    req.ik_request.pose_stamped = joyInt.currentPose;
    req.ik_request.robot_state.joint_state = planState;

    moveit_msgs::GetPositionIKResponse resp;
    gIKinClient.call(req, resp);

    // Check for valid solution and update the full plan
//		if (resp.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
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
        planState.header.frame_id = baseFrame;
        planState.header.stamp = ros::Time::now();

        gStatePublisher.publish(planState);
    }
    
    
    if(armSide == DUAL_ARM)
        dualHelper();
}


void placeJoystick()
{
    moveit_msgs::GetPositionFKRequest req;
    req.robot_state.joint_state = planState;
    req.header.stamp = ros::Time::now();

    if(armSide==RIGHT || armSide==DUAL_ARM)
        req.fk_link_names.push_back("RightArm");
    else if(armSide==LEFT)
        req.fk_link_names.push_back("LeftArm");

    moveit_msgs::GetPositionFKResponse resp;
    gFKinClient.call(req, resp);

    if (resp.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS
            && resp.pose_stamped.size() > 0)
    {
		joyInt.setPose(resp.pose_stamped[0]);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to solve FK: " << resp.error_code.val);
    }

    gRPosePublisher.publish(joyInt.currentPose);
}


void savePlanState()
{
    if(armSide==RIGHT || armSide==LEFT)
    {
        for(int i=0; i<planState.name.size(); i++)
        {
            if(DRCHUBO_JOINT_NAME_TO_LIMB.find(planState.name[i]) !=
                                                DRCHUBO_JOINT_NAME_TO_LIMB.end())
                if(DRCHUBO_JOINT_NAME_TO_LIMB.at(planState.name[i]) == armSide)
                    lastPlanState.position[i] = planState.position[i];
        }
    }
}

void nudgeRequestCallback(const hubo_motion_ros::TeleopPoseNudge nudge)
{
    if(nudge.type == hubo_motion_ros::TeleopPoseNudge::TRANSLATION)
    {
        Eigen::Vector3f trans; trans.setZero();
        if(nudge.axis == hubo_motion_ros::TeleopPoseNudge::X_AXIS)
            trans[0] = nudge.value;
        else if(nudge.axis == hubo_motion_ros::TeleopPoseNudge::Y_AXIS)
            trans[1] = nudge.value;
        else if(nudge.axis == hubo_motion_ros::TeleopPoseNudge::Z_AXIS)
            trans[2] = nudge.value;
        else
            return;
        
        if(nudge.frame == hubo_motion_ros::TeleopPoseNudge::LOCAL)
            trans = joyInt.currentOrientation * trans;
        
        joyInt.currentPose.pose.position.x += trans.x();
        joyInt.currentPose.pose.position.y += trans.y();
        joyInt.currentPose.pose.position.z += trans.z();
    }
    else if(nudge.type == hubo_motion_ros::TeleopPoseNudge::ROTATION)
    {
        double alpha=0, beta=0, gamma=0;
        if(nudge.axis == hubo_motion_ros::TeleopPoseNudge::X_AXIS)
            alpha = nudge.value;
        else if(nudge.axis == hubo_motion_ros::TeleopPoseNudge::Y_AXIS)
            beta = nudge.value;
        else if(nudge.axis == hubo_motion_ros::TeleopPoseNudge::Z_AXIS)
            gamma = nudge.value;
        else
            return;
        
        Eigen::Quaternionf q = rpyToQ(alpha, beta, gamma);
        
        if(nudge.frame == hubo_motion_ros::TeleopPoseNudge::LOCAL)
            joyInt.currentOrientation = joyInt.currentOrientation * q;
        else if(nudge.frame == hubo_motion_ros::TeleopPoseNudge::GLOBAL)
            joyInt.currentOrientation = q * joyInt.currentOrientation;
        else
            return;
        
        joyInt.currentPose.pose.orientation.w = joyInt.currentOrientation.w();
        joyInt.currentPose.pose.orientation.x = joyInt.currentOrientation.x();
        joyInt.currentPose.pose.orientation.y = joyInt.currentOrientation.y();
        joyInt.currentPose.pose.orientation.z = joyInt.currentOrientation.z();
        
        joyInt.currentPose.header.frame_id = joyInt.frame_id;
        joyInt.currentPose.header.stamp = ros::Time::now();
    }
    else
        return;
    
    
    gRPosePublisher.publish(joyInt.currentPose);

    publishJointResults();
    
}


void sendCommandCallback(const hubo_motion_ros::TeleopCmd cmd)
{
    std::cerr << "COMMAND REQUEST RECEIVED" << std::endl;

    if(cmd.CommandType == hubo_motion_ros::TeleopCmd::END_EFFECTOR)
    {
        hubo_motion_ros::ExecutePoseTrajectoryGoal goal;
        if(armSide==RIGHT || armSide==DUAL_ARM)
        {
            lastPose[RIGHT] = joyInt.currentPose;
            goal.ArmIndex.push_back(RIGHT);
        }
        else if(armSide==LEFT)
        {
            lastPose[LEFT] = joyInt.currentPose;
            goal.ArmIndex.push_back(LEFT);
        }
        savePlanState();

        geometry_msgs::PoseArray kittens;
        kittens.header.frame_id = baseFrame;
        kittens.poses.push_back(joyInt.currentPose.pose);

        goal.PoseTargets.push_back(kittens);
        
        
        if(armSide==DUAL_ARM)
        {
            Eigen::Vector3d trans = offsetPose.translation();
            Eigen::Quaterniond quat(offsetPose.rotation());
            
            geometry_msgs::PoseStamped off = joyInt.currentPose;
            off.pose.position.x = trans.x();
            off.pose.position.y = trans.y();
            off.pose.position.z = trans.z();
            
            off.pose.orientation.w = quat.w();
            off.pose.orientation.x = quat.x();
            off.pose.orientation.y = quat.y();
            off.pose.orientation.z = quat.z();
            
            geometry_msgs::PoseArray puppies;
            puppies.header.frame_id = baseFrame;
            puppies.poses.push_back(off.pose);
            
            goal.DualOffset.push_back(puppies);
        }
        
        
        actionlib::SimpleActionClient<hubo_motion_ros::ExecutePoseTrajectoryAction> ac("/hubo_trajectory_server_pose", true);
        
        
        ac.waitForServer();
        ac.sendGoal(goal);
//        bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));
    }
    else if(cmd.CommandType == hubo_motion_ros::TeleopCmd::JOINTSPACE)
    {
        if(armSide==RIGHT)
            lastPose[RIGHT] = joyInt.currentPose;
        else if(armSide==LEFT)
            lastPose[LEFT] = joyInt.currentPose;
        savePlanState();
        
        hubo_robot_msgs::JointTrajectoryGoal goal;
        hubo_robot_msgs::JointTrajectoryPoint tPoint;
        for (int i = 0; i < planState.name.size(); i++)
        {
            goal.trajectory.joint_names.push_back(planState.name[i]);
            tPoint.positions.push_back(planState.position[i]);
        }
        goal.trajectory.points.push_back(tPoint);
        actionlib::SimpleActionClient<hubo_robot_msgs::JointTrajectoryAction> ac("/hubo_trajectory_server_joint", true);


        ac.waitForServer();
        ac.sendGoal(goal);
    }
    else if(cmd.CommandType == hubo_motion_ros::TeleopCmd::RESET)
    {
        if( armSide==LEFT || armSide==RIGHT )
        {
            joyInt.currentOrientation.w() = lastPose[armSide].pose.orientation.w;
            joyInt.currentOrientation.x() = lastPose[armSide].pose.orientation.x;
            joyInt.currentOrientation.y() = lastPose[armSide].pose.orientation.y;
            joyInt.currentOrientation.z() = lastPose[armSide].pose.orientation.z;
            joyInt.currentPose = lastPose[armSide];
        }
        gRPosePublisher.publish(joyInt.currentPose);

        if(armSide==RIGHT || armSide==LEFT)
        {
            for(int i=0; i<planState.name.size(); i++)
            {
                if(DRCHUBO_JOINT_NAME_TO_LIMB.find(planState.name[i]) !=
                                                    DRCHUBO_JOINT_NAME_TO_LIMB.end())
                    if(DRCHUBO_JOINT_NAME_TO_LIMB.at(planState.name[i]) == armSide)
                        planState.position[i] = lastPlanState.position[i];
            }
        }
        else
            planState = lastPlanState; // TODO: Make sure this works

        gStatePublisher.publish(planState);
    }
    else if(cmd.CommandType == hubo_motion_ros::TeleopCmd::ZEROS)
    {
        if(armSide==RIGHT || armSide==LEFT)
        {
            for(int i=0; i<planState.name.size(); i++)
            {
                if(DRCHUBO_JOINT_NAME_TO_LIMB.find(planState.name[i]) !=
                                                    DRCHUBO_JOINT_NAME_TO_LIMB.end())
                    if(DRCHUBO_JOINT_NAME_TO_LIMB.at(planState.name[i]) == armSide)
                        planState.position[i] = 0;
            }
        }

        placeJoystick();

    }
    else if(cmd.CommandType == hubo_motion_ros::TeleopCmd::SWITCH_LEFT)
    {
        armSide = LEFT;
        placeJoystick();
    }
    else if(cmd.CommandType == hubo_motion_ros::TeleopCmd::SWITCH_RIGHT)
    {
        armSide = RIGHT;
        placeJoystick();
    }
    else if(cmd.CommandType == hubo_motion_ros::TeleopCmd::SWITCH_DUAL)
    {
        armSide = DUAL_ARM;
        
        moveit_msgs::GetPositionFKRequest req;
        req.fk_link_names.push_back("RightArm");
    
        moveit_msgs::GetPositionFKResponse resp;
        gFKinClient.call(req, resp);
    
        Eigen::Isometry3d rightPose;
        Eigen::Quaterniond rightQuat(resp.pose_stamped[0].pose.orientation.w,
                                     resp.pose_stamped[0].pose.orientation.x,
                                     resp.pose_stamped[0].pose.orientation.y,
                                     resp.pose_stamped[0].pose.orientation.z);
        Eigen::Vector3d rightTrans(resp.pose_stamped[0].pose.position.x,
                                   resp.pose_stamped[0].pose.position.y,
                                   resp.pose_stamped[0].pose.position.z);
        rightPose.translate(rightTrans);
        rightPose.rotate(rightQuat);
        
        
        req.fk_link_names[0] = "LeftArm";
        
        gFKinClient.call(req, resp);
        
        Eigen::Isometry3d leftPose;
        Eigen::Quaterniond leftQuat(resp.pose_stamped[0].pose.orientation.w,
                                     resp.pose_stamped[0].pose.orientation.x,
                                     resp.pose_stamped[0].pose.orientation.y,
                                     resp.pose_stamped[0].pose.orientation.z);
        Eigen::Vector3d leftTrans(resp.pose_stamped[0].pose.position.x,
                                   resp.pose_stamped[0].pose.position.y,
                                   resp.pose_stamped[0].pose.position.z);
        leftPose.translate(leftTrans);
        leftPose.rotate(leftQuat);
        
        offsetPose = leftPose.inverse() * rightPose;
        
        gRPosePublisher.publish(joyInt.currentPose);
    }

}


void joyCallback(const sensor_msgs::JoyPtr joy)
{
	if (mouseInUse)
	{ return; }
	bool allZeros = true;
	for (int i = 0; i < joy->axes.size(); i++)
	{
		if (fabs(joy->axes[i]) > 0.01)
		{
			allZeros = false;
		}
	}

	if (!allZeros)
	{
		// Update the pose based on the joystick
		joyInt.spacenavUpdate(joy);
		gRPosePublisher.publish(joyInt.currentPose);

        publishJointResults();
	}
	else
	{
        // Time and Frame stamps
        planState.header.frame_id = baseFrame;
        planState.header.stamp = ros::Time::now();
		gStatePublisher.publish(planState);
	}

	// Check for button presses
	if (prevJoy.buttons.size() > 0 && joy->buttons.size() > 0)
	{
		if (prevJoy.buttons[0] == 0 && joy->buttons[0] != 0)
		{

		}

		if (prevJoy.buttons[1] == 0 && joy->buttons[1] != 0)
		{

		}
	}

	prevJoy = *joy;
	gRPosePublisher.publish(joyInt.currentPose);
}

void buttonCallback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	switch ( feedback->event_type )
	{
	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
		mouseInUse = true;
		//if (feedback->marker_name == "TestButton")
	{
		std::cerr << "Clicked!" << std::endl;
		std::ostringstream s;

		for (int i = 0; i < planState.name.size(); i++)
		{
			s << planState.name[i];
			if (i < planState.name.size() - 1)
			{
				s << ", ";
			}
		}
		s << std::endl;
		for (int i = 0; i < planState.position.size(); i++)
		{
			s << planState.position[i];
			if (i < planState.position.size() - 1)
			{
				s << ", ";
			}
		}
		s << std::endl;

		std_msgs::String jointString;
		jointString.data = s.str();

		gTextPublisher.publish(jointString);
	}
		break;
	}
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

      placeJoystick();

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

	  boost::shared_ptr<const urdf::Joint> targetJoint = huboModel.getJoint(feedback->marker_name);
	  Eigen::Vector3f axisVector = hubo_motion_ros::toEVector3(targetJoint->axis);
	  float angle;
	  // Get sign of angle
	  if (aa.axis().dot(axisVector) < 0)
	  {
		  angle = -aa.angle();
	  }
	  else
	  {
		  angle = aa.angle();
	  }

	  // Trim angle to joint limits
      if (angle > targetJoint->limits->upper)
	  {
		  angle = targetJoint->limits->upper;
	  }
	  else if (angle < targetJoint->limits->lower)
	  {
		  angle = targetJoint->limits->lower;
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
      planState.header.frame_id = baseFrame;
	  planState.header.stamp = ros::Time::now();

	  gStatePublisher.publish(planState);
	  break;
  }
  }

  gIntServer->applyChanges();
}

void makeJointMarker(std::string jointName)
{
	boost::shared_ptr<const urdf::Joint> targetJoint = huboModel.getJoint(jointName);

	// The marker must be created in the parent frame so you don't get feedback when you move it
	visualization_msgs::InteractiveMarker marker;

	marker.scale = .125;
	marker.name = jointName;
	marker.header.frame_id = targetJoint->parent_link_name;

	geometry_msgs::Pose controlPose = hubo_motion_ros::toPose( targetJoint->parent_to_joint_origin_transform);
	marker.pose = controlPose;

	visualization_msgs::InteractiveMarkerControl control;

	Eigen::Quaternionf jointAxis;
	Eigen::Vector3f axisVector = hubo_motion_ros::toEVector3(targetJoint->axis);
	jointAxis.setFromTwoVectors(Eigen::Vector3f::UnitX(), axisVector);

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

void makeSaveButton()
{
	visualization_msgs::InteractiveMarker marker;
	marker.header.frame_id = baseFrame;
	marker.scale = 0.1;

	marker.name = "TestButton";

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 1;

	visualization_msgs::InteractiveMarkerControl control;

	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
	control.always_visible = true;
	control.name = "CopyJoints";

	visualization_msgs::Marker box;
	box.type = visualization_msgs::Marker::CUBE;
	box.scale.x = 0.05;
	box.scale.y = 0.05;
	box.scale.z = 0.05;
	box.color.r = 0.5;
	box.color.g = 0.5;
	box.color.b = 0.5;
	box.color.a = 1.0;

	control.markers.push_back(box);

	marker.controls.push_back(control);

	gIntServer->insert(marker);
	gIntServer->setCallback(marker.name, &buttonCallback);
}

void timerCallback(const ros::TimerEvent&)
{	
	planState.header.frame_id = baseFrame;
	planState.header.stamp = ros::Time::now();
	gStatePublisher.publish(planState);
	gRPosePublisher.publish(joyInt.currentPose);
}

int main(int argc, char** argv)
{
	ROS_INFO("Started fullbody_teleop.");
	ros::init(argc, argv, "fullbody_teleop");

    armSide = RIGHT;
    joyInt.settings.useLocalRotations = false;
    joyInt.settings.useLocalTranslations = false;

	ros::NodeHandle nh;

	std::string robotDescription;
	if (!nh.getParam("/robot_description", robotDescription))
	{
		ROS_FATAL("Parameter for robot description not provided");
	}

	// Use the planning model instead of real one
	boost::replace_all(robotDescription, "Body_", "/Plan_");
    boost::replace_all(robotDescription, "rightFoot", "/planRightFoot");
    boost::replace_all(robotDescription, "leftFoot", "/planLeftFoot");
    boost::replace_all(robotDescription, "rightPalm", "/planRightPalm");
    boost::replace_all(robotDescription, "leftPalm", "/planLeftPalm");
	nh.setParam("/robot_planning_description", robotDescription);

	huboModel.initString(robotDescription);
//	baseFrame = huboModel.getRoot()->name;
//    baseFrame = "Plan_RAR";
    baseFrame = "planRightFoot";

	//ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

	gIntServer.reset( new interactive_markers::InteractiveMarkerServer("joint_controls","",false) );

	for (auto jointPair : huboModel.joints_)
	{
		boost::shared_ptr<urdf::Joint> joint = jointPair.second;
		if (joint->name[1] == 'F') { continue; }
		if (joint->name== "TSY") { continue; }
		makeJointMarker(joint->name);
	}
	std::cerr << "\nURDF size: " << huboModel.joints_.size() << std::endl;

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


	makeSaveButton();
	gIntServer->applyChanges();

	gJoySubscriber = nh.subscribe("joy_in", 1, &joyCallback);
    gCmdSubscriber = nh.subscribe("teleop_cmd_req", 1, &sendCommandCallback);
    gNudgeSubscriber = nh.subscribe("teleop_nudge_req", 1, &nudgeRequestCallback);
    gStateSubscriber = nh.subscribe("/hubo/joint_states", 1, &jointStateCallback);
	gIKinClient = nh.serviceClient<moveit_msgs::GetPositionIK>("/hubo/kinematics/ik_service");
	gFKinClient = nh.serviceClient<moveit_msgs::GetPositionFK>("/hubo/kinematics/fk_service");
	gStatePublisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
	gRPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("rh_pose", 1);
	gTextPublisher = nh.advertise<std_msgs::String>("text_out", 1);

	gTimer = nh.createTimer(ros::Duration(1), &timerCallback);
	gTimer.start();

    lastPlanState = planState;

    armSide = LEFT;
    savePlanState();
    placeJoystick();
    lastPose[LEFT] = joyInt.currentPose;
    armSide = RIGHT;
    placeJoystick();
    savePlanState();
    lastPose[RIGHT] = joyInt.currentPose;

	ros::spin();

	gIntServer.reset();
	return 0;
}
