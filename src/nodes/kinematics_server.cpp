/**
 * \file kinematics_server.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 7 1, 2013
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
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <hubo.h>
#include <HuboKin.h>

#include "hubo_motion_ros/PoseConverter.h"
#include "hubo_motion_ros/drchubo_joint_names.h"

typedef Eigen::Matrix< float, 6, 1 > Vector6f;
Vector6f q;
HK::HuboKin kinematics;

bool ikCallback(moveit_msgs::GetPositionIK::Request& request, moveit_msgs::GetPositionIK::Response& response)
{
	int arm = 0;
	Eigen::Isometry3f proposal = hubo_motion_ros::toIsometry(request.ik_request.pose_stamped.pose);
	if (request.ik_request.group_name == "left_arm")
	{
		arm = LEFT;
	}
	else if (request.ik_request.group_name == "right_arm")
	{
		arm = RIGHT;
	}
	else
	{
		ROS_ERROR("Group name '%s' is unknown.", request.ik_request.group_name.c_str());
		return false;
	}

	kinematics.armIK(q, proposal, HK::Vector6f::Zero(), arm);

	sensor_msgs::JointState js;
	int origin = 0;
	if (LEFT == arm)
	{
		origin = LSP;
	}
	else if (RIGHT == arm)
	{
		origin = RSP;
	}

	for (int i = origin; i < origin+7; i++)
	{
		if ( i == RWR || i == LWR ) { continue; }
		js.position.push_back(q[DRCHUBO_JOINT_INDEX_TO_LIMB_POSITION[i]]);
		js.name.push_back(DRCHUBO_URDF_JOINT_NAMES[i]);
	}

	response.solution.joint_state = js;
	response.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

	return true;
}

bool fkCallback(moveit_msgs::GetPositionFK::Request& request, moveit_msgs::GetPositionFK::Response& response)
{
	Eigen::VectorXf qLeft = Eigen::VectorXf::Ones(6);
	Eigen::VectorXf qRight = Eigen::VectorXf::Ones(6);
	Eigen::Isometry3f resultLeft, resultRight;
	bool computeLeft = false;
	bool computeRight = false;

	// Check which arms we're supposed to compute
	for (int i = 0; i < request.fk_link_names.size(); i++)
	{
		if (request.fk_link_names[i] == "left_arm")
		{
			computeLeft = true;
		}
		else if (request.fk_link_names[i] == "right_arm")
		{
			computeRight = true;
		}
		else
		{
			ROS_ERROR("Group name '%s' is unknown.", request.fk_link_names[i].c_str());
		}
	}

	// Populate structs to send to HuboKin
	for (int i = 0; i < request.robot_state.joint_state.name.size(); i++)
	{
		std::string jointName = request.robot_state.joint_state.name[i];
		std::map<std::string, unsigned>::const_iterator iter = DRCHUBO_JOINT_NAME_TO_LIMB.find(
					jointName);
		if (iter == DRCHUBO_JOINT_NAME_TO_LIMB.end())
		{
			ROS_ERROR("Joint name '%s' is unknown.", jointName.c_str());
			continue;
		}
		int limb = iter->second;

		if (LEFT == limb)
		{
			computeLeft = true;
			qLeft[DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.at(jointName)] = request.robot_state.joint_state.position[i];
		}
		else if (RIGHT == limb)
		{
			computeRight = true;
			qRight[DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.at(jointName)] = request.robot_state.joint_state.position[i];
		}
	}

	// Run FK on both limbs and push results
	if (computeLeft)
	{
		kinematics.armFK(resultLeft, qLeft, LEFT);

		geometry_msgs::PoseStamped pose;
		pose.pose = hubo_motion_ros::toPose(resultLeft);
		pose.header.frame_id = "/Body_Hip";
		pose.header.stamp = ros::Time::now();
		response.pose_stamped.push_back(pose);
		response.fk_link_names.push_back("left_arm");
	}

	if (computeRight)
	{
		kinematics.armFK(resultRight, qRight, RIGHT);

		geometry_msgs::PoseStamped pose;
		pose.pose = hubo_motion_ros::toPose(resultRight);
		pose.header.frame_id = "/Body_Hip";
		pose.header.stamp = ros::Time::now();
		response.pose_stamped.push_back(pose);
		response.fk_link_names.push_back("right_arm");
	}

	return true;
}

int main(int argc, char** argv)
{
	ROS_INFO("Started kinematics_server.");
	ros::init(argc, argv, "kinematics_server");

	ros::NodeHandle nh;

	ros::ServiceServer ikServer = nh.advertiseService("/hubo/kinematics/ik_service", ikCallback);
	ros::ServiceServer fkServer = nh.advertiseService("/hubo/kinematics/fk_service", fkCallback);

	ros::spin();
}

