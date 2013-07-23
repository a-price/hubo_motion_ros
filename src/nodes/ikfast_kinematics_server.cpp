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

#include "hubo_motion_ros/PoseConverter.h"
#include "hubo_motion_ros/drchubo_joint_names.h"
#include "hubo_motion_ros/DrcHuboKin.h"

DrcHuboKin* kinematics;

#define IKFAST_HAS_LIBRARY
#include "ikfast.h"

using namespace ikfast;

bool ComputeIkFast(int arm, DrcHuboKin::ArmVector& p, Eigen::Isometry3d B) {
	Eigen::Matrix<IkReal, 3, 3, Eigen::RowMajor> eerot = B.rotation();
	Eigen::Matrix<IkReal, 3, 1> eetrans = B.translation();

	IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
	int free = *GetFreeParameters();    
    for(std::size_t i = 0; i < vfree.size(); ++i)
        vfree[i] = p[free];
    bool bSuccess = ComputeIk(eetrans.data(), eerot.data(), vfree.size() > 0 ? &vfree[0] : NULL, solutions);

    if( !bSuccess ) {
        fprintf(stderr,"Failed to get ik solution\n");
        return bSuccess;
    }

    Eigen::VectorXd q(GetNumJoints());
    Eigen::MatrixXd QAll(GetNumJoints(), (int)solutions.GetNumSolutions());
    printf("Found %d ik solutions:\n", (int)solutions.GetNumSolutions());    
    std::vector<IkReal> solvalues(GetNumJoints());
    for(std::size_t i=0; i < solutions.GetNumSolutions(); ++i) {
        const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        printf("sol%d (free=%d): ", (int)i, (int)sol.GetFree().size());
        std::vector<IkReal> vsolfree(sol.GetFree().size());
        sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
        for( std::size_t j = 0; j < solvalues.size(); ++j) {
        	QAll(j, i) = solvalues[j];
            printf("%.15f, ", solvalues[j]);
        }
        printf("\n");
    }

    IkReal nearest = std::numeric_limits<double>::max();
    int index = -1;
    for(int i=0; i < QAll.cols(); ++i) {
    	IkReal n = (QAll.col(i) - p).norm();
    	if(index == -1 || n < nearest) { 
    		index = i;
    		nearest = n;
    	}
    }
    p = QAll.col(index);

    return bSuccess;
}

bool ikCallback(moveit_msgs::GetPositionIK::Request& request, moveit_msgs::GetPositionIK::Response& response)
{
	int arm = 0;
	Eigen::Isometry3d proposal = hubo_motion_ros::toIsometry(request.ik_request.pose_stamped.pose).cast<double>();
	DrcHuboKin::ArmVector q = DrcHuboKin::ArmVector::Zero();

	// Get the arm (group)
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
		response.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
		return false;
	}

	// Get the seed state
	sensor_msgs::JointState& seedState = request.ik_request.robot_state.joint_state;
	for (int i = 0; i < seedState.name.size(); i++)
	{
		auto limbIter = DRCHUBO_JOINT_NAME_TO_LIMB.find(seedState.name[i]);
		if (DRCHUBO_JOINT_NAME_TO_LIMB.end() == limbIter) { continue; }

		if (limbIter->second == arm)
		{
			q[DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.at(seedState.name[i])] = seedState.position[i];
		}
	}

	// RobotKin::rk_result_t result = kinematics->armIK(arm, q, proposal);
	bool ok = ComputeIkFast(arm, q, proposal);

	sensor_msgs::JointState js;
	int baseJoint = 0;
	if (LEFT == arm)
	{
		baseJoint = LSP;
	}
	else if (RIGHT == arm)
	{
		baseJoint = RSP;
	}

	for (int i = baseJoint; i < baseJoint+7; i++)
	{
		//if ( i == RWR || i == LWR ) { continue; }
		js.position.push_back(q[DRCHUBO_JOINT_INDEX_TO_LIMB_POSITION[i]]);
		js.name.push_back(DRCHUBO_URDF_JOINT_NAMES[i]);
	}

	response.solution.joint_state = js;
	if(ok)
		response.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
	else
		response.error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;

	return true;
}

bool fkCallback(moveit_msgs::GetPositionFK::Request& request, moveit_msgs::GetPositionFK::Response& response)
{
	Eigen::Isometry3d resultFrame;

	// Populate the joint states
	for (int i = 0; i < request.robot_state.joint_state.name.size(); i++)
	{
		sensor_msgs::JointState& jointState = request.robot_state.joint_state;

		// Check whether joint is legitimate
		if (kinematics->joint(jointState.name[i]).name() != "invalid")
		{
			kinematics->joint(jointState.name[i]).value(jointState.position[i]);
		}
		else
		{
			ROS_ERROR("Joint name '%s' is unknown with new name '%s'.", jointState.name[i].c_str(), kinematics->joint(jointState.name[i]).name().c_str());
			response.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
			return false;
		}
	}

	for (int i = 0; i < request.fk_link_names.size(); i++)
	{
		if (kinematics->linkage(request.fk_link_names[i]).name() != "invalid")
		{
			resultFrame = kinematics->linkage(request.fk_link_names[i]).tool().respectToRobot();

			geometry_msgs::PoseStamped pose;
			pose.pose = hubo_motion_ros::toPose(resultFrame);
			pose.header.frame_id = "/Body_TSY";
			pose.header.stamp = ros::Time::now();
			response.pose_stamped.push_back(pose);
			response.fk_link_names.push_back(request.fk_link_names[i]);
		}
		else
		{
			ROS_ERROR("Group name '%s' is unknown.", request.fk_link_names[i].c_str());
			response.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
			return false;
		}
	}

	return true;
}

int main(int argc, char** argv)
{
	ROS_INFO("Started kinematics_server.");
	ros::init(argc, argv, "kinematics_server");

	ros::NodeHandle nh;

	std::string robotDescription;
	if (!nh.getParam("robot_description", robotDescription))
	{
		ROS_ERROR("Could not find robot description.");
		return -1;
	}

	kinematics = new DrcHuboKin(robotDescription, true);

	ros::ServiceServer ikServer = nh.advertiseService("/hubo/kinematics/ik_service", ikCallback);
	ros::ServiceServer fkServer = nh.advertiseService("/hubo/kinematics/fk_service", fkCallback);

	ros::spin();
}

