/**
 *
 * \file test_manipulation_server.cpp
 * \brief Subsystem test for verifying operation of the manipulation and control daemons
 *
 * \author Andrew Price
 * \date May 21, 2013
 *
 * \copyright
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab Georgia Institute of Technology
 * Director: Mike Stilman http://www.golems.org
 *
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

#include <manip.h>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hubo_motion_ros/AchROSBridge.h"
#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/ExecuteJointTrajectoryAction.h"

hubo_motion_ros::ExecutePoseTrajectoryGoal createSimplePoseGoal()
{
	hubo_motion_ros::ExecutePoseTrajectoryGoal goal;
	geometry_msgs::PoseArray pArrayL, pArrayR;
	geometry_msgs::Pose pose;

	pArrayR.header.frame_id = "/world";
	pArrayL.header.frame_id = "/world";

	goal.ArmIndex.push_back(0);
	goal.ArmIndex.push_back(1);


	pose.position.x = 0.25;
	pose.position.y = -0.25;
	pose.position.z = -0.2;

	pose.orientation.w = 1.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;

	pArrayR.poses.push_back(pose);
	goal.PoseTargets.push_back(pArrayR);

	pose.position.y = -pose.position.y;
	pArrayL.poses.push_back(pose);
	goal.PoseTargets.push_back(pArrayL);

	return goal;
}

hubo_motion_ros::ExecutePoseTrajectoryGoal createTrajectoryPoseGoal()
{
	hubo_motion_ros::ExecutePoseTrajectoryGoal goal;
	geometry_msgs::PoseArray pArrayL, pArrayR;
	geometry_msgs::Pose poseA0,poseA1,poseA2,poseB0,poseB1,poseB2;

	pArrayR.header.frame_id = "/world";
	pArrayL.header.frame_id = "/world";


	poseA0.position.x = 0.0;
	poseA0.position.y = -0.45;
	poseA0.position.z = -0.25;

	Eigen::Quaternionf q; q.setFromTwoVectors(Eigen::Vector3f::UnitX(), -Eigen::Vector3f::UnitZ());
	poseA0.orientation.w = q.w();
	poseA0.orientation.x = q.x();
	poseA0.orientation.y = q.y();
	poseA0.orientation.z = q.z();

	pArrayR.poses.push_back(poseA0);
	poseB0 = poseA0;
	poseB0.position.y = -poseB0.position.y;
	pArrayL.poses.push_back(poseB0);


	poseA1.position.x = 0.25;
	poseA1.position.y = -0.25;
	poseA1.position.z = -0.2;

	poseA1.orientation.w = 1.0;
	poseA1.orientation.x = 0.0;
	poseA1.orientation.y = 0.0;
	poseA1.orientation.z = 0.0;

	pArrayR.poses.push_back(poseA1);
	poseB1 = poseA1;
	poseB1.position.y = -poseB1.position.y;
	pArrayL.poses.push_back(poseB1);

//	pose.position.y = -pose.position.y;
	poseA2.position.x = 0.0;
	poseA2.position.y = -0.45;
	poseA2.position.z = 0.25;

	q.setFromTwoVectors(Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitZ());
	poseA2.orientation.w = q.w();
	poseA2.orientation.x = q.x();
	poseA2.orientation.y = q.y();
	poseA2.orientation.z = q.z();

	pArrayR.poses.push_back(poseA2);
	poseB2 = poseA2;
	poseB2.position.y = -poseB2.position.y;
	pArrayL.poses.push_back(poseB2);

	goal.ArmIndex.push_back(0);
	goal.ArmIndex.push_back(1);

	goal.PoseTargets.push_back(pArrayR);
	goal.PoseTargets.push_back(pArrayL);


	// Hand states
	goal.ClosedStateAtBeginning.push_back(false);
	goal.ClosedStateAtBeginning.push_back(false);
	goal.ClosedStateAtEnd.push_back(false);
	goal.ClosedStateAtEnd.push_back(false);


	return goal;
}


hubo_motion_ros::ExecuteJointTrajectoryGoal createCurlGoal()
{
	hubo_motion_ros::ExecuteJointTrajectoryGoal goal;
	trajectory_msgs::JointTrajectory traj;
	traj.joint_names.push_back("LSP");
	traj.joint_names.push_back("LSR");
	traj.joint_names.push_back("LSY");
	traj.joint_names.push_back("LEB");
	traj.joint_names.push_back("LWY");
	traj.joint_names.push_back("LWP");

	for (int i = 0; i < 200; i++)
	{
		trajectory_msgs::JointTrajectoryPoint point;
		for (size_t j = 0; j < traj.joint_names.size(); j++)
		{
			if (traj.joint_names[j] == "LEB")
			{
				point.positions.push_back(i/2.0 * M_PI/180);
				point.velocities.push_back(0);
				point.accelerations.push_back(0);
			}
			else
			{
				point.positions.push_back(0);
				point.velocities.push_back(0);
				point.accelerations.push_back(0);
			}
		}
		traj.points.push_back(point);
	}

	goal.JointTargets = traj;

	return goal;
}

bool testJointClient(hubo_motion_ros::ExecuteJointTrajectoryGoal goal)
{
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<hubo_motion_ros::ExecuteJointTrajectoryAction> ac("/hubo/motion/hubo_trajectory_server_joint", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	ac.sendGoal(goal);

	// Check Ach data here
	hubo_motion_ros::AchROSBridge<hubo_manip_cmd> cmdChannel(CHAN_HUBO_MANIP_CMD);
	hubo_manip_cmd_t cmdActual;
	bool allCorrect = true;

	cmdActual = cmdChannel.waitState(250);
	for (int armIdx = 0; armIdx < NUM_ARMS; armIdx++)
	{
		allCorrect &= cmdActual.m_mode[armIdx] == manip_mode_t::MC_ANGLES;
		allCorrect &= cmdActual.m_ctrl[armIdx] == manip_ctrl_t::MC_NONE;
		allCorrect &= cmdActual.m_grasp[armIdx] == manip_grasp_t::MC_GRASP_LIMP;
		allCorrect &= cmdActual.interrupt[armIdx] == true;
	}

	if (allCorrect)
	{
		ROS_INFO("Everything's Right!");
	}


	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
	return true;
}

bool testPoseClient(hubo_motion_ros::ExecutePoseTrajectoryGoal goal)
{
	// create the action client
		// true causes the client to spin its own thread
		actionlib::SimpleActionClient<hubo_motion_ros::ExecutePoseTrajectoryAction> ac("/hubo/motion/hubo_trajectory_server_pose", true);

		ROS_INFO("Waiting for action server to start.");
		// wait for the action server to start
		ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");

		// send a goal to the action
		ac.sendGoal(goal);

		// TODO: Check Ach data here

		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else
		{
			ROS_INFO("Action did not finish before the time out.");
			return false;
		}
		return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_manipulation_server");
	ROS_INFO("Started test_manipulation_server.");
	ros::NodeHandle nh;
	ros::Publisher m_posePublisher = nh.advertise<geometry_msgs::PoseArray>("/hubo/pose_targets", 1);

	hubo_motion_ros::ExecutePoseTrajectoryGoal goal = createTrajectoryPoseGoal();
	m_posePublisher.publish(goal.PoseTargets[0]);

	//testPoseClient(goal);
	testJointClient(createCurlGoal());

	//ros::spin();

	return 0;
}

