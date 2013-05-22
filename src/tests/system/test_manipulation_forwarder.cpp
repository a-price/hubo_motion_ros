/**
 *
 * \file test_manipulation_forwarder.cpp
 * \brief 
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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/ExecuteJointTrajectoryAction.h"

class HuboManipulationAction
{
protected:

public:

};

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
	geometry_msgs::Pose pose;

	pArrayR.header.frame_id = "/world";
	pArrayL.header.frame_id = "/world";

	pose.position.x = 0.25;
	pose.position.y = -0.25;
	pose.position.z = -0.2;

	pose.orientation.w = 1.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;

	pArrayR.poses.push_back(pose);
	//goal.PoseTargets.push_back(pArrayR);

	pose.position.y = -pose.position.y;
	pArrayL.poses.push_back(pose);
	//goal.PoseTargets.push_back(pArrayL);

//	pose.position.y = -pose.position.y;
	pose.position.x = 0.0;
	pose.position.y = -0.45;
	pose.position.z = 0.25;

	pose.orientation.w = 0.7071067811865475;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.7071067811865475;

	pArrayR.poses.push_back(pose);

	pose.position.y = -pose.position.y;
	pArrayL.poses.push_back(pose);

	goal.ArmIndex.push_back(0);
	goal.ArmIndex.push_back(1);

	goal.PoseTargets.push_back(pArrayR);
	goal.PoseTargets.push_back(pArrayL);

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

	goal.ArmIndex.push_back(0);
	goal.JointTargets.push_back(traj);

	return goal;
}

bool testJointClient(hubo_motion_ros::ExecuteJointTrajectoryGoal goal)
{
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<hubo_motion_ros::ExecuteJointTrajectoryAction> ac("manip_traj_forwarder_joint", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	ac.sendGoal(goal);

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
		actionlib::SimpleActionClient<hubo_motion_ros::ExecutePoseTrajectoryAction> ac("manip_traj_forwarder_pose", true);

		ROS_INFO("Waiting for action server to start.");
		// wait for the action server to start
		ac.waitForServer(); //will wait for infinite time

		ROS_INFO("Action server started, sending goal.");

		// send a goal to the action
		ac.sendGoal(goal);

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_manipulation_forwarder");
	ROS_INFO("Started test_manipulation_forwarder.");
	ros::NodeHandle nh;
	ros::Publisher m_posePublisher = nh.advertise<geometry_msgs::PoseArray>("/hubo/pose_targets", 1);

	hubo_motion_ros::ExecutePoseTrajectoryGoal goal = createTrajectoryPoseGoal();
	m_posePublisher.publish(goal.PoseTargets[0]);
	testPoseClient(goal);
	//testJointClient(createCurlGoal());

	//ros::spin();

	return 0;
}

