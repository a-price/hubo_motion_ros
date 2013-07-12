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

#include "hubo_motion_ros/drchubo_joint_names.h"
#include "hubo_motion_ros/AchROSBridge.h"
#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/ExecuteJointTrajectoryAction.h"

#define ROS_EXPECT_EQ(expected, actual) \
	do { \
	  if (!((expected) == (actual))) { \
		ROS_FATAL("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s == %s\n\tmessage = ", __FILE__, __LINE__, #expected, #actual); \
		ROS_FATAL_STREAM("Expected: " << std::to_string(expected) << ". Actual: " << std::to_string(actual) << "."); \
		ROS_FATAL("\n"); \
		ROS_ISSUE_BREAK(); \
	  } \
	} while (0)

using namespace hubo_motion_ros;

bool spoofDaemon = true;
volatile bool receivedResult = false;

// Called once when the goal completes
void jointDoneCB(const actionlib::SimpleClientGoalState& state,
				 const ExecuteJointTrajectoryResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	receivedResult = true;
}

// Called once when the goal becomes active
void activeCB()
{
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void jointFeedbackCB(const ExecuteJointTrajectoryFeedbackConstPtr& feedback)
{
	if (!feedback->ErrorState == manip_error_t::MC_NO_ERROR && spoofDaemon)
	{
	ROS_INFO_STREAM("Got Feedback " <<
					"Command: " << (int)feedback->CommandState << ", "<<
					"Error: " << (int)feedback->ErrorState << ", "<<
					"Grasp: " << (int)feedback->GraspState);
	}
}

ExecutePoseTrajectoryGoal createSimplePoseGoal()
{
	ExecutePoseTrajectoryGoal goal;
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

ExecutePoseTrajectoryGoal createTrajectoryPoseGoal()
{
	ExecutePoseTrajectoryGoal goal;
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

ExecuteJointTrajectoryGoal createAngleGoal()
{
	ExecuteJointTrajectoryGoal goal;
	trajectory_msgs::JointTrajectory traj;
	trajectory_msgs::JointTrajectoryPoint point;

	traj.joint_names.push_back("REP");

	point.positions.push_back(100.0 * M_PI/180.0);
	traj.points.push_back(point);

	goal.JointTargets = traj;

	return goal;
}


ExecuteJointTrajectoryGoal createCurlGoal()
{
	ExecuteJointTrajectoryGoal goal;
	trajectory_msgs::JointTrajectory traj;
	traj.joint_names.push_back("RSP");
	traj.joint_names.push_back("RSR");
	traj.joint_names.push_back("RSY");
	traj.joint_names.push_back("REP");
	traj.joint_names.push_back("RWY");
	traj.joint_names.push_back("RWP");
	traj.joint_names.push_back("RWR");

	for (int i = 0; i < 200; i++)
	{
		trajectory_msgs::JointTrajectoryPoint point;
		for (size_t j = 0; j < traj.joint_names.size(); j++)
		{
			if (traj.joint_names[j] == "REP")
			{
				point.positions.push_back(i/2.0 * M_PI/180.0);
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
	ac.sendGoal(goal, &jointDoneCB, &activeCB, &jointFeedbackCB);

	if (spoofDaemon)
	{
	// Check Ach data here
	hubo_motion_ros::AchROSBridge<hubo_manip_cmd> cmdChannel(CHAN_HUBO_MANIP_CMD);
	hubo_motion_ros::AchROSBridge<hubo_manip_state> stateChannel(CHAN_HUBO_MANIP_STATE);
	hubo_manip_cmd_t cmdActual;
	hubo_manip_state_t stateSimulated;
	memset(&stateSimulated, 0, sizeof(stateSimulated));

	// Read the command from the channel
	while(!receivedResult && ros::ok())
	{
		cmdActual = cmdChannel.waitState(1000);
		int32_t step = cmdActual.goalID[0] - 1;
		ROS_ASSERT(step >= 0);

		ROS_INFO_STREAM("\n" << cmdActual);

		// Verify command parameters
		for (int armIdx = 0; armIdx < NUM_ARMS; armIdx++)
		{
			ROS_EXPECT_EQ(manip_mode_t::MC_ANGLES, cmdActual.m_mode[armIdx]);
			ROS_EXPECT_EQ(manip_ctrl_t::MC_NONE, cmdActual.m_ctrl[armIdx]);
			ROS_EXPECT_EQ(manip_grasp_t::MC_GRASP_LIMP, cmdActual.m_grasp[armIdx]);
			ROS_EXPECT_EQ(true, cmdActual.interrupt[armIdx]);
		}

		// Verify joint settings
		if (!goal.UseHardTrajectory)
		{
			for (size_t joint = 0; joint < goal.JointTargets.joint_names.size(); joint++)
			{
				std::string jointName = goal.JointTargets.joint_names[joint];
				unsigned arm = DRCHUBO_JOINT_NAME_TO_LIMB.at(jointName);
				unsigned pos = DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.at(jointName);
				if (cmdActual.arm_angles[arm][pos] != 0.0)
					ROS_INFO_STREAM(jointName << " : " << cmdActual.arm_angles[arm][pos]);
				ROS_EXPECT_EQ(goal.JointTargets.points[step].positions[joint], cmdActual.arm_angles[arm][pos]);
			}
		}
		else
		{

		}

		if (spoofDaemon)
		{
			// Send feedback state
			for (int armIdx = 0; armIdx < NUM_ARMS; armIdx++)
			{
				stateSimulated.goalID[armIdx] = cmdActual.goalID[armIdx];
				stateSimulated.error[armIdx] = manip_error_t::MC_NO_ERROR;
				stateSimulated.mode_state[armIdx] = manip_mode_t::MC_READY;
			}

			stateChannel.pushState(stateSimulated);
		}

		// TODO:Verify correct feedback
	}
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

	nh.param<bool>("spoof_daemon", spoofDaemon, true);

	ros::Publisher m_posePublisher = nh.advertise<geometry_msgs::PoseArray>("/hubo/pose_targets", 1);

	hubo_motion_ros::ExecutePoseTrajectoryGoal goal = createTrajectoryPoseGoal();
	m_posePublisher.publish(goal.PoseTargets[0]);

	//testPoseClient(goal);
	testJointClient(createAngleGoal());

	//ros::spin();

	return 0;
}

