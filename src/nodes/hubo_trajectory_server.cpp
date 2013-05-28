/**
 *
 * \file hubo_manip_traj_forwarder.cpp
 * \brief Subscribes to ROS topics containing manipulation goal information, forwards that information over ach, and reports on the progress.
 *
 * \author Andrew Price
 * \date May 20, 2013
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

#include <algorithm>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <hubo.h>
#include <manip.h>

#include "hubo_motion_ros/hubo_joint_names.h"
#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/ExecuteJointTrajectoryAction.h"
#include "hubo_motion_ros/AchROSBridge.h"

#define CONVERGENCE_THRESHOLD 0.075
#define IMMOBILITY_THRESHOLD 0.01

namespace hubo_motion_ros
{

/**
 * \class HuboManipulationAction
 * \brief ActionServer handling callbacks for pose and joint trajectory goals.
 */
class HuboManipulationAction
{
protected:

	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<hubo_motion_ros::ExecutePoseTrajectoryAction> asp_;
	actionlib::SimpleActionServer<hubo_motion_ros::ExecuteJointTrajectoryAction> asj_;
	std::string action_name_j_;
	std::string action_name_p_;

	// create messages that are used to publish feedback/result
	hubo_motion_ros::ExecutePoseTrajectoryFeedback feedback_p_;
	hubo_motion_ros::ExecutePoseTrajectoryResult result_p_;

	hubo_motion_ros::ExecuteJointTrajectoryFeedback feedback_j_;
	hubo_motion_ros::ExecuteJointTrajectoryResult result_j_;

	AchROSBridge<hubo_manip_state> stateChannel;
	AchROSBridge<hubo_manip_cmd> cmdChannel;
	AchROSBridge<hubo_manip_traj> trajChannel;
	AchROSBridge<hubo_manip_param> paramChannel;

	unsigned goalCount;

public:

	HuboManipulationAction(std::string name) :
		asp_(nh_, name + "_pose", boost::bind(&HuboManipulationAction::executePoseCB, this, _1), false),
		asj_(nh_, name + "_joint", boost::bind(&HuboManipulationAction::executeJointCB, this, _1), false),
		action_name_j_(name + "_joint"),
		action_name_p_(name + "_pose"),
		cmdChannel(CHAN_HUBO_MANIP_CMD),
		trajChannel(CHAN_HUBO_MANIP_TRAJ),
		paramChannel(CHAN_HUBO_MANIP_PARAM),
		stateChannel(CHAN_HUBO_MANIP_STATE)
	{
		asp_.start();
		asj_.start();
		goalCount = 1;
		ROS_INFO("Constructed Server.");
	}

	~HuboManipulationAction(void)
	{
	}

	void executeJointCB(const hubo_motion_ros::ExecuteJointTrajectoryGoalConstPtr &goal)
	{
		bool preempted = false, error = false, completed = false;
		result_j_.Success = false;
		hubo_manip_cmd_t cmd;
		hubo_manip_traj_t traj;

		// Set global properties
		cmd.convergeNorm = CONVERGENCE_THRESHOLD;
		cmd.stopNorm = IMMOBILITY_THRESHOLD;
		goalCount++;

		// Iterate through all arms provided
		for (int armIter = 0; armIter < std::min(goal->ArmIndex.size(),(size_t)NUM_ARMS); armIter++)
		{
			// check that preempt has not been requested by the client
			if (asj_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_j_.c_str());
				// set the action state to preempted
				asj_.setPreempted(result_j_);
				preempted = true;
				return;
			}

			// Get the actual arm index, not the position in the array of indices
			size_t armIdx = goal->ArmIndex[armIter];

			// Set arm properties
			cmd.m_mode[armIdx] = manip_mode_t::MC_TRAJ;
			cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_NONE;
			cmd.m_grasp[armIdx] = manip_grasp_t::MC_GRASP_AT_END;
			cmd.interrupt[armIdx] = true;
			cmd.goalID[armIdx] = goalCount;

			std::map<std::string, int>::const_iterator mapIter;
			// Iterate through all joints in each step
			for (int jointIter = 0; jointIter < goal->JointTargets[armIter].joint_names.size(); jointIter++)
			{
				// Get joint index for joint name
				mapIter = HUBO_JOINT_NAME_TO_LIMB_POSITION.find(goal->JointTargets[armIter].joint_names[jointIter]);
				if (mapIter == HUBO_JOINT_NAME_TO_LIMB_POSITION.end())
				{
					ROS_ERROR("Joint name '%s' is unknown.", goal->JointTargets[armIter].joint_names[jointIter].c_str());
					continue;
				}
				size_t jointIdx = mapIter->second;

				// Iterate through all timesteps
				for (int timeStep = 0; timeStep < std::min(goal->JointTargets[armIter].points.size(), (size_t)MAX_TRAJ_SIZE); timeStep++)
				{
					// Assign all step parameters
					traj.arm_angles[armIdx][jointIdx][timeStep] = goal->JointTargets[armIter].points[timeStep].positions[jointIter];
					traj.arm_speeds[armIdx][jointIdx][timeStep] = goal->JointTargets[armIter].points[timeStep].velocities[jointIter];
					traj.arm_accels[armIdx][jointIdx][timeStep] = goal->JointTargets[armIter].points[timeStep].accelerations[jointIter];
				}
			}
		}

		// write to channel
		cmdChannel.pushState(cmd);
		trajChannel.pushState(traj);

		// wait for completion, with preemption
		hubo_manip_state_t state;
		//ros::Rate rate(10); // 10 hz
		while (!preempted && !completed && !error && ros::ok())
		{
			// check that preempt has not been requested by the client
			if (asp_.isPreemptRequested())
			{
				ROS_WARN("%s: Preempted", action_name_j_.c_str());
				// set the action state to preempted
				asp_.setPreempted();
				preempted = true;
				return;
			}
			else if (asp_.isNewGoalAvailable())
			{
				ROS_WARN("New Goal available.");
			}

			// read the state channel
			completed = true;
			state = stateChannel.waitState(50);
			for (int arm = 0; arm < NUM_ARMS; arm++)
			{
				feedback_j_.CommandState = state.mode_state[arm];
				feedback_j_.GraspState = state.grasp_state[arm];
				feedback_j_.ErrorState = state.error[arm];

				completed = completed &&
						((state.mode_state[arm] == manip_mode_t::MC_READY) ||
						(state.mode_state[arm] == manip_mode_t::MC_STOPPED));
				error = state.error[arm] != manip_error_t::MC_NO_ERROR;
			}

			// publish the feedback
			asj_.publishFeedback(feedback_j_);
		}

		// return the result
		result_j_.Success = (completed && !error && !preempted) ? 1 : 0;
		asj_.setSucceeded(result_j_);
	}

	void executePoseCB(const hubo_motion_ros::ExecutePoseTrajectoryGoalConstPtr &goal)
	{
		bool preempted = false, error = false, completed = false;
		result_p_.Success = false;
		hubo_manip_cmd_t cmd;

		ros::Time tOut;

		// Set global properties
		cmd.convergeNorm = CONVERGENCE_THRESHOLD;
		cmd.stopNorm = IMMOBILITY_THRESHOLD;

		// Set the initial hand state
		for (int armIter = 0; armIter < goal->ArmIndex.size(); armIter++)
		{
			// Caution: Remember that armIndex refers to the command's arm index, while
			//  armIter refers to the goal.
			size_t armIdx = goal->ArmIndex[armIter];
			if (armIdx > (size_t)NUM_ARMS) {continue;} // Should probably throw warning here...

			cmd.m_mode[armIdx] = manip_mode_t::MC_READY;
			cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_NONE;
			cmd.m_grasp[armIdx] = goal->ClosedStateAtBeginning[armIter] ? manip_grasp_t::MC_GRASP_NOW : manip_grasp_t::MC_RELEASE_NOW;
			//cmd.m_grasp[armIdx] = manip_grasp_t::MC_RELEASE_NOW;
			ROS_INFO("Hand: %i", armIdx);
			ROS_INFO(goal->ClosedStateAtBeginning[armIter] ? "Closing hand.\n\n\n\n" : "Opening hand.\n\n\n\n");
			cmd.interrupt[armIdx] = true;
			cmd.goalID[armIdx] = goalCount;
		}
		// Open or close the hands
		cmdChannel.pushState(cmd);
		stateChannel.waitState(250);

		goalCount++;

		// Iterate through all poses provided
		ROS_INFO("%lu steps to complete.", goal->PoseTargets[0].poses.size());
		for (int poseIter = 0; poseIter < goal->PoseTargets[0].poses.size(); poseIter++)
		{
			preempted = false, error = false, completed = false;
			ROS_INFO("Pose # %i", poseIter);
			tOut = ros::Time::now() + ros::Duration(10,0);
			goalCount++;

			// Build cmd packet by iterating through all arms provided
			for (int armIter = 0; armIter < goal->ArmIndex.size(); armIter++)
			{
				size_t armIdx = goal->ArmIndex[armIter];
				if (armIdx > (size_t)NUM_ARMS) {continue;}

				cmd.m_mode[armIdx] = manip_mode_t::MC_TRANS_QUAT;
				cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_NONE;
				cmd.interrupt[armIdx] = true;
				cmd.goalID[armIdx] = goalCount;
				if (goal->PoseTargets[0].poses.size()-1 == poseIter)
				{
					cmd.m_grasp[armIdx] = goal->ClosedStateAtEnd[armIter] ? manip_grasp_t::MC_GRASP_AT_END : manip_grasp_t::MC_RELEASE_AT_END;
					//cmd.m_grasp[armIdx] = manip_grasp_t::MC_RELEASE_NOW;
				}
				else
				{
					cmd.m_grasp[armIdx] = manip_grasp_t::MC_GRASP_STATIC;
					//cmd.m_grasp[armIdx] = manip_grasp_t::MC_RELEASE_NOW;
				}

				hubo_manip_pose_t pose;
				const geometry_msgs::Pose goalPose = goal->PoseTargets[armIter].poses[poseIter];
				pose.x = goalPose.position.x;
				pose.y = goalPose.position.y;
				pose.z = goalPose.position.z;
				pose.i = goalPose.orientation.x;
				pose.j = goalPose.orientation.y;
				pose.k = goalPose.orientation.z;
				pose.w = goalPose.orientation.w;

				cmd.pose[armIdx] = pose;
			}


			// NB: the feedback publishing must be nested here, since the protocol only defines one pose at a time.
			// write to channel
			cmdChannel.pushState(cmd);

			// wait for completion, with preemption
			hubo_manip_state_t state;
			//ros::Rate rate(10); // 10 hz
			while (!preempted && !completed && ros::ok())
			{
				// check that preempt has not been requested by the client
				if (asp_.isPreemptRequested())
				{
					ROS_WARN("%s: Preempted", action_name_p_.c_str());
					// set the action state to preempted
					asp_.setPreempted();
					preempted = true;
					break;
				}
				if (asp_.isNewGoalAvailable())
				{
					ROS_WARN("New Goal available.");
				}
				if (tOut < ros::Time::now())
				{
					ROS_WARN("Goal Timed out.");
					error = true;
					break;
				}

				// read the state channel
				completed = true;
				state = stateChannel.waitState(250);
				for (int arm = 0; arm < NUM_ARMS; arm++)
				{
					feedback_p_.CommandState = state.mode_state[arm];
					feedback_p_.GraspState = state.grasp_state[arm];
					feedback_p_.ErrorState = state.error[arm];

					// Data is from an old command
					if (state.goalID[arm] != goalCount)
					{
						completed = false;
						error = false;
						ROS_WARN("Got old cmd ID: %i", state.goalID[arm]);
						continue;
					}

					completed = completed &&
								((state.mode_state[arm] == manip_mode_t::MC_READY) ||
								(state.mode_state[arm] == manip_mode_t::MC_STOPPED));
					error = state.error[arm] != manip_error_t::MC_NO_ERROR;
					if (error)
					{
						ROS_ERROR("Manipulation State Error: %i for arm %i", state.error[arm], arm);
					}

					if (completed)
					{
						ROS_INFO("Manipulation State Completed: %i for arm %i", state.mode_state[arm], arm);
					}
					else
					{
						ROS_INFO("Not finished yet: arm %i", arm);
					}
				}

				// publish the feedback
				asp_.publishFeedback(feedback_p_);
			}
		}

		// return the result
		if (error && completed)
		{
			ROS_WARN("Completed Goal with Error");
		}
		else if (preempted)
		{
			ROS_INFO("Goal was preempted.");
		}
		else
		{
			ROS_INFO("Completed Goal.");
		}
		result_p_.Success = (completed && !error && !preempted) ? 1 : 0;
		asp_.setSucceeded(result_p_);
	}
};

} // namespace hubo_motion_ros

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hw_trajectory_server");
	ROS_INFO("Started trajectory_server.");

	hubo_motion_ros::HuboManipulationAction hma(ros::this_node::getName());
	ros::spin();

	return 0;
}

