/**
 *
 * \file hubo_manip_traj_forwarder.cpp
 * \brief Subscribes to ROS topics containing goal
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
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
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

#include <hubo.h>
#include <manip.h>

#include "hubo_motion_ros/hubo_joint_names.h"
#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/ExecuteJointTrajectoryAction.h"
#include "hubo_motion_ros/AchROSBridge.h"

class HuboManipulationAction
{
protected:

	ros::NodeHandle nh_;
	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
	actionlib::SimpleActionServer<hubo_motion_ros::ExecutePoseTrajectoryAction> asp_;
	actionlib::SimpleActionServer<hubo_motion_ros::ExecuteJointTrajectoryAction> asj_;
	std::string action_name_j_;
	std::string action_name_p_;

	// create messages that are used to published feedback/result
	hubo_motion_ros::ExecutePoseTrajectoryFeedback feedback_p_;
	hubo_motion_ros::ExecutePoseTrajectoryResult result_p_;

	hubo_motion_ros::ExecuteJointTrajectoryFeedback feedback_j_;
	hubo_motion_ros::ExecuteJointTrajectoryResult result_j_;

	AchROSBridge<hubo_manip_cmd> cmdChannel;
	AchROSBridge<hubo_manip_traj> trajChannel;
	AchROSBridge<hubo_manip_param> paramChannel;
	AchROSBridge<hubo_manip_state> stateChannel;

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

			size_t armIdx = goal->ArmIndex[armIter];

			cmd.m_mode[armIdx] = manip_mode_t::MC_TRAJ;
			cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_NONE;
			cmd.m_grasp[armIdx] = manip_grasp_t::MC_GRASP_AT_END;

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
		ros::Rate rate(10); // 10 hz
		while (!preempted && !completed && !error && ros::ok())
		{
			// check that preempt has not been requested by the client
			if (asp_.isPreemptRequested())
			{
				ROS_INFO("%s: Preempted", action_name_j_.c_str());
				// set the action state to preempted
				asp_.setPreempted();
				preempted = true;
				break;
			}

			// read the state channel
			completed = true;
			state = stateChannel.getState(true);
			for (int arm = 0; arm < NUM_ARMS; arm++)
			{
				feedback_j_.CommandState = state.mode_state[arm];
				feedback_j_.GraspState = state.grasp_state[arm];
				feedback_j_.ErrorState = state.error[arm];
				// TODO: how do we end this?
				//completed = completed && state.grasp_state[arm] == manip_grasp_t::MC_GRASP_AT_END;
				error = state.error[arm] != manip_error_t::MC_NO_ERROR;
			}

			// publish the feedback
			asj_.publishFeedback(feedback_j_);
		}

		// return the result
		result_j_.Success = (completed && !error) ? 1 : 0;
		asj_.setSucceeded(result_j_);
	}

	void executePoseCB(const hubo_motion_ros::ExecutePoseTrajectoryGoalConstPtr &goal)
	{
		bool success;

		hubo_manip_cmd_t cmd;


		// publish info to the console for the user
		//ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

		// start executing the action
		for (int armIdx = 1; armIdx <= goal->ArmIndex.size(); armIdx++)
		{
			// check that preempt has not been requested by the client
			if (asp_.isPreemptRequested() || !ros::ok())
			{
				ROS_INFO("%s: Preempted", action_name_p_.c_str());
				// set the action state to preempted
				asp_.setPreempted();
				success = false;
				return;
			}
			// publish the feedback
			asp_.publishFeedback(feedback_p_);
		}

		if (success)
		{
			//result_.sequence = feedback_.sequence;
			ROS_INFO("%s: Succeeded", action_name_p_.c_str());
			// set the action state to succeeded
			result_p_.Success = 1;
			asp_.setSucceeded(result_p_);
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "manip_traj_forwarder");
	ROS_INFO("Started hubo_manip_traj_forwarder.");

	HuboManipulationAction hma(ros::this_node::getName());
	ros::spin();

	return 0;
}

