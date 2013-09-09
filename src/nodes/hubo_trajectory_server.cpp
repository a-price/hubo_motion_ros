/**
 *
 * \file hubo_trajectory_server.cpp
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
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <hubo.h>
#include <manip.h>

#include <HuboKin.h>

#include <hubo_robot_msgs/JointTrajectoryAction.h>
#include <hubo_robot_msgs/PoseTrajectoryAction.h>
#include <hubo_robot_msgs/HybridTrajectoryAction.h>

//#include <control_msgs/GripperCommandAction.h>

#include "hubo_motion_ros/drchubo_joint_names.h"
#include "hubo_motion_ros/PoseConverter.h"
#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/ExecuteGripperAction.h"
//#include "hubo_motion_ros/ExecuteJointTrajectoryAction.h"
#include "hubo_motion_ros/AchROSBridge.h"
#include "hubo_motion_ros/ExecuteWaistAngleAction.h"

#define JOINTS_NOT_IMPLEMENTED

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
	actionlib::SimpleActionServer<hubo_robot_msgs::JointTrajectoryAction> asj_;
//	actionlib::SimpleActionServer<control_msgs::GripperCommandAction> asg_;
    actionlib::SimpleActionServer<hubo_motion_ros::ExecuteGripperAction> asg_;
    actionlib::SimpleActionServer<hubo_motion_ros::ExecuteWaistAngleAction> asw_;

	std::string action_name_j_;
	std::string action_name_p_;
	std::string action_name_g_;
    std::string action_name_w_;

	ros::Publisher finalHandPub;

	// create messages that are used to publish feedback/result
	hubo_motion_ros::ExecutePoseTrajectoryFeedback feedback_p_;
	hubo_motion_ros::ExecutePoseTrajectoryResult result_p_;

	hubo_robot_msgs::JointTrajectoryFeedback feedback_j_;
	hubo_robot_msgs::JointTrajectoryResult result_j_;

//	control_msgs::GripperCommandResult result_g_;
    hubo_motion_ros::ExecuteGripperResult result_g_;

	AchROSBridge<hubo_manip_state> stateChannel;
	AchROSBridge<hubo_manip_cmd> cmdChannel;
	AchROSBridge<hubo_manip_traj> trajChannel;
	AchROSBridge<hubo_manip_param> paramChannel;

    hubo_manip_cmd_t cmd;


    
	unsigned goalCount;

public:

	HuboManipulationAction(std::string name) :
		asp_(nh_, name + "_pose", boost::bind(&HuboManipulationAction::executePoseCB, this, _1), false),
		asj_(nh_, name + "_joint", boost::bind(&HuboManipulationAction::executeJointCB, this, _1), false),
		asg_(nh_, name + "_gripper", boost::bind(&HuboManipulationAction::executeGripperCB, this, _1), false),
        asw_(nh_, name + "_waist", boost::bind(&HuboManipulationAction::executeWaistCB, this, _1), false),
        
		action_name_j_(name + "_joint"),
		action_name_p_(name + "_pose"),
		action_name_g_(name + "_gripper"),
        action_name_w_(name + "_waist"),
		cmdChannel(CHAN_HUBO_MANIP_CMD),
		trajChannel(CHAN_HUBO_MANIP_TRAJ),
		paramChannel(CHAN_HUBO_MANIP_PARAM),
		stateChannel(CHAN_HUBO_MANIP_STATE)
	{
		asp_.start();
		asj_.start();
		asg_.start();
        asw_.start();
		goalCount = 1;
		cmdChannel.flush();
		stateChannel.flush();
		ROS_INFO("Constructed Server.");

		finalHandPub = nh_.advertise<geometry_msgs::PoseArray>("/hubo/final_hand_poses", 1);

        memset(&cmd, 0, sizeof(cmd));
	}

	~HuboManipulationAction(void)
	{
	}
    
    void executeWaistCB(const hubo_motion_ros::ExecuteWaistAngleGoalConstPtr &goal)
    {
        cmd.waistAngle = goal->waistAngle;
        cmdChannel.pushState(cmd);
        
        // Should probably do other things, like feedback and result,
        // but who really cares right now?
    }

	void executeJointCB(const hubo_robot_msgs::JointTrajectoryGoalConstPtr &goal)
	{
        ROS_INFO("New joint configuration!");
		bool preempted = false, error = false, completed = false;
		bool staleWarning = true, errorWarning = true, channelWarning = true;
		///////////result_j_.Success = false;
        ros::Time tOut;

		// Set global properties
		cmd.convergeNorm = CONVERGENCE_THRESHOLD;
		cmd.stopNorm = IMMOBILITY_THRESHOLD;
		cmd.waistAngle = 0.0;
		goalCount++; // Is it better to restart or increment?
		//std::cerr << "Joints: " << goal->trajectory << std::endl;
#ifdef JOINTS_NOT_IMPLEMENTED

		// Set command properties for each arm
		for (int armIdx = 0; armIdx < NUM_ARMS; armIdx++)
		{
			cmd.m_mode[armIdx] = manip_mode_t::MC_ANGLES;
            cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_RIGID;
			cmd.m_grasp[armIdx] = manip_grasp_t::MC_GRASP_LIMP;
			cmd.interrupt[armIdx] = true;
		}

		// Send an "MC_ANGLES" command for each timestep
		for (size_t point = 0; point < goal->trajectory.points.size(); point++)
		{
			// Reset error flags
			preempted = false; error = false; completed = false;

			// Set the goal count for all arms
			for (int armIdx = 0; armIdx < NUM_ARMS; armIdx++)
			{
				std::cerr << "Goal ID: " << goalCount << std::endl;
				cmd.goalID[armIdx] = goalCount;
			}

			// Assign all joint targets to their positions in the command structure arrays.
			for (size_t joint = 0; joint < goal->trajectory.joint_names.size(); joint++)
			{
				std::string jointName = goal->trajectory.joint_names[joint];
				auto limbIter = DRCHUBO_JOINT_NAME_TO_LIMB.find(jointName);
				auto posIter = DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.find(jointName);

				if (limbIter == DRCHUBO_JOINT_NAME_TO_LIMB.end())
				{
					ROS_WARN_STREAM("Joint Name '" << jointName << "' does not exist in DRCHUBO_JOINT_NAME_TO_LIMB.");
					continue;
				}
				if (posIter == DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.end())
				{
					ROS_WARN_STREAM("Joint Name '" << jointName << "' does not exist in DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.");
					continue;
				}

				unsigned arm = limbIter->second;
				unsigned pos = posIter->second;

				if (arm != LEFT && arm != RIGHT)
				{
					ROS_WARN_STREAM("Joint Name '" << jointName << "' is not an arm joint.");
					continue;
				}
				cmd.arm_angles[arm][pos] = goal->trajectory.points[point].positions[joint];
			}

			// Set the wait period
			ros::Duration waitTime;
			if (point > 0)
			{
				waitTime = goal->trajectory.points[point].time_from_start - goal->trajectory.points[point-1].time_from_start;
			}
			else
			{
				waitTime = goal->trajectory.points[point].time_from_start;
			}

			if (waitTime < ros::Duration(1.0))
			{
				waitTime = ros::Duration(1.0);
			}
			tOut = ros::Time::now() + waitTime;

			// Send the command
			std::cerr << "Sending Command:\n" << cmd << std::endl;
			cmdChannel.pushState(cmd);

			// Wait for completion, failure, or timeout, while checking for feedback or preemption
			hubo_manip_state_t state;
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

				// Check for timeout
				if (tOut < ros::Time::now())
				{
					ROS_WARN("Goal Timed out.");
					error = true;
					break;
				}

				// read the state channel
				ach_status_t achResult;
				completed = true;
				state = stateChannel.waitState(50, &achResult, false);

				if (ACH_OK != achResult && ACH_MISSED_FRAME != achResult)
				{
					completed = false;
					ROS_ERROR("Problem reading Ach channel '%s', error: (%d) %s",
						CHAN_HUBO_MANIP_STATE, achResult, ach_result_to_string((ach_status_t)achResult));
					continue;
				}
				else if (ACH_TIMEOUT == achResult)
				{
					completed = false;
					if (channelWarning)
					{
						ROS_ERROR("Problem reading Ach channel '%s', error: (%d) %s",
								  CHAN_HUBO_MANIP_STATE, achResult, ach_result_to_string((ach_status_t)achResult));
						channelWarning = false;
					}
					continue;
				}
				else
				{
					//std::cerr << "Got good state:\n" << state << std::endl;
				}

				for (int arm = 0; arm < NUM_ARMS; arm++)
				{
					///////////feedback_j_.CommandState = state.mode_state[arm];
					///////////feedback_j_.GraspState = state.grasp_state[arm];
					///////////feedback_j_.ErrorState = state.error[arm];

					// Data is from an old command
					if (state.goalID[arm] != goalCount)
					{
						completed = false;
						error = false;
						if (staleWarning)
						{
							ROS_WARN("Got old cmd ID: %i, arm: %i", state.goalID[arm], arm);
							staleWarning = false;
						}
						continue;
					}

					completed = completed &&
							((state.mode_state[arm] == manip_mode_t::MC_READY) ||
							(state.mode_state[arm] == manip_mode_t::MC_HALT));
					error = error || state.error[arm] != manip_error_t::MC_NO_ERROR;

					if (error && errorWarning)
					{
						ROS_ERROR("Manipulation State Error: %i for arm %i", state.error[arm], arm);
						errorWarning = false;
					}

					if (completed)
					{
						ROS_INFO("Manipulation State Completed: %i for arm %i", state.mode_state[arm], arm);
					}
				}

				// publish the feedback
				asj_.publishFeedback(feedback_j_);
			}

			// Increment the goal counter
			goalCount++;
			staleWarning = true;
			channelWarning = true;
			errorWarning = true;

		}

		std::cerr << "error: " << error << std::endl;
		bool success = (completed && !error && !preempted);
		///////////result_j_.Success = (uint8_t)success;// ? 1 : 0;
		if (success)
		{
			asj_.setSucceeded(result_j_);
		}
		else
		{
			asj_.setAborted(result_j_);
		}
#else

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

			// Get the actual arm index, not the position in the array of indices
			size_t armIdx = goal->ArmIndex[armIter];

			// Set arm properties
			cmd.m_mode[armIdx] = manip_mode_t::MC_TRAJ;
			cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_RIGID;
			cmd.m_grasp[armIdx] = manip_grasp_t::MC_GRASP_AT_END;
			cmd.interrupt[armIdx] = true;
			cmd.goalID[armIdx] = goalCount;

			std::map<std::string, int>::const_iterator mapIter;
			// Iterate through all joints in each step
			for (int jointIter = 0; jointIter < goal->trajectory[armIter].joint_names.size(); jointIter++)
			{
				// Get joint index for joint name
				mapIter = DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.find(goal->trajectory[armIter].joint_names[jointIter]);
				if (mapIter == DRCHUBO_JOINT_NAME_TO_LIMB_POSITION.end())
				{
					ROS_ERROR("Joint name '%s' is unknown.", goal->trajectory[armIter].joint_names[jointIter].c_str());
					continue;
				}
				size_t jointIdx = mapIter->second;

				// Iterate through all timesteps
				for (int timeStep = 0; timeStep < std::min(goal->trajectory[armIter].points.size(), (size_t)MAX_TRAJ_SIZE); timeStep++)
				{
					// Assign all step parameters
					traj.arm_angles[armIdx][jointIdx][timeStep] = goal->trajectory[armIter].points[timeStep].positions[jointIter];
					traj.arm_speeds[armIdx][jointIdx][timeStep] = goal->trajectory[armIter].points[timeStep].velocities[jointIter];
					traj.arm_accels[armIdx][jointIdx][timeStep] = goal->trajectory[armIter].points[timeStep].accelerations[jointIter];
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
						(state.mode_state[arm] == manip_mode_t::MC_HALT));
				error = state.error[arm] != manip_error_t::MC_NO_ERROR;

				if (error)
				{
					ROS_ERROR("Manipulation State Error: %i for arm %i", state.error[arm], arm);
				}

				if (completed)
				{
					ROS_INFO("Manipulation State Completed: %i for arm %i", state.mode_state[arm], arm);
				}
			}

			// publish the feedback
			asj_.publishFeedback(feedback_j_);
		}

		// return the result
		result_j_.Success = (completed && !error && !preempted) ? 1 : 0;
		asj_.setSucceeded(result_j_);
#endif
	}

	void forceSetGrasps(manip_grasp_t grasps, bool interrupting)
	{
		hubo_manip_cmd_t cmd;
		memset(&cmd, 0, sizeof(cmd));

		cmd.convergeNorm = CONVERGENCE_THRESHOLD;
		cmd.stopNorm = IMMOBILITY_THRESHOLD;
		goalCount++;

		// Set the initial hand state
		//for (size_t armIdx = 0; armIdx < NUM_ARMS; armIdx++)
		size_t armIdx = RIGHT;
		{
			cmd.m_mode[armIdx] = manip_mode_t::MC_READY;
			cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_RIGID;
			cmd.m_grasp[armIdx] = grasps; //manip_grasp_t::MC_GRASP_NOW;
			cmd.interrupt[armIdx] = interrupting;
			cmd.goalID[armIdx] = goalCount;
		}

		cmdChannel.pushState(cmd);
	}

	void executePoseCB(const hubo_motion_ros::ExecutePoseTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO("Pose command");
		geometry_msgs::PoseArray currentPoses;
		std::set<size_t> armIndices;
		bool preempted = false, error = false, completed = false;
        ///////////result_p_.Success = false;

		ros::Time tOut;

		// Set global properties
		cmd.convergeNorm = CONVERGENCE_THRESHOLD;
		cmd.stopNorm = IMMOBILITY_THRESHOLD;

        // Set the initial hand state
		for (size_t armIter = 0; armIter < goal->ArmIndex.size(); armIter++)
        {
			// Caution: Remember that armIndex refers to the command's arm index, while
			//  armIter refers to the goal.
			size_t armIdx = goal->ArmIndex[armIter];
			if (armIdx > (size_t)NUM_ARMS) {continue;} // Should probably throw warning here...

			cmd.m_mode[armIdx] = manip_mode_t::MC_READY;
            cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_RIGID;
//			cmd.m_grasp[armIdx] = goal->ClosedStateAtBeginning[armIter] ? manip_grasp_t::MC_GRASP_NOW : manip_grasp_t::MC_RELEASE_NOW;
//			//cmd.m_grasp[armIdx] = manip_grasp_t::MC_RELEASE_NOW;
//			ROS_INFO("Hand: %lu", armIdx);
//			ROS_INFO(goal->ClosedStateAtBeginning[armIter] ? "Closing hand." : "Opening hand.");
//			cmd.interrupt[armIdx] = true;
//			cmd.goalID[armIdx] = goalCount;
		}
		// Open or close the hands
//		cmdChannel.pushState(cmd);
//		stateChannel.waitState(250);

		goalCount++;

		// Iterate through all poses provided
		ROS_INFO("%lu steps to complete.", goal->PoseTargets[0].poses.size());
		for (int poseIter = 0; poseIter < goal->PoseTargets[0].poses.size(); poseIter++)
		{
			preempted = false, error = false, completed = false;
			ROS_INFO("Pose # %i", poseIter);
			tOut = ros::Time::now() + ros::Duration(10,0);
			currentPoses.poses.clear();
			goalCount++;

			// Build cmd packet by iterating through all arms provided
			for (size_t armIter = 0; armIter < goal->ArmIndex.size(); armIter++)
            {
				size_t armIdx = goal->ArmIndex[armIter];
				if (armIdx > (size_t)NUM_ARMS) {continue;}
				armIndices.insert(armIdx);

//				cmd.m_mode[armIdx] = manip_mode_t::MC_TRANS_QUAT;
                cmd.m_mode[armIdx] = manip_mode_t::MC_TELEOP;
                cmd.m_ctrl[armIdx] = manip_ctrl_t::MC_RIGID;
				cmd.interrupt[armIdx] = true;
                cmd.goalID[armIdx] = goalCount;
//				if (goal->PoseTargets[0].poses.size()-1 == poseIter)
//				{
//					cmd.m_grasp[armIdx] = goal->ClosedStateAtEnd[armIter] ? manip_grasp_t::MC_GRASP_AT_END : manip_grasp_t::MC_RELEASE_AT_END;
//					//cmd.m_grasp[armIdx] = manip_grasp_t::MC_RELEASE_NOW;
//				}
//				else
//				{
//					cmd.m_grasp[armIdx] = manip_grasp_t::MC_GRASP_STATIC;
//					//cmd.m_grasp[armIdx] = manip_grasp_t::MC_RELEASE_NOW;
//				}

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

				currentPoses.poses.push_back(goalPose);
			}

			// Publish the target hand positions for debugging purposes
			currentPoses.header.stamp = ros::Time::now();
			currentPoses.header.frame_id = "/Body_Hip";
			finalHandPub.publish(currentPoses);


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
				for (unsigned arm = 0; arm < NUM_ARMS; arm++)
				{
					if (armIndices.end() == armIndices.find(arm)) // Don't care about this arm, keep going
					{
						continue;
					}
					///////////feedback_p_.CommandState = state.mode_state[arm];
					///////////feedback_p_.GraspState = state.grasp_state[arm];
					///////////feedback_p_.ErrorState = state.error[arm];

					// Data is from an old command
					if (state.goalID[arm] != goalCount)
					{
						completed = false;
						error = false;
						ROS_WARN("Got old cmd ID: %i, arm: %i", state.goalID[arm], arm);
						continue;
					}

					completed = completed &&
								((state.mode_state[arm] == manip_mode_t::MC_READY) ||
								(state.mode_state[arm] == manip_mode_t::MC_HALT));
					error = state.error[arm] != manip_error_t::MC_NO_ERROR;
					if (error)
					{
						ROS_ERROR("Manipulation State Error: %i for arm %i", state.error[arm], arm);
					}

					if (completed)
					{
						ROS_INFO("Manipulation State Completed: %i for arm %i", state.mode_state[arm], arm);
					}
				}

				// publish the feedback
				asp_.publishFeedback(feedback_p_);
			}
		}

//		forceSetGrasps(manip_grasp_t::MC_GRASP_NOW, true);

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
		///////////result_p_.Success = (completed && !error && !preempted) ? 1 : 0;
		if (result_p_.Success)
		{
			asp_.setSucceeded(result_p_);
		}
		else
		{
			asp_.setAborted(result_p_);
		}
//		forceSetGrasps(manip_grasp_t::MC_GRASP_STATIC, false);
	}

    void executeGripperCB(const hubo_motion_ros::ExecuteGripperGoalConstPtr &goal)
	{

        ROS_INFO("Gripper command");
//		if (fabs(goal->command.position) <= 0.01)
//		{
//			if (fabs(goal->command.max_effort) <= 0.01)
//			{
//				forceSetGrasps(manip_grasp_t::MC_GRASP_LIMP, true);
//				result_g_.position = 0.0;
//				result_g_.effort = 0.0;
//			}
//			else
//			{
//				forceSetGrasps(manip_grasp_t::MC_GRASP_STATIC, true);
//				result_g_.position = 0.0;
//				result_g_.effort = 1.0;
//			}
//		}
//		else if (goal->command.position > 0.01)
//		{
//			std::cerr << "[Manip Server] Setting grasp to close." << std::endl;
//			forceSetGrasps(manip_grasp_t::MC_GRASP_NOW, true);
//			result_g_.position = 1.0;
//			result_g_.effort = 1.0;
//		}
//		else if (goal->command.position < -0.01)
//		{
//			std::cerr << "[Manip Server] Setting grasp to release." << std::endl;
//			forceSetGrasps(manip_grasp_t::MC_RELEASE_NOW, true);
//			result_g_.position = -1.0;
//			result_g_.effort = 1.0;
//		}

//		result_g_.reached_goal = true;
//		result_g_.stalled = false;

//      asg_.setSucceeded(result_g_);


        if( goal->grip.size() == goal->ArmIndex.size() )
        {
            for(int i=0; i<goal->grip.size(); i++)
            {
                manip_grasp_t graspType;
                if( goal->grip[i] == hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_LIMP )
                    graspType = MC_GRASP_LIMP;
                else if( goal->grip[i] == hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_NOW )
                    graspType = MC_GRASP_NOW;
                else if( goal->grip[i] == hubo_motion_ros::ExecuteGripperGoal::PTA_GRASP_RELEASE_NOW )
                    graspType = MC_RELEASE_NOW;

                if( goal->ArmIndex[i] == hubo_motion_ros::ExecuteGripperGoal::PTA_LEFT )
                    cmd.m_grasp[LEFT] = graspType;
                else if( goal->ArmIndex[i] == hubo_motion_ros::ExecuteGripperGoal::PTA_RIGHT )
                    cmd.m_grasp[RIGHT] = graspType;
                else if( goal->ArmIndex[i] == hubo_motion_ros::ExecuteGripperGoal::PTA_TRIG )
                    cmd.trigger = graspType;


                cmdChannel.pushState(cmd);
            }

            result_g_.Success = true;
            asg_.setSucceeded(result_g_);
        }
        else
        {
            ROS_ERROR("Number of grip commands did not match number of arms");
            result_g_.Success = false;
            asg_.setSucceeded(result_g_);
        }
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

