/**
 *
 * \file trajectory_publisher.cpp
 * \brief Reads the ach channel containing trajectory data and republishes it as a ROS message
 *
 * \author Andrew Price
 * \date May 29, 2013
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
#include <trajectory_msgs/JointTrajectory.h>

#include <hubo-zmp.h>

#include "hubo_motion_ros/hubo_joint_names.h"
#include "hubo_motion_ros/drchubo_joint_names.h"
#include "hubo_motion_ros/AchROSBridge.h"

namespace hubo_motion_ros
{

/**
 * \class ZMPTrajectoryPublisher
 * \brief Reads the ach channel containing trajectory data and republishes it as a ROS message
 */
class ZMPTrajectoryPublisher
{
public:
	enum HUBO_MODEL
	{
		HUBOPLUS,
		DRCHUBO
	};

	ZMPTrajectoryPublisher(HUBO_MODEL hubo) :
		m_TrajChannel("hubo-zmp-traj")
	{
		m_TrajectoryPublisher = m_nh.advertise<trajectory_msgs::JointTrajectory>("/joint_trajectory", 1);
		m_HuboModel = hubo;
		ROS_INFO_STREAM("Hubo: " << m_HuboModel);
	}

	void publishTrajectory(const uint32_t  waitTime)
	{
		// Wait for a new trajectory message from the ach channel
		zmp_traj trajectory = m_TrajChannel.waitState(waitTime);

		trajectory_msgs::JointTrajectory jt;

		// Add all joint names
		for (unsigned joint = WST; joint <= LF5; joint++)
		{
			std::string jointName;
			if (HUBO_MODEL::DRCHUBO == m_HuboModel)
			{
				std::string jName = DRCHUBO_URDF_JOINT_NAMES[joint];
				if (jName == "") {continue;}
				jt.joint_names.push_back(jName);
				ROS_INFO("Adding a DRC-Hubo Joint!");
			}
			else
			{
				jt.joint_names.push_back(DRCHUBO_URDF_JOINT_NAMES[joint]);
			}
		}

		// Loop through all active trajectory steps
		for (size_t i = 0; i < trajectory.count; i++)
		{
			trajectory_msgs::JointTrajectoryPoint point;

			// Add the positions for each joint
			// TODO: do we need to do finite difference method to get V&A?
			for (unsigned joint = WST; joint <= LF5; joint++)
			{
				std::string jName = DRCHUBO_URDF_JOINT_NAMES[joint];
				if (jName == "") {continue;}
				point.positions.push_back(trajectory.traj[i].angles[joint]);
			}

			// Compute the time from the beginning
			point.time_from_start = ros::Duration(((float)i) * 1.0/(float)ZMP_TRAJ_FREQ_HZ);

			jt.points.push_back(point);
		}

		m_TrajectoryPublisher.publish(jt);
	}

protected:
	AchROSBridge<zmp_traj> m_TrajChannel;   ///< Handles setting up and reading from Ach channel

	ros::NodeHandle m_nh;                   ///< ROS NodeHandle for advertising topics
	ros::Publisher m_TrajectoryPublisher;   ///< ROS Publisher to publish new joint_trajectory messages
	HUBO_MODEL m_HuboModel;                 ///< Switches between different joint names for different models
};

} // namespace hubo_motion_ros

using namespace hubo_motion_ros;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_publisher");
	ROS_INFO("Started trajectory_publisher.");

	ros::NodeHandle nh;
	std::string huboModel;
	ZMPTrajectoryPublisher::HUBO_MODEL model;
	nh.param<std::string>("hubo_model", huboModel, "drc_hubo");

	if ("drc_hubo" == huboModel)
		model = ZMPTrajectoryPublisher::HUBO_MODEL::DRCHUBO;
	else if ("hubo_plus" == huboModel)
		model = ZMPTrajectoryPublisher::HUBO_MODEL::HUBOPLUS;
	else
		model = ZMPTrajectoryPublisher::HUBO_MODEL::HUBOPLUS;


	hubo_motion_ros::ZMPTrajectoryPublisher publisher(model);

	while (ros::ok())
	{
		publisher.publishTrajectory(0); // Wait forever
		ros::spinOnce();
	}

	return 0;
}

