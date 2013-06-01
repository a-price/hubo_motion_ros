/**
 *
 * \file joint_trajectory_to_states.cpp
 * \brief 
 *
 * \author Andrew Price
 * \date Jun 1, 2013
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
#include <sensor_msgs/JointState.h>

ros::Publisher m_JointPublisher;
ros::Subscriber m_TrajSubscriber;

void trajectoryCallback(trajectory_msgs::JointTrajectoryConstPtr jt)
{
	ros::Time startTime = ros::Time::now();

	for (int step = 0; step < jt->points.size(); step++)
	{
		sensor_msgs::JointState js;

		// Copy names
		for (int j = 0; j < jt->joint_names.size(); j++)
		{
			js.name.push_back(jt->joint_names[j]);
		}

		// Copy position and velocity data
		for (int j = 0; j < jt->joint_names.size(); j++)
		{
			js.position.push_back(jt->points[step].positions[j]);
			js.velocity.push_back(jt->points[step].velocities[j]);
		}

		// Possibly sleep if the goal is still in the future
		ros::Time goalTime = startTime + jt->points[step].time_from_start;
		if (goalTime > ros::Time::now())
		{
			(goalTime - ros::Time::now()).sleep();
		}

		// Send the goal as the actual state
		m_JointPublisher.publish(js);

	}

}

int main(int argc, char** argv)
{
	ROS_INFO("Started joint_trajectory_to_states.");
	ros::init(argc, argv, "joint_trajectory_to_states");

	ros::NodeHandle m_nh;

	m_TrajSubscriber = m_nh.subscribe("joint_trajectory", 1, &trajectoryCallback);
	m_JointPublisher = m_nh.advertise<sensor_msgs::JointState>("joint_states", 1);

	ros::spin();

	return 0;
}

