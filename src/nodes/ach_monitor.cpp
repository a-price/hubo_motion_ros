/**
 *
 * \file ach_monitor.cpp
 * \brief 
 *
 * \author Andrew Price
 * \date May 23, 2013
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
#include <manip.h>

#include "hubo_motion_ros/AchMonitor.h"

/*
std::ostream& operator<<(std::ostream& os, const hubo_manip_state& obj)
{
	for (size_t arm = 0; arm < NUM_ARMS; arm++)
	{
		os << "Arm " << arm << " Manipulation State\n";
		os << "Goal ID: " << obj.goalID[arm] << "\n";
		os << "Mode: " << obj.mode_state[arm] << "\n";
		os << "Grasp: " << obj.grasp_state[arm] << "\n";
		os << "Error: " << obj.error[arm] << "\n";
	}
	return os;
}

std::ostream& operator<<(std::ostream& os, const hubo_manip_cmd& obj)
{
	for (size_t arm = 0; arm < NUM_ARMS; arm++)
	{
		os << "Arm " << arm << " Manipulation State\n";
		os << "Goal ID: " << obj.goalID[arm] << "\n";
		os << "Mode: " << obj.m_mode[arm] << "\n";
		os << "Grasp: " << obj.m_grasp[arm] << "\n";
		//os << "Error: " << obj.pose[arm] << "\n";
	}
	return os;
}*/

int main(int argc, char** argv)
{
	ROS_INFO("Started ach_monitor.");
	ros::init(argc, argv, "ach_monitor", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	//hubo_motion_ros::AchMonitor<hubo_manip_state> stateChannel("manip-state");
	hubo_motion_ros::AchMonitor<hubo_manip_cmd> stateChannel("manip-cmd");

	ros::Rate r(1);

	while(ros::ok())
	{
		fprintf(stderr, "a.");
		stateChannel.waitState(1000);
		fprintf(stderr, "b");
		r.sleep();
	}

	return 0;
}

