/**
 * \file simple_teleop.cpp
 * \brief
 *
 * \author Andrew Price
 * \date July 19, 2013
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
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <trajectory_msgs/JointTrajectory.h>


ros::Subscriber gPoseSubscriber;
ros::ServiceClient gIKinClient;
ros::Publisher gStatePublisher;

sensor_msgs::JointState planState;

void poseCallback(geometry_msgs::PoseStampedConstPtr poseIn)
{
	moveit_msgs::GetPositionIKRequest req;
	req.ik_request.group_name = "right_arm";
	req.ik_request.pose_stamped = *poseIn;
	req.ik_request.robot_state.joint_state = planState;

	moveit_msgs::GetPositionIKResponse resp;
	gIKinClient.call(req, resp);

	resp.solution.joint_state.name.push_back("TSY");
	resp.solution.joint_state.position.push_back(0.0);
	resp.solution.joint_state.header.frame_id = "/Body_TSY";
	resp.solution.joint_state.header.stamp = ros::Time::now();

	gStatePublisher.publish(resp.solution.joint_state);
	planState = resp.solution.joint_state;
}


int main(int argc, char** argv)
{
	ROS_INFO("Started simple_teleop.");
	ros::init(argc, argv, "simple_teleop");

	ros::NodeHandle m_nh;

	gPoseSubscriber = m_nh.subscribe("pose_in", 1, &poseCallback);
	gIKinClient = m_nh.serviceClient<moveit_msgs::GetPositionIK>("/hubo/kinematics/ik_service");
	gStatePublisher = m_nh.advertise<sensor_msgs::JointState>("joint_states", 1);

	ros::spin();

	return 0;
}
