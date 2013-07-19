/**
 * \file test_kinematics_server.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 7 19, 2013
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

ros::ServiceClient gKinClient;

geometry_msgs::PoseStamped createSimplePoseGoal()
{
	geometry_msgs::PoseStamped poseStamped;

	poseStamped.header.frame_id = "/Body_TSY";

	poseStamped.pose.position.x = 0.25;
	poseStamped.pose.position.y = -0.25;
	poseStamped.pose.position.z = -0.2;

	poseStamped.pose.orientation.w = 1.0;
	poseStamped.pose.orientation.x = 0.0;
	poseStamped.pose.orientation.y = 0.0;
	poseStamped.pose.orientation.z = 0.0;


	return poseStamped;
}

bool testIK(geometry_msgs::PoseStamped poseTarget)
{
	moveit_msgs::GetPositionIKRequest req;
	req.ik_request.group_name = "right_arm";
	req.ik_request.pose_stamped = poseTarget;

	moveit_msgs::GetPositionIKResponse resp;
	gKinClient.call(req, resp);
}

int main(int argc, char** argv)
{
	ROS_INFO("Started test_kinematics_server.");
	ros::init(argc, argv, "test_kinematics_server");

	ros::NodeHandle nh;

	gKinClient = nh.serviceClient<moveit_msgs::GetPositionIK>("/hubo/kinematics/ik_service");

	testIK(createSimplePoseGoal());

	return 0;
}
