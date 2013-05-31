/**
 *
 * \file PoseConverter.cpp
 * \brief 
 *
 * \author Andrew Price
 * \date May 30, 2013
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

#include "hubo_motion_ros/PoseConverter.h"

namespace hubo_motion_ros
{

Eigen::Isometry3d toIsometry(geometry_msgs::Pose pose)
{
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	Eigen::Vector3d trans(pose.position.x,pose.position.y,pose.position.z);
	Eigen::Quaterniond quat(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);

	result.translate(trans);
	result.rotate(quat);

	return result;
}
Eigen::Isometry3d toIsometry(tf::Transform pose)
{
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	tf::Vector3 tVec = pose.getOrigin();
	tf::Quaternion tQuat = pose.getRotation();
	Eigen::Vector3d trans(tVec.x(),tVec.y(),tVec.z());
	Eigen::Quaterniond quat(tQuat.w(),tQuat.x(),tQuat.y(),tQuat.z());

	result.translate(trans);
	result.rotate(quat);

	return result;
}
//
//tf::Transform toTF(geometry_msgs::Pose pose)
//{
//
//}
//tf::Transform toTF(Eigen::Isometry3d pose)
//{
//
//}
//
geometry_msgs::Pose toPose(Eigen::Isometry3d pose)
{
	geometry_msgs::Pose result;
	Eigen::Vector3d eTrans = pose.translation();
	Eigen::Quaterniond eQuat(pose.rotation());

	result.position.x = eTrans[0];
	result.position.y = eTrans[1];
	result.position.z = eTrans[2];
	result.orientation.w = eQuat.w();
	result.orientation.x = eQuat.x();
	result.orientation.y = eQuat.y();
	result.orientation.z = eQuat.z();

	return result;
}
//geometry_msgs::Pose toPose(tf::Transform pose)
//{
//
//}

} /* namespace hubo_manipulation_planner */
