/**
 *
 * \file PoseConverter.h
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

#ifndef POSECONVERTER_H_
#define POSECONVERTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

namespace hubo_motion_ros
{

Eigen::Isometry3d toIsometry(geometry_msgs::Pose pose);
Eigen::Isometry3d toIsometry(tf::Transform pose);

tf::Transform toTF(geometry_msgs::Pose pose);

template <typename Derived>
tf::Transform toTF(Eigen::Transform<Derived, 3, Eigen::Isometry> pose)
{
	tf::Transform t;
	Eigen::Matrix<Derived, 3, 1> eTrans = pose.translation();
	Eigen::Quaternion<Derived> eQuat(pose.rotation());
	t.setOrigin(tf::Vector3(eTrans.x(), eTrans.y(),eTrans.z()));
	t.setRotation(tf::Quaternion(eQuat.x(), eQuat.y(), eQuat.z(), eQuat.w()));
	return t;
}

template <typename Derived>
geometry_msgs::Pose toPose(Eigen::Transform<Derived, 3, Eigen::Isometry> pose)
{
	geometry_msgs::Pose result;
	Eigen::Matrix<Derived, 3, 1> eTrans = pose.translation();
	Eigen::Quaternion<Derived> eQuat(pose.rotation());

	result.position.x = eTrans.x();
	result.position.y = eTrans.y();
	result.position.z = eTrans.z();
	result.orientation.w = eQuat.w();
	result.orientation.x = eQuat.x();
	result.orientation.y = eQuat.y();
	result.orientation.z = eQuat.z();

	return result;
}

geometry_msgs::Pose toPose(tf::Transform pose);

} /* namespace hubo_manipulation_planner */
#endif /* POSECONVERTER_H_ */
