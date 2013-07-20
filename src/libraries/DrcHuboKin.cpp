/**
 * \file DrcHuboKin.cpp
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

#include "hubo_motion_ros/DrcHuboKin.h"
#include <hubo.h>

inline double mod(double x, double y)
{
	if (0 == y)
		return x;

	return x - y * floor(x/y);
}

inline double wrapToPi(double fAng)
{
	return mod(fAng + M_PI, 2.0*M_PI) - M_PI;
}

DrcHuboKin::DrcHuboKin(std::string urdf, bool isFiletext) : RobotKin::Robot()
{
	this->name("drchubo");
	if (isFiletext)
	{
		this->loadURDFString(urdf);
	}
	else
	{
		this->loadURDF(urdf);
	}
	linkage("Body_RSP").name("RightArm");
	linkage("Body_LSP").name("LeftArm");
	linkage("Body_RHY").name("RightLeg");
	linkage("Body_LHY").name("LeftLeg");

	std::cerr << this->joints().size() << " joints loaded." << std::endl;

}

RobotKin::rk_result_t DrcHuboKin::armIK(int side, ArmVector &q, const Eigen::Isometry3d B)
{
	Eigen::VectorXd xq(q);
	RobotKin::rk_result_t result;

	std::string armName;
	if(side==LEFT)
		armName = "LeftArm";
	else
		armName = "RightArm";

	result = dampedLeastSquaresIK_linkage(armName, xq, B);

	for (int i = 0; i < xq.size(); i++)
	{
		q[i] = xq[i];
	}

	return result;
}

RobotKin::rk_result_t DrcHuboKin::legIK(int side, LegVector &q, const Eigen::Isometry3d B)
{
	Eigen::VectorXd xq(q);
	RobotKin::rk_result_t result;

	std::string armName;
	if(side==LEFT)
		armName = "LeftLeg";
	else
		armName = "RightLeg";

	result = dampedLeastSquaresIK_linkage(armName, xq, B);

	return result;
}
