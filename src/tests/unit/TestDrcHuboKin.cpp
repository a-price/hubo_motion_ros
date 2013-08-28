/**
 * \file TestDrcHuboKin.cpp
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


#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <hubo.h>

#include "hubo_motion_ros/DrcHuboKin.h"
#include "hubo_motion_ros/drchubo_joint_names.h"

//using namespace hubo_motion_ros;

class TestDrcHuboKin : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestDrcHuboKin );
	CPPUNIT_TEST(TestFKIKFK);
	CPPUNIT_TEST_SUITE_END();
public:

	virtual void setUp()
	{
		std::string urdf = "/home/arprice/catkin_workspace/src/drchubo/drchubo_v2/robots/drchubo_v2.urdf";
		kinematics = new DrcHuboKin(urdf, false);
		srand(time(NULL));
	}

	virtual void tearDown () {}

	float randbetween(double min, double max)
	{
		return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
	}

	void TestFKIKFK()
	{
		int arm = RIGHT;
		// Create random legitimate joint values
		std::vector<RobotKin::Joint*> joints = kinematics->linkage("RightArm").joints();
		for (int i = 0; i < joints.size(); i++)
		{
			double jointVal = randbetween(joints[i]->min(), joints[i]->max());
			kinematics->joint(joints[i]->name()).value(jointVal);
		}

		// Do FK to get ee pose
		Eigen::Isometry3d initialFrame;
		initialFrame = kinematics->linkage("RightArm").tool().respectToRobot();

		// Do IK to get joints
		DrcHuboKin::ArmVector q = DrcHuboKin::ArmVector::Zero();
		RobotKin::rk_result_t result = kinematics->armIK(arm, q, initialFrame);

		std::cerr << "Solver result: " << result << std::endl;
		CPPUNIT_ASSERT(RobotKin::RK_SOLVED == result);

		// Do FK and verify that it's pretty much the same
		int baseJoint = 0;
		if (LEFT == arm)
		{ baseJoint = LSP; }
		else if (RIGHT == arm)
		{ baseJoint = RSP; }

		for (int i = baseJoint; i < baseJoint+7; i++)
		{
			//kinematics->joint(DRCHUBO_URDF_JOINT_NAMES[i]).value(q[DRCHUBO_JOINT_INDEX_TO_LIMB_POSITION[i]]);
		}

		Eigen::Isometry3d finalFrame;
		finalFrame = kinematics->linkage("RightArm").tool().respectToRobot();

		//CPPUNIT_ASSERT((initialFrame.matrix() - finalFrame.matrix()).norm() < 0.001);
		std::cerr << "Inital pose: \n" << initialFrame.matrix() << std::endl;
		std::cerr << "Final pose: \n" << finalFrame.matrix() << std::endl;
	}

	DrcHuboKin* kinematics;
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestDrcHuboKin);
