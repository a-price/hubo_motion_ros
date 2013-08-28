/**
 * \file TestJointNames.cpp
 * \brief Verifies operation of ParameterizedObject classes
 *
 * \author Andrew Price
 * \date July 7, 2013
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

//#include "gtest/gtest.h"
#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "hubo_motion_ros/drchubo_joint_names.h"

//using namespace hubo_motion_ros;

class TestJointNames : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestJointNames );
	CPPUNIT_TEST(TestNameGetSet);
	CPPUNIT_TEST_SUITE_END();
public:

	virtual void setUp()
	{
	}

	virtual void tearDown () {}

	void TestNameGetSet()
	{
		for (int i = 0; i < HUBO_JOINT_COUNT; i++)
		{
			std::string jointName = DRCHUBO_URDF_JOINT_NAMES[i];
			if (jointName == "")
			{
				continue;
			}
			auto jointIter = DRCHUBO_JOINT_NAME_TO_INDEX.find(jointName);
			CPPUNIT_ASSERT_MESSAGE("Unable to find " + jointName + " in lookup.", DRCHUBO_JOINT_NAME_TO_INDEX.end() != jointIter);
			int jointIndex = jointIter->second;
			CPPUNIT_ASSERT_EQUAL(i, jointIndex);
		}

		for (const auto& arm : DRCHUBO_ARM_INDEX_TO_NAMES)
		{
			int side = arm.first;
			std::vector<std::string> jointNames = arm.second;

			for (std::string jointName : jointNames)
			{
				auto jointIter = DRCHUBO_JOINT_NAME_TO_LIMB.find(jointName);
				CPPUNIT_ASSERT_MESSAGE("Unable to find " + jointName + " in lookup.", DRCHUBO_JOINT_NAME_TO_LIMB.end() != jointIter);
				int jointSide = jointIter->second;
				CPPUNIT_ASSERT_EQUAL(side, jointSide);
			}
		}
	}
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestJointNames);

//int main(int argc, char **argv)
//{
//	::testing::InitGoogleTest(&argc, argv);
//	return RUN_ALL_TESTS();
//}
