/**
 * \file HuboStateAchBridge.h
 * \brief Provides ROS representation of Hubo's state
 * 
 * \author Andrew Price
 */

#ifndef HUBO_STATE_ACH_BRIDGE_H
#define HUBO_STATE_ACH_BRIDGE_H

#include "hubo_motion_ros/AchROSBridge.h"

#include <hubo.h>
#include "hubo_motion_ros/hubo_joint_names.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#define COM_IMU_INDEX 0

/**
 * \class HuboStateAchBridge
 * \brief Provides ROS representation of Hubo's state
 */
class HuboStateAchBridge : public AchROSBridge<hubo_state>
{
public:
	/// Constructor
	HuboStateAchBridge(std::string chanName = "hubo-state");

	/// Destructor
	~HuboStateAchBridge();

	sensor_msgs::JointState getJointState(bool update = true);
	sensor_msgs::Imu getIMUState(bool update = true);
};


#endif // HUBO_STATE_ACH_BRIDGE_H
