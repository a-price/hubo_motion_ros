/**
 * \file HuboStateAchBridge.h
 * \brief Contains provider for ROS representation of Hubo's state
 * 
 * \author Andrew Price
 */

#ifndef HUBO_STATE_ACH_BRIDGE_H
#define HUBO_STATE_ACH_BRIDGE_H

#include <hubo.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include "hubo_motion_ros/AchROSBridge.h"
#include "hubo_motion_ros/drchubo_joint_names.h"

#define COM_IMU_INDEX 0

namespace hubo_motion_ros
{

/**
 * \class HuboStateAchBridge
 * \brief Provides ROS representation of Hubo's state by inheriting from AchROSBridge
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

} // namespace hubo_motion_ros

#endif // HUBO_STATE_ACH_BRIDGE_H
