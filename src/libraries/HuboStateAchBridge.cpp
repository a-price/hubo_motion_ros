/**
 * \file HuboStateAchBridge.cpp
 * \brief Provides ROS representation of Hubo's hardware state
 * 
 * \author Andrew Price
 */

#include "hubo_motion_ros/HuboStateAchBridge.h"

namespace hubo_motion_ros
{

HuboStateAchBridge::HuboStateAchBridge(std::string chanName) :
	AchROSBridge(chanName)
{
}

HuboStateAchBridge::~HuboStateAchBridge()
{
}

sensor_msgs::JointState HuboStateAchBridge::getJointState(bool update)
{
	if (update)
	{
		updateState();
	}

	sensor_msgs::JointState js;

	for (int i = 0; i < HUBO_JOINT_COUNT; i++)
	{
		js.position.push_back(((hubo_state)mAchData).joint[i].pos);
		js.velocity.push_back(((hubo_state)mAchData).joint[i].vel);
		js.name.push_back(HUBO_URDF_JOINT_NAMES[i]);
	}

	// How is hubo time specified?
	js.header.stamp = ros::Time::now(); // (((hubo_state)mAchData).time);

	return js;
}

sensor_msgs::Imu HuboStateAchBridge::getIMUState(bool update)
{
	if (update)
	{
		updateState();
	}

	sensor_msgs::Imu imu;
	geometry_msgs::Vector3 w, a;
	geometry_msgs::Quaternion q;

	((hubo_state)mAchData).imu[COM_IMU_INDEX].a_y = 90.0;

	Eigen::Isometry3f t = Eigen::Isometry3f::Identity();
	t.rotate(Eigen::AngleAxisf(-((hubo_state)mAchData).imu[COM_IMU_INDEX].a_x*M_PI/180.0, Eigen::Vector3f::UnitX()));
	t.rotate(Eigen::AngleAxisf(-((hubo_state)mAchData).imu[COM_IMU_INDEX].a_y*M_PI/180.0, Eigen::Vector3f::UnitY()));
	t.rotate(Eigen::AngleAxisf(-((hubo_state)mAchData).imu[COM_IMU_INDEX].a_z*M_PI/180.0, Eigen::Vector3f::UnitZ()));
	Eigen::Quaternionf orientation(t.rotation());

	Eigen::Vector4f pos = t*-Eigen::Vector4f::UnitZ()*9.8;

	q.w = orientation.w();
	q.x = orientation.x();
	q.y = orientation.y();
	q.z = orientation.z();

//	fprintf(stderr, "Orientation: %f,%f,%f,%f\n", q.w,q.x,q.y,q.z);

	a.x = pos.x();
	a.y = pos.y();
	a.z = pos.z();

	w.x = ((hubo_state)mAchData).imu[COM_IMU_INDEX].w_x;
	w.y = ((hubo_state)mAchData).imu[COM_IMU_INDEX].w_y;
	w.z = ((hubo_state)mAchData).imu[COM_IMU_INDEX].w_z;

	imu.orientation = q;
	imu.linear_acceleration = a;
	imu.angular_velocity = w;

	imu.header.stamp = ros::Time::now();

	return imu;
}
 
} //namespace hubo_motion_ros
