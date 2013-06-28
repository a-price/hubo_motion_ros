/**
 * \file hubo_hw_state_publisher.cpp
 * \brief ROS node that publishes topics for Hubo's joint and IMU states
 *
 * \date May 20, 2013
 * \author Andrew Price
 */

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hubo_motion_ros/HuboStateAchBridge.h"

namespace hubo_motion_ros
{

/**
 * \class HuboHWStatePublisher
 * \brief Handles the retrieval of state information from an Ach channel and publishes it as a series of ROS topics
 */
class HuboHWStatePublisher
{
public:
	HuboHWStatePublisher() :
		m_HuboState()
	{
		m_JointPublisher = m_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
		m_ImuPublisher = m_nh.advertise<sensor_msgs::Imu>("imu", 1);
	}

	void publishState()
	{
		sensor_msgs::Imu iState = m_HuboState.getIMUState(true);
		iState.header.frame_id = "/Body_Hip";
		m_ImuPublisher.publish(iState);

		sensor_msgs::JointState jState = m_HuboState.getJointState(false);
		jState.name.push_back("HNP");
		jState.position.push_back(0);
		jState.velocity.push_back(0);

		jState.name.push_back("HNR");
		jState.position.push_back(0);
		jState.velocity.push_back(0);

		const hubo_state& state = m_HuboState.getState(false);

		for (int hand = 0; hand < 2; hand++)
		{
			std::string handName;
			std::string jointName;
			if (hand == LEFT) {handName = "left";}
			else {handName = "right";}

			for (int finger = 0; finger < sizeof(DRCHUBO_URDF_FINGER_NAMES)/sizeof(std::string); finger++)
			{
				for (int joint = 2; joint <= sizeof(DRCHUBO_URDF_FINGER_LINK_NAMES)/sizeof(std::string); joint++)
				{
					jointName = handName + DRCHUBO_URDF_FINGER_NAMES[finger] + "Knuckle" + std::to_string(joint);
					jState.name.push_back(jointName);
					jState.position.push_back(0);
					jState.velocity.push_back(0);
				}
			}
		}

		m_JointPublisher.publish(jState);

		// Have to publish head separately, since we don't have an accurate URDF
		// TODO: Replace with values derived from known angles + calibration
		Eigen::Affine3f psA, psB;
		psA = Eigen::Affine3f::Identity();
		psB = Eigen::Affine3f::Identity();

		const float alpha = 0.84, beta = 0;//0.86602540378, beta = 1.04719755;
		psA.translate(Eigen::Vector3f(0.04, 0.04, 0.015));
		psA.rotate(Eigen::Quaternionf(cos(alpha/2), 0, sin(alpha/2), 0).normalized());

		psB.translate(Eigen::Vector3f(0.035, 0.04, 0.073));
		psB.rotate(Eigen::Quaternionf(cos(beta/2), 0, sin(beta/2), 0).normalized());

		tf::Transform tfA,tfB;
		tf::transformEigenToTF(psA.cast<double>(), tfA);
		tf::transformEigenToTF(psB.cast<double>(), tfB);

		m_TFBroad.sendTransform(tf::StampedTransform(tfA, ros::Time::now(), "/Body_HNP", "/camera_link"));
		m_TFBroad.sendTransform(tf::StampedTransform(tfB, ros::Time::now(), "/Body_HNP", "/camera_link1"));
	}

protected:
	ros::NodeHandle m_nh;
	ros::Publisher m_JointPublisher;
	ros::Publisher m_ImuPublisher;

	tf::TransformBroadcaster m_TFBroad;

	HuboStateAchBridge m_HuboState;
};

} // namespace hubo_motion_ros

int main(int argc, char** argv)
{
	ROS_INFO("Started hubo_hw_state_publisher.");
	ros::init(argc, argv, "hw_state_publisher");

	// TODO: attempt to create the state channels here

	hubo_motion_ros::HuboHWStatePublisher publisher;

	// TODO: parameterize
	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
		publisher.publishState();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}

