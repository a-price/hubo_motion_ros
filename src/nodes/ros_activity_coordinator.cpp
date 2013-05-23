/**
 * \file ros_activity_coordinator.cpp
 * \brief 
 *
 *  \date April 15, 2013
 *  \author Andrew Price
 */

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "hubo_motion_ros/ExecutePoseTrajectoryAction.h"
#include "hubo_motion_ros/ExecuteJointTrajectoryAction.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

geometry_msgs::Pose generateGCP(tf::StampedTransform objectPose, unsigned int objectID = 0)
{
	geometry_msgs::Pose gcp;
	Eigen::Isometry3f eObj = Eigen::Isometry3f::Identity(),
			eLocalGcp = Eigen::Isometry3f::Identity(),
			eWorldGcp = Eigen::Isometry3f::Identity();
	//if (objectID == 0) // cylinder
	{
		const double radius = 0.04;
		const double graspHeight = 0.04;
		const double graspAngle = -3*M_PI/4;

		eLocalGcp.translate(Eigen::Vector3f(radius*cos(graspAngle),radius*sin(graspAngle),graspHeight));
		eLocalGcp.rotate(Eigen::AngleAxisf(M_PI+graspAngle, Eigen::Vector3f::UnitZ()));

		//eObj.translation() = Eigen::Map<Eigen::Vector3f>((float*)objectPose.getOrigin().m_floats);
		eObj.translation().x() = objectPose.getOrigin().x();
		eObj.translation().y() = objectPose.getOrigin().y();
		eObj.translation().z() = objectPose.getOrigin().z();

		eWorldGcp = eObj * eLocalGcp;
		//eWorldGcp = eLocalGcp;

		gcp.position.x = eWorldGcp.translation().x();
		gcp.position.y = eWorldGcp.translation().y();
		gcp.position.z = eWorldGcp.translation().z();

		Eigen::Quaternionf q(eWorldGcp.rotation());
		gcp.orientation.w = q.w();
		gcp.orientation.x = q.x();
		gcp.orientation.y = q.y();
		gcp.orientation.z = q.z();
	}


	return gcp;
}

bool requestPose(hubo_motion_ros::ExecutePoseTrajectoryGoal goal)
{
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<hubo_motion_ros::ExecutePoseTrajectoryAction> ac("manip_traj_forwarder_pose", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_activity_coordinator");
	ros::NodeHandle nh;
	ROS_INFO("Started ros_activity_coordinator.");

	tf::TransformListener listener;
	tf::TransformBroadcaster tfBroadcaster;


	//pose_client = nh.serviceClient<hubo_drc_vision::SetHuboObjectPose>("/hubo/set_object");

	std::string text;
	hubo_motion_ros::ExecutePoseTrajectoryGoal goal;
	geometry_msgs::PoseArray pArray;
	geometry_msgs::Pose gcpPose;

	tf::Transform gcpTrans;

	while (ros::ok())
	{
		std::getline(std::cin, text);

		// Get pose in body frame, grab it. eventually in a different node
		tf::StampedTransform tTorsoObject;
		try
		{
			listener.lookupTransform("/Body_Torso", "/cylinder_gcp", ros::Time(0), tTorsoObject);
			ROS_INFO("Got transform!");

			tf::poseTFToMsg(tf::Transform(tTorsoObject), gcpPose);
			//gcpPose = generateGCP(tTorsoObject, 0);


			// publish gcp
			//tf::poseMsgToTF(gcpPose, gcpTrans);
			//tf::transformMsgToTF(gcpPose, gcpTrans);
			//tfBroadcaster.sendTransform(tf::StampedTransform(
			//		gcpTrans, ros::Time::now(), "/Body_Torso", "/cylinder_gcp"));

			pArray.header.frame_id = "/Body_Torso";
			pArray.poses.push_back(gcpPose);
			goal.PoseTargets.push_back(pArray);
			goal.ArmIndex.push_back(0);
			requestPose(goal);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
		}
	}

	return 0;
}

