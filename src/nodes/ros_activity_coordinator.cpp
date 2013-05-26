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

