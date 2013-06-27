/**
 * \file TestIK.cpp
 * \brief Unit tests for forward and Inverse Kinematics
 *
 * \author Maxwell McRae
 */

#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <hubo.h>
#include "HuboKin.h"

typedef Eigen::Matrix<float, 6, 1> Vector6f;

typedef struct
{
	float x, y, z, error;
} reachability_t;

float compareT(Eigen::Isometry3f& a, Eigen::Isometry3f& b,
		Eigen::VectorXf weight)
{
	Eigen::Quaternionf qa(a.rotation());
	Eigen::Quaternionf qb(b.rotation());
	Eigen::Vector3f pa = a.translation();
	Eigen::Vector3f pb = b.translation();
	Eigen::VectorXf va(7), vb(7), verr(7), vScaled(7);
	va << pa, qa.x(), qa.y(), qa.z(), qa.w();
	vb << pb, qb.x(), qb.y(), qb.z(), qb.w();
	verr = vb - va;
	vScaled = weight.cwiseProduct(verr);
	return vScaled.squaredNorm();
}

// TODO: consider orientations other than identity
std::vector<reachability_t> TestReachability(float rotation)
{
	const float XMIN = -0.1, XMAX = 0.7, YMIN = -0.8, YMAX = .2, ZMIN = -1, ZMAX =
			0.3, STEPSIZE = 0.05;
	HK::HuboKin hubo;
	std::vector<reachability_t> results;

	Eigen::VectorXf weight(7);
	weight << 1, 1, 1, 1, 1, 1, 1; //change this for weights!
	Vector6f q;
	int count = 0, valid = 0;
	for (float x = XMIN; x <= XMAX; x += STEPSIZE)
	{
		for (float y = YMIN; y <= YMAX; y += STEPSIZE)
		{
			for (float z = ZMIN; z <= ZMAX; z += STEPSIZE)
			{
				++count;
				Eigen::Isometry3f target = Eigen::Isometry3f::Identity();
				Eigen::Isometry3f result;
				target.translation().x() = x;
				target.translation().y() = y;
				target.translation().z() = z;
				target.rotate(Eigen::AngleAxisf(-(M_PI/6) * rotation, Eigen::Vector3f::UnitX()));
				hubo.armIK(q, target, Vector6f::Zero(), RIGHT);
				hubo.armFK(result, q, RIGHT);
				float ret = compareT(target, result, weight);


				//std::cout << ret << "\t";
				if (ret > 0.45)
				{
					continue;
				}
				else
				{
					valid++;
					reachability_t pointScore;
					pointScore.x = x;
					pointScore.y = y;
					pointScore.z = z;
					pointScore.error = ret;
					results.push_back(pointScore);
				}

			}

		}

	}

	return results;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ROSHuboTester");

	ROS_INFO("Starting Hubo Test Publisher.\n");

	ros::NodeHandle nh;
	ros::Publisher resultPublisher;

	resultPublisher = nh.advertise<visualization_msgs::MarkerArray>( "grasp_points", 0 );

	ros::Rate rate(1);
	float count;
	while (ros::ok())
	{
		std::vector<reachability_t> results = TestReachability(count);
		visualization_msgs::MarkerArray mArray;

		float maxError=0;

		for (int j = 0; j < results.size(); j++)
		{
			reachability_t point = results[j];
			if (point.error>maxError)maxError=point.error;
		}

		for (int i = 0; i < results.size(); i++)
		{
			reachability_t point = results[i];

			visualization_msgs::Marker marker;
			marker.header.frame_id = "Body_Torso";
			//marker.header.stamp = headerTime;
			marker.ns = "HuboApplication";
			marker.id = i;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = point.x;
			marker.pose.position.y = point.y;
			marker.pose.position.z = point.z;

			marker.scale.x = 0.015;//*point.error/maxError;
			marker.scale.y = 0.015;//*point.error/maxError;
			marker.scale.z = 0.015;//*point.error/maxError;
			marker.color.a = 0.5;//(1-point.error/maxError)*.70; // based on # & weight of connections

			if (point.error < 0.005)
			{
				marker.color.r = 0; // based on parent
				marker.color.g = 1; //
				marker.color.a = 1;
			}
			else if (point.error < 0.045)
			{
				marker.color.r = 1; // based on parent
				marker.color.g = 0; //
				marker.color.a = 1;
			}
			else
			{
				marker.color.r = 1; // based on parent
				marker.color.g = 0; //
				marker.color.a = 0;
			}
			//marker.color.r = point.error/maxError; // based on parent
			//marker.color.g = 1-point.error/maxError; //
			marker.color.b = 0.0;
			mArray.markers.push_back(marker);
		}

		resultPublisher.publish(mArray);

		ros::spinOnce();

		//rate.sleep();
		count++;
	}


}
