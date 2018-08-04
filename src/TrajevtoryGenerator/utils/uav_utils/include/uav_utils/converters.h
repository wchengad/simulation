#ifndef __UAVUTILS_CONVERTERS_H
#define __UAVUTILS_CONVERTERS_H

#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>

namespace uav_utils
{
	void extract_odometry(nav_msgs::OdometryConstPtr msg, Eigen::Vector3d& p, 
		Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& w);
	void extract_odometry(nav_msgs::OdometryConstPtr msg, Eigen::Vector3d& p, 
		Eigen::Vector3d& v, Eigen::Quaterniond& q);

	template<typename T>
	T toRad(const T& x)
	{
		return x/180.0*M_PI;
	}

	template<typename T>
	T toDeg(const T& x)
	{
		return x*180.0/M_PI;
	}
}


#endif