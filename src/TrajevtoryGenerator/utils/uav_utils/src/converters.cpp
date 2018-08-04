#include "converters.h"

namespace uav_utils
{
void extract_odometry(nav_msgs::OdometryConstPtr msg, Eigen::Vector3d& p,
                      Eigen::Vector3d& v, Eigen::Quaterniond& q, Eigen::Vector3d& w)
{
	extract_odometry(msg, p, v, q);

	w(0) = msg->twist.twist.angular.x;
	w(1) = msg->twist.twist.angular.y;
	w(2) = msg->twist.twist.angular.z;
}

void extract_odometry(nav_msgs::OdometryConstPtr msg, Eigen::Vector3d& p,
                      Eigen::Vector3d& v, Eigen::Quaterniond& q)
{
	p(0) = msg->pose.pose.position.x;
	p(1) = msg->pose.pose.position.y;
	p(2) = msg->pose.pose.position.z;

	v(0) = msg->twist.twist.linear.x;
	v(1) = msg->twist.twist.linear.y;
	v(2) = msg->twist.twist.linear.z;

	q.w() = msg->pose.pose.orientation.w;
	q.x() = msg->pose.pose.orientation.x;
	q.y() = msg->pose.pose.orientation.y;
	q.z() = msg->pose.pose.orientation.z;
}
}
