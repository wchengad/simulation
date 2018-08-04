#ifndef __GEOMETRY_UTILS_H
#define __GEOMETRY_UTILS_H

#include <eigen3/Eigen/Dense>

Eigen::Matrix3d ypr_to_R(const Eigen::Vector3d& ypr);

Eigen::Vector3d R_to_ypr(const Eigen::Matrix3d& R);

Eigen::Vector3d quaternion_to_ypr(const Eigen::Quaterniond& q);

double get_yaw_from_quaternion(const Eigen::Quaterniond& q);

Eigen::Quaterniond yaw_to_quaternion(double yaw);

template <typename T>
Eigen::Matrix3d rotx(T t)
{
	Eigen::Matrix3d R;
	R(0,0) = 1;		R(0,1) = 0;				R(0,2) = 0;
	R(1,0) = 0;		R(1,1) = std::cos(t);	R(1,2) = -std::sin(t);
	R(2,0) = 0;		R(2,1) = std::sin(t);	R(2,2) =  std::cos(t);

	return R;
}

template <typename T>
Eigen::Matrix3d roty(T t)
{
	Eigen::Matrix3d R;
	R(0,0) =  std::cos(t);	R(0,1) = 0;		R(0,2) = std::sin(t);
	R(1,0) =  0;			R(1,1) = 1;		R(1,2) = 0;
	R(2,0) = -std::sin(t);	R(2,1) = 0;		R(2,2) = std::cos(t);

	return R;
}

template <typename T>
Eigen::Matrix3d rotz(T t)
{
	Eigen::Matrix3d R;
	R(0,0) = std::cos(t);	R(0,1) = -std::sin(t);	R(0,2) = 0;
	R(1,0) = std::sin(t);	R(1,1) =  std::cos(t);	R(1,2) = 0;
	R(2,0) = 0;				R(2,1) =  0;			R(2,2) = 1;

	return R;
}

template <typename T>
T normalize_angle(T a)
{
	int cnt = 0;
	while(true)
	{
		cnt++;

		if (a < -M_PI)
		{
			a += M_PI*2;
		}
		else if (a > M_PI)
		{
			a -= M_PI*2;
		}
		
		if (-M_PI<=a && a <=M_PI)
		{
			break;
		};
		
		assert(cnt < 10 && "[uav_utils/geometry_msgs] INVALID INPUT ANGLE");
	}

	return a;
}

template <typename T>
T yaw_add(T a, T b)
{
	T c = a + b;
	c = normalize_angle(c);
	assert(-M_PI<=c && c <=M_PI);
	return c;
}


#endif