#include "geometry_utils.h"

using namespace Eigen;
using namespace std;

Matrix3d ypr_to_R(const Vector3d& ypr)
{
    double c, s;
    Matrix3d Rz = Matrix3d::Zero();
    double y = ypr(0);
    c = cos(y);
    s = sin(y);
    Rz(0,0) =  c;
    Rz(1,0) =  s;
    Rz(0,1) = -s;
    Rz(1,1) =  c;
    Rz(2,2) =  1;

    Matrix3d Ry = Matrix3d::Zero();
    double p = ypr(1);
    c = cos(p);
    s = sin(p);
    Ry(0,0) =  c;
    Ry(2,0) = -s;
    Ry(0,2) =  s;
    Ry(2,2) =  c;
    Ry(1,1) =  1;

    Matrix3d Rx = Matrix3d::Zero();
    double r = ypr(2);
    c = cos(r);
    s = sin(r);
    Rx(1,1) =  c;
    Rx(2,1) =  s;
    Rx(1,2) = -s;
    Rx(2,2) =  c;
    Rx(0,0) =  1;

    Matrix3d R = Rz*Ry*Rx;
    return R;
}

Vector3d R_to_ypr(const Matrix3d& R)
{
    Vector3d n = R.col(0);
    Vector3d o = R.col(1);
    Vector3d a = R.col(2);

    Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0)*cos(y)+n(1)*sin(y));
    double r = atan2(a(0)*sin(y)-a(1)*cos(y), -o(0)*sin(y)+o(1)*cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
}


Eigen::Vector3d quaternion_to_ypr(const Eigen::Quaterniond& q)
{
    return R_to_ypr(q.toRotationMatrix());
}

double get_yaw_from_quaternion(const Eigen::Quaterniond& q)
{
    return quaternion_to_ypr(q)(0);
}

Quaterniond yaw_to_quaternion(double yaw)
{
    return Quaterniond(rotz(yaw));
}
