#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <quadrotor_msgs/PositionCommand.h>
#include <armadillo>
using namespace arma;

void pid_control_xyz(quadrotor_msgs::PositionCommand& cmd, colvec cur_p, double cur_yaw,
                     colvec desired_p, double desired_yaw, double depth, double t);
double pid_control_x(double cur_x, double desired_x, double depth, double delta_t);
double pid_control_y(double cur_y, double desired_y, double delta_t);
double pid_control_z(double cur_z, double desired_z, double delta_t);
double pid_control_yaw(double cur_yaw, double desired_yaw, double delta_t);

#endif