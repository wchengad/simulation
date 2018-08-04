#include "pid_controller.h"
#include <algorithm>
#include <math.h>

using namespace std;


void pid_control_xyz(quadrotor_msgs::PositionCommand& cmd, colvec cur_p, double cur_yaw,
                     colvec desired_p, double desired_yaw, double depth, double t)
{
    static double pre_t = 0;
    // double delta_t = pre_t == 0.01 ? 0 : t - pre_t;
    double delta_t = pre_t == 0 ? 0.01 : t - pre_t;
    pre_t = t;

    cmd.velocity.x = pid_control_x(cur_p(0), desired_p(0), depth, delta_t);
    cmd.velocity.y = pid_control_y(cur_p(1), desired_p(1), delta_t);
    cmd.velocity.z = pid_control_z(cur_p(2), desired_p(2), delta_t);
    cmd.yaw_dot = pid_control_yaw(cur_yaw, desired_yaw, delta_t);
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

    // using cmd.position send to reconstruction model the UAV's coordinates in RVIZ
    cmd.position.x = cur_p(0);
    cmd.position.y = cur_p(1);
    cmd.position.z = cur_p(2);
}
/// TODO: Add the depth hard-threshold
double pid_control_x(double cur_x, double desired_x, double depth, double delta_t)
{
    const double Max_X_Vel = 0.8, Min_X_Vel = -1.0;

//P controller
    const double X_KP_hgt_front = 1.2;
    const double X_KP_low_front = 0.6;

    const double X_KP_hgt_back = 1.6;
    const double X_KP_low_back = 0.8;

//D controller
    const double X_KD_hgt_front = 0.08;
    const double X_KD_low_front = 0.03;

    const double X_KD_hgt_back = 0.12;
    const double X_KD_low_back = 0.05;

//For error
    double error_x = desired_x - cur_x;
    static double pre_error_x;
    static  double pre_xd_filter = 0;

    double x_vel_p, x_vel_d;

    // forward motion
    if(error_x > 0){
        if(error_x > 0.1) {
            x_vel_p = error_x * X_KP_hgt_front;
            x_vel_d = (error_x - pre_error_x) / (delta_t)* X_KD_hgt_front;
        } else {
            x_vel_p = error_x * X_KP_low_front;
            x_vel_d = (error_x - pre_error_x) / (delta_t)* X_KD_low_front;
        }
    }
    // backward motion
    else{
        if(error_x < -0.1) {
            x_vel_p = error_x * X_KP_hgt_back;
            x_vel_d = (error_x - pre_error_x) / (delta_t)* X_KD_hgt_back;
        } else {
            x_vel_p = error_x * X_KP_low_back;
            x_vel_d = (error_x - pre_error_x) / (delta_t)* X_KD_low_back;
        }
    }



    double vel_x = x_vel_p - x_vel_d;

//     printf("--------------------------\n@@@@---error_x %f, x_vel_p %f\n", error_x, x_vel_p);
//     printf("del_error %f, del_t %f, x_vel_d %f\n",error_x - pre_error_x, (delta_t), x_vel_d);

    if(vel_x > 0)
        vel_x = min(vel_x, Max_X_Vel);
    else
        vel_x = max(vel_x, Min_X_Vel);

    //printf("x_velocity %f \n", vel_x);

    pre_error_x = error_x;

    return vel_x;

}

double pid_control_y(double cur_y, double desired_y, double delta_t)
{
    const double Max_Y_Vel = 0.7, Min_Y_Vel = -0.7;

// P controoler
    const double Drone_Y_KP = 0.5;
    const double Drone_Y_KP_hgt = 1.5;
    const double Drone_Y_KP_mid = 0.9;
    const double Drone_Y_KP_low = 0.7;

// D controller
    const double Drone_Y_KD = 0.05;
    const double Drone_Y_KD_hgt = 0.1;
    const double Drone_Y_KD_mid = 0.05;
    const double Drone_Y_KD_low = 0.05;

    double error_y = desired_y - cur_y;
    static double pre_error_y;

    double y_vel_d, y_vel_p;
    if(abs(error_y) > 0.3){
        y_vel_d = (error_y - pre_error_y) / (delta_t)* Drone_Y_KD_hgt;
        y_vel_p = error_y * Drone_Y_KP_hgt;
    }
    else if (abs(error_y) > 0.15){
        y_vel_d = (error_y - pre_error_y) / (delta_t)* Drone_Y_KD_mid;
        y_vel_p = error_y * Drone_Y_KP_mid;
    }
    else {
        y_vel_d = (error_y - pre_error_y) / delta_t* Drone_Y_KD_low;
        y_vel_p = error_y * Drone_Y_KP_low;
    }

    //double y_vel_d = (error_y - pre_error_y) / delta_t* Drone_Y_KD;
    //double y_vel_p = error_y * Drone_Y_KP;
    double vel_y = y_vel_p - y_vel_d;

//     printf("--------------------------\n@@@@---error_y %f, y_vel_p %f\n", error_y, y_vel_p);
//     printf("del_error_y %f, del_t %f, y_vel_d %f\n",error_y - pre_error_y, delta_t, y_vel_d);
    // drone_ctrl_data.x = x_velocity * 0.8 + drone_state.drone_velocity.vx_body_level * 0.2;
    if(vel_y > 0)
        vel_y = min(vel_y, Max_Y_Vel);
    else
        vel_y = max(vel_y, Min_Y_Vel);

//     printf("@@@@---y_velocity %f error_y %f \n",vel_y, error_y);

    pre_error_y = error_y;

    return vel_y;
}

double pid_control_z(double cur_z, double desired_z, double delta_t)
{
    const double Drone_Z_KP = 1.0;
    const double Max_Z_Vel = 0.5, Min_Z_Vel = -0.5;

    double error_z = desired_z - cur_z;

    double vel_z;
    vel_z = error_z * Drone_Z_KP;
    if(vel_z > 0)
        vel_z = min(vel_z, Max_Z_Vel);
    else
        vel_z = max(vel_z, Min_Z_Vel);

    //printf("z_velocity %f error_z %f \n", vel_z , error_z);

    return -vel_z;
}


double pid_control_yaw(double cur_yaw, double desired_yaw, double delta_t)
{
    const double Max_Yaw_Vel = M_PI/2, Min_Yaw_Vel = -M_PI/2;
    /// P controller
    const double Drone_Yaw_KP = 3;
    /// D controller
    const double Drone_Yaw_KD = 0.05;

    double error_yaw = desired_yaw - cur_yaw;
    if(error_yaw > M_PI)
        error_yaw = error_yaw - M_PI * 2;
    else if(error_yaw < - M_PI)
        error_yaw = error_yaw + M_PI * 2;

    static double pre_error_yaw;

    double vel_yaw_p, vel_yaw_d;
    double vel_yaw;

    vel_yaw_p = error_yaw * Drone_Yaw_KP;
    vel_yaw_d = (error_yaw - pre_error_yaw) / (delta_t)* Drone_Yaw_KD;
    vel_yaw = vel_yaw_p - vel_yaw_d;
    // vel_yaw = error_yaw * Drone_Yaw_KP;

    if(vel_yaw > 0)
        vel_yaw = min(vel_yaw, Max_Yaw_Vel);
    else
        vel_yaw = max(vel_yaw, Min_Yaw_Vel);

//    printf("desired_yaw %f cur_yaw %f error_yaw %f \n", desired_yaw, cur_yaw , error_yaw);

    pre_error_yaw = error_yaw;
    return vel_yaw;
}






