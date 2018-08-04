//TODO: Add yaw planning
//TODO: Fix hover case
//TODO: Fix short segment crashes
//TODO: Find correct segment times based on initial condition
//TODO: Corridor constraint
//TODO: Fix reuse time issue, recompute time is the trajectory length is scaled siginficantly

#include <iostream>
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/OutputData.h>
#include <dynamic_reconfigure/server.h>
#include <trajectory_generator/TrajectoryGeneratorUIConfig.h>
#include <geometry_msgs/Quaternion.h>
//#include "pose_utils.h"
//#include "polynomial_trajectory_generator.h"
#include "pid_controller.h"
#include <sensor_msgs/PointCloud.h>
#include "armadillo"
#include <sys/types.h>
#include <sys/stat.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


#include "../../../../devel/include/quadrotor_msgs/PositionCommand.h"

using namespace arma;
using namespace std;

#define _PID_BODY


// ROS
ros::Publisher pubc;
ros::Publisher pubp;
ros::Publisher pubt;
ros::Publisher pubo;
ros::Publisher pubct;
ros::Publisher meshPub;
ros::Publisher meshPub2;
ros::Publisher meshPub3;
ros::Publisher arrowPub;
ros::Publisher arrowPub2;
ros::Publisher volumePubx;
ros::Publisher volumePuby;
ros::Publisher volumePubz;
ros::Publisher pointcloudPub;
quadrotor_msgs::PositionCommand position_cmd;
bool isConfig = false;

// Control Mode FSM
#define HOVER_MODE         1
#define POSITION_MODE    2
int control_mode = POSITION_MODE;

#define TARGET_BUFFER_SIZE    50

//#define ALIGN_TO_VOLUME
#define ALIGN_TO_RVIZ

// State
bool is_odom        = true;                                // First odom received
colvec p            = zeros<colvec>(3);         // Current state & time
colvec v            = zeros<colvec>(3);
colvec q            = zeros<colvec>(4);         // Current orientation
colvec desired_p    = zeros<colvec>(3);
colvec volume_p     = zeros<colvec>(3);
colvec volume_q     = zeros<colvec>(4);
colvec ypr          = zeros<colvec>(3);
colvec p_blender    = zeros<colvec>(3);

double yaw          = 0;
double desired_yaw  = 0;
double yaw_blender  = 0;
ros::Time t;
ros::Time pre_t;
int frame_num = 0;
int keyframe = 0;
int keyframe_stepsize;

struct uav_status{
    int frame_idx;
    colvec p;
    colvec v;
    colvec q;
    colvec volume_p;
    colvec volume_q;
};

uav_status keyframe_status;

// For position mode
vector<colvec> waypoints;                                                         // Waypoints [x, y, z, yaw] in world frame

nav_msgs::Path odom_traj;
nav_msgs::Path cmd_traj;

static string mesh_resource;
static string mesh_resource_volume;

colvec flyfield_q(4);
colvec flyfield_origin  = zeros<colvec>(3);

double integrete_pose_x = 0;
double integrete_pose_y = 0;
double integrete_pose_z = 0;

double odom_rate;

string output_dir;

void uav_position_velocity_visualization(colvec q, colvec ypr, colvec pose,
                                         colvec desired_pose, double desired_yaw, colvec vel, colvec p_v2d,colvec q_v2d);
void velocity_quant_rotate2rviz(colvec& ypr, colvec& q, colvec& v , double& yaw);
void position_cmd_rotate(colvec q, quadrotor_msgs::PositionCommand &position_cmd);
void pose_integration(double t_now, colvec& p, colvec& v);

unsigned int loc_time_sec, loc_time_nsec;

void loc_time_elapse_ms(double time_ms){
    loc_time_nsec += (int)(time_ms * 1000000);
}

ros::Time loc_time(void){
    if(loc_time_nsec>1000000000) {
        loc_time_sec += loc_time_nsec / 1000000000;
        loc_time_nsec = loc_time_nsec % 1000000000;
    }
    ros::Time loc(loc_time_sec, loc_time_nsec);
    return loc;
}


mat ypr_to_R(const colvec& ypr)
{
    double c, s;
    mat Rz = zeros<mat>(3,3);
    double y = ypr(0);
    c = cos(y);
    s = sin(y);
    Rz(0,0) =  c;
    Rz(1,0) =  s;
    Rz(0,1) = -s;
    Rz(1,1) =  c;
    Rz(2,2) =  1;

    mat Ry = zeros<mat>(3,3);
    double p = ypr(1);
    c = cos(p);
    s = sin(p);
    Ry(0,0) =  c;
    Ry(2,0) = -s;
    Ry(0,2) =  s;
    Ry(2,2) =  c;
    Ry(1,1) =  1;

    mat Rx = zeros<mat>(3,3);
    double r = ypr(2);
    c = cos(r);
    s = sin(r);
    Rx(1,1) =  c;
    Rx(2,1) =  s;
    Rx(1,2) = -s;
    Rx(2,2) =  c;
    Rx(0,0) =  1;

    mat R = Rz*Ry*Rx;
    return R;
}

mat yaw_to_R(double yaw)
{
    mat R = zeros<mat>(2,2);
    double c = cos(yaw);
    double s = sin(yaw);
    R(0,0) =  c;
    R(1,0) =  s;
    R(0,1) = -s;
    R(1,1) =  c;
    return R;
}

colvec R_to_ypr(const mat& R)
{
    colvec n = R.col(0);
    colvec o = R.col(1);
    colvec a = R.col(2);

    colvec ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0)*cos(y)+n(1)*sin(y));
    double r = atan2(a(0)*sin(y)-a(1)*cos(y), -o(0)*sin(y)+o(1)*cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
}

mat quaternion_to_R(const colvec& q)
{
    double n = norm(q, 2);
    colvec nq = q / n;

    double w = nq(0);
    double x = nq(1);
    double y = nq(2);
    double z = nq(3);
    double w2 = w*w;
    double x2 = x*x;
    double y2 = y*y;
    double z2 = z*z;
    double xy = x*y;
    double xz = x*z;
    double yz = y*z;
    double wx = w*x;
    double wy = w*y;
    double wz = w*z;

//    printf("cw_test1\n");
    mat R = zeros<mat>(3,3);
    R(0,0) = w2+x2-y2-z2;
    R(1,0) = 2*(wz + xy);
    R(2,0) = 2*(xz - wy);
    R(0,1) = 2*(xy - wz);
    R(1,1) = w2-x2+y2-z2;
    R(2,1) = 2*(wx + yz);
    R(0,2) = 2*(wy + xz);
    R(1,2) = 2*(yz - wx);
    R(2,2) = w2-x2-y2+z2;
    return R;
}

colvec R_to_quaternion(const mat& R)
{
    colvec q(4);
    double  tr = R(0,0) + R(1,1) + R(2,2);
    if (tr > 0)
    {
        double S = sqrt(tr + 1.0) * 2;
        q(0) = 0.25 * S;
        q(1) = (R(2,1) - R(1,2)) / S;
        q(2) = (R(0,2) - R(2,0)) / S;
        q(3) = (R(1,0) - R(0,1)) / S;
    }
    else if (R(0,0) > R(1,1) && R(0,0) > R(2,2))
    {
        double S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2;
        q(0) = (R(2,1) - R(1,2)) / S;
        q(1) = 0.25 * S;
        q(2) = (R(0,1) + R(1,0)) / S;
        q(3) = (R(0,2) + R(2,0)) / S;
    }
    else if (R(1,1) > R(2,2))
    {
        double S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2;
        q(0) = (R(0,2) - R(2,0)) / S;
        q(1) = (R(0,1) + R(1,0)) / S;
        q(2) = 0.25 * S;
        q(3) = (R(1,2) + R(2,1)) / S;
    }
    else
    {
        double S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2;
        q(0) = (R(1,0) - R(0,1)) / S;
        q(1) = (R(0,2) + R(2,0)) / S;
        q(2) = (R(1,2) + R(2,1)) / S;
        q(3) = 0.25 * S;
    }
    return q;
}

void velocity_quant_rotate2rviz(colvec& ypr, colvec& q, colvec& v , double& yaw)
{
    mat rotate_0(3,3,fill::eye);
    rotate_0(1,1) = -1;         rotate_0(2,2) = -1;
    q = R_to_quaternion( rotate_0 * inv(quaternion_to_R(flyfield_q)) * quaternion_to_R(q) );
    ypr = R_to_ypr(quaternion_to_R(q));

    mat rotate_1(3,3,fill::eye);
    mat rotate_2(3,3,fill::eye);
    rotate_1(1,1) = -1;         rotate_1(2,2) = -1;
    rotate_2(2,2) = -1;
    v = rotate_1 * inv(quaternion_to_R(flyfield_q)) * rotate_2 * v;

    geometry_msgs::Quaternion quat;
    quat.w = q(0);
    quat.x = q(1);
    quat.y = q(2);
    quat.z = q(3);
    yaw = tf::getYaw((const geometry_msgs::Quaternion)quat);
}

void pose_integration(double t_now, colvec& p, colvec& v)
{
    static double t_pre = 0;
    double t_incre = (t_pre == 0)? 0:(t_now - t_pre);
    integrete_pose_x += v(0) * t_incre;
    integrete_pose_y += v(1) * t_incre;
    integrete_pose_z += v(2) * t_incre;
    t_pre = t_now;
    p(0) = integrete_pose_x;
    p(1) = integrete_pose_y;
//    p(2) = new_pose_z;    // need more test
    p(2) = p(2);
}

void virtual_uav_trajectory()
{
    static quadrotor_msgs::PositionCommand pre_position_cmd;
    static colvec p_cmd = zeros<colvec>(3);
    static double yaw_cmd;

    /// pose integration
    double t_incre = (pre_position_cmd.header.stamp.toSec() == 0)?0:(position_cmd.header.stamp.toSec() - pre_position_cmd.header.stamp.toSec());
    colvec cmd_v(3);
//    t_incre = 0;
    cmd_v(0) = pre_position_cmd.velocity.x;         //body velcoity
    cmd_v(1) = pre_position_cmd.velocity.y;
    cmd_v(2) = pre_position_cmd.velocity.z;
    cmd_v = quaternion_to_R(q) * cmd_v;   //body to visualization
    p_cmd(0) = p(0) + cmd_v(0) * t_incre;
    p_cmd(1) = p(1) + cmd_v(1) * t_incre;
    p_cmd(2) = p(2) + cmd_v(2) * t_incre;

    geometry_msgs::Quaternion tmp_q = tf::createQuaternionMsgFromYaw(pre_position_cmd.yaw_dot * t_incre);
    colvec rotation_q(4);
    rotation_q(0) = tmp_q.w; rotation_q(1) = tmp_q.x; rotation_q(2) = tmp_q.y; rotation_q(3) = tmp_q.z;

//    printf("current yaw: %f, yaw increasement: %f\n", yaw, pre_position_cmd.yaw_dot * t_incre);

    /// get new odometry from integrate position command
    pre_position_cmd = position_cmd;
    p = p_cmd;
    v = cmd_v;
    q = R_to_quaternion(quaternion_to_R(q) * inv(quaternion_to_R(rotation_q)));     /// R_newbody2rviz = R_body2rviz * R_newbody2body
    ypr = R_to_ypr(quaternion_to_R(q));
    yaw = ypr(0);

    /// get new camera RT in blender
    mat r2bl = zeros<mat>(3,3);          /// rviz to blender
    r2bl(0,0) = -1; r2bl(1,1) = -1; r2bl(2,2) = 1;
    mat axis_flip = zeros<mat>(3,3);
    axis_flip(0,0) = -1; axis_flip(1,1) = 1; axis_flip(2,2) = -1;
    p_blender = r2bl * p;
    colvec ypr_blender = R_to_ypr(r2bl * quaternion_to_R(q) * axis_flip);           /// R_temp2blender = R_rviz2blender * R_body2rviz * R_temp2body
    yaw_blender = ypr_blender(0);
}

/* ************************* State feedback ************************* */
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(control_mode != POSITION_MODE)
        return;

    double depth = 1.0;
    loc_time_elapse_ms(1000.0/odom_rate);

    // Get time, position and velocity
    t    = loc_time();

    uav_position_velocity_visualization(q, ypr, p, desired_p, desired_yaw, v, volume_p, volume_q);

    static int cmd_delay;
    if (control_mode == POSITION_MODE) {
        colvec body_p = zeros<colvec>(3);
        colvec desired_body_p = zeros<colvec>(3);
        double body_yaw = 0;
        double desired_body_yaw = 0;
        desired_body_p = desired_p;
        desired_body_yaw = desired_yaw;

        static float nbv_buffer[TARGET_BUFFER_SIZE][4];
        static int nbv_buffer_ptr = 0;
        bool control_lost = true;
        for (int i = 0; i < 3; i++)
            nbv_buffer[nbv_buffer_ptr][i] = desired_p(i);
        nbv_buffer[nbv_buffer_ptr][3] = desired_yaw;
        for (int j = 0; j < TARGET_BUFFER_SIZE; j++){
            for(int k = 0; k < 4; k++){
                if (nbv_buffer[j][k] != nbv_buffer[nbv_buffer_ptr][k]){
                    control_lost = false;
                }
            }
        }
        nbv_buffer_ptr = (nbv_buffer_ptr + 1) % TARGET_BUFFER_SIZE;
        if(control_lost){
            desired_body_p = zeros<colvec>(3);
            desired_body_yaw = 0;
            printf("traj_gen_node.cpp [WARNING]: Control lost!\n");
        }

        pid_control_xyz(position_cmd, body_p, body_yaw, desired_body_p, desired_body_yaw, depth, t.toSec());

        position_cmd.header.stamp = t;

        // Rivz drone real trajectory visulization
        odom_traj.header.stamp = t;
        odom_traj.header.frame_id = string("/map");
        geometry_msgs::PoseStamped trajpoint;
        trajpoint.header = odom_traj.header;
        trajpoint.pose.position.x = p(0);
        trajpoint.pose.position.y = p(1);
        trajpoint.pose.position.z = p(2);
        trajpoint.pose.orientation = msg->pose.pose.orientation;
        odom_traj.poses.push_back(trajpoint);
        pubo.publish(odom_traj);

        virtual_uav_trajectory();

        quadrotor_msgs::PositionCommand blender_position_cmd;

        /// position and yaw to blender, rotation to master for volume drifting
        blender_position_cmd.position.x = p_blender(0);
        blender_position_cmd.position.y = p_blender(1);
        blender_position_cmd.position.z = p_blender(2);
        blender_position_cmd.yaw = yaw_blender;
//        blender_position_cmd.yaw_dot = yaw;

        pubc.publish(blender_position_cmd);
    }

}

void waypoints_callback2(const nav_msgs::Path::ConstPtr &msg)
{
    if (msg->poses.size() < 1) {
        ROS_ERROR("[Trajectory] path size = %lu", msg->poses.size());
        return;
    }
    static int count_xl = 0;
    static bool first_time_trigger = true;
    static colvec pose(3);
    if(first_time_trigger){
        pose(0) = p(0);
        pose(1) = p(1);
        pose(2) = p(2);
        first_time_trigger = false;
    }
    // Decode waypoints
    waypoints.clear();
    waypoints.resize(msg->poses.size(), zeros<colvec>(4));
    for (unsigned int k = 0; k < msg->poses.size(); k++) {
        waypoints[k](0) = msg->poses[k].pose.position.x;
        waypoints[k](1) = msg->poses[k].pose.position.y;
        waypoints[k](2) = msg->poses[k].pose.position.z;
        waypoints[k](3) = tf::getYaw(msg->poses[k].pose.orientation);
    }

    frame_num = msg->header.seq;

    desired_p(0) = waypoints[0](0); desired_p(1) = waypoints[0](1); desired_p(2) = 0;//flyfield_origin(2);
    desired_yaw = -waypoints[0](3); //waypoints[0](3);

    volume_p(0) = waypoints[1](0); volume_p(1) = waypoints[1](1); volume_p(2) = waypoints[1](2);
    volume_q(0) = msg->poses[1].pose.orientation.w; // volume 2 depth
    volume_q(1) = msg->poses[1].pose.orientation.x;
    volume_q(2) = msg->poses[1].pose.orientation.y;
    volume_q(3) = msg->poses[1].pose.orientation.z;


//    control_mode = POSITION_MODE;
    double depth = 1.0;
    loc_time_elapse_ms(1000.0/odom_rate);

    // Get time, position and velocity
    t    = loc_time();

    uav_position_velocity_visualization(q, ypr, p, desired_p, desired_yaw, v, volume_p, volume_q);

    static int cmd_delay;
    if (control_mode == POSITION_MODE) {
        colvec body_p = zeros<colvec>(3);
        colvec desired_body_p = zeros<colvec>(3);
        double body_yaw = 0;
        double desired_body_yaw = 0;
        desired_body_p = desired_p;
        desired_body_yaw = desired_yaw;

        static float nbv_buffer[TARGET_BUFFER_SIZE][4];
        static int nbv_buffer_ptr = 0;
        bool control_lost = true;
        for (int i = 0; i < 3; i++)
            nbv_buffer[nbv_buffer_ptr][i] = desired_p(i);
        nbv_buffer[nbv_buffer_ptr][3] = desired_yaw;
        for (int j = 0; j < TARGET_BUFFER_SIZE; j++){
            for(int k = 0; k < 4; k++){
                if (nbv_buffer[j][k] != nbv_buffer[nbv_buffer_ptr][k]){
                    control_lost = false;
                }
            }
        }
        nbv_buffer_ptr = (nbv_buffer_ptr + 1) % TARGET_BUFFER_SIZE;
        if(control_lost){
            desired_body_p = zeros<colvec>(3);
            desired_body_yaw = 0;
            printf("traj_gen_node.cpp [WARNING]: Control lost!\n");
        }

        pid_control_xyz(position_cmd, body_p, body_yaw, desired_body_p, desired_body_yaw, depth, t.toSec());

        position_cmd.header.stamp = t;

        // Rivz drone real trajectory visulization
        odom_traj.header.stamp = t;
        odom_traj.header.frame_id = string("/map");
        geometry_msgs::PoseStamped trajpoint;
        trajpoint.header = odom_traj.header;
        trajpoint.pose.position.x = p(0);
        trajpoint.pose.position.y = p(1);
        trajpoint.pose.position.z = p(2);
        odom_traj.poses.push_back(trajpoint);
        pubo.publish(odom_traj);

        virtual_uav_trajectory();

        quadrotor_msgs::PositionCommand blender_position_cmd;

        /// position and yaw to blender, rotation to master for volume drifting
        blender_position_cmd.position.x = p_blender(0);
        blender_position_cmd.position.y = p_blender(1);
        blender_position_cmd.position.z = p_blender(2);
        blender_position_cmd.yaw = yaw_blender;
//        blender_position_cmd.yaw_dot = yaw;

        pubc.publish(blender_position_cmd);
    }
    
    if(keyframe == frame_num){
        keyframe_status.frame_idx = keyframe;
        keyframe_status.p = p;
        keyframe_status.q = q;
        keyframe_status.v = v;
        keyframe_status.volume_p = volume_p;
        keyframe_status.volume_q = volume_q;
    }
}


void fly_field_callback(const geometry_msgs::Quaternion &flyfield_orientation)
{
    flyfield_q(0) = flyfield_orientation.w;
    flyfield_q(1) = flyfield_orientation.x;
    flyfield_q(2) = flyfield_orientation.y;
    flyfield_q(3) = flyfield_orientation.z;

    flyfield_origin(0) = p(0);
    flyfield_origin(1) = p(1);
    flyfield_origin(2) = p(2);

    integrete_pose_x = integrete_pose_y = 0;
}

void uav_position_velocity_visualization(colvec q, colvec ypr, colvec pose,
                                        colvec desired_pose, double desired_yaw, colvec vel, colvec p_v2d,colvec q_v2d)
{
    static visualization_msgs::Marker meshROS;
    static visualization_msgs::Marker meshROS2;
    static visualization_msgs::Marker meshVolume;
    static visualization_msgs::Marker arrowROS;
    static visualization_msgs::Marker arrowROS2;
    static visualization_msgs::Marker volume_x;
    static visualization_msgs::Marker volume_y;
    static visualization_msgs::Marker volume_z;
    // Mesh model
    meshROS.header.frame_id = string("/map");
    meshROS.header.stamp = t;
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = pose(0);
    meshROS.pose.position.y = pose(1);
    meshROS.pose.position.z = pose(2);

    q = R_to_quaternion(ypr_to_R(ypr));

    meshROS.pose.orientation.w = q(0);
    meshROS.pose.orientation.x = q(1);
    meshROS.pose.orientation.y = q(2);
    meshROS.pose.orientation.z = q(3);
    meshROS.scale.x = 1.0;
    meshROS.scale.y = 1.0;
    meshROS.scale.z = 1.0;
    meshROS.color.a = 0.5;
    meshROS.color.r = 1.0;
    meshROS.color.g = 1.0;
    meshROS.color.b = 1.0;
    meshROS.mesh_resource = mesh_resource;
    meshPub.publish(meshROS);

    colvec desired_pose_rviz(3);
    desired_pose_rviz = quaternion_to_R(q) * desired_pose + pose;

    colvec desired_ypr_body(3);
    desired_ypr_body(0) = desired_yaw;
    desired_ypr_body(1) = 0;
    desired_ypr_body(2) = 0;
    colvec desired_q(4);
    desired_q = R_to_quaternion(quaternion_to_R(q) * ypr_to_R(desired_ypr_body));

    // Mesh model
    meshROS2.header.frame_id = string("/map");
    meshROS2.header.stamp = t;
    meshROS2.ns = "mesh";
    meshROS2.id = 0;
    meshROS2.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS2.action = visualization_msgs::Marker::ADD;
    meshROS2.pose.position.x = desired_pose_rviz(0);
    meshROS2.pose.position.y = desired_pose_rviz(1);
    meshROS2.pose.position.z = desired_pose_rviz(2);

    q = R_to_quaternion(ypr_to_R(ypr));
    meshROS2.pose.orientation.w = desired_q(0);
    meshROS2.pose.orientation.x = desired_q(1);
    meshROS2.pose.orientation.y = desired_q(2);
    meshROS2.pose.orientation.z = desired_q(3);
    meshROS2.scale.x = 1.0;
    meshROS2.scale.y = 1.0;
    meshROS2.scale.z = 1.0;
    meshROS2.color.a = 0.5;
    meshROS2.color.r = 0.0;

    meshROS2.color.g = 1.0;
    meshROS2.color.b = 0.0;
    meshROS2.mesh_resource = mesh_resource;
    meshPub2.publish(meshROS2);
        

    colvec volume_pose_rviz(3);
    colvec volume_q_rviz(4);
    mat d2b(3,3,fill::zeros);
    d2b(0,2) = 1; d2b(1,0) = 1; d2b(2,1) = 1;
//    q_v2d(0) = 1; q_v2d(1) = q_v2d(2) = q_v2d(3) = 0;
    volume_q_rviz = R_to_quaternion(quaternion_to_R(q) * d2b * quaternion_to_R(q_v2d));
    volume_pose_rviz = quaternion_to_R(q) * d2b * p_v2d + pose;

    // Mesh model
    meshVolume.header.frame_id = string("/map");
    meshVolume.header.stamp = t;
    meshVolume.ns = "mesh";
    meshVolume.id = 0;
    meshVolume.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshVolume.action = visualization_msgs::Marker::ADD;
    meshVolume.pose.position.x = volume_pose_rviz(0);
    meshVolume.pose.position.y = volume_pose_rviz(1);
    meshVolume.pose.position.z = volume_pose_rviz(2);

//    q = R_to_quaternion(ypr_to_R(ypr));

    meshVolume.pose.orientation.w = volume_q_rviz(0);
    meshVolume.pose.orientation.x = volume_q_rviz(1);
    meshVolume.pose.orientation.y = volume_q_rviz(2);
    meshVolume.pose.orientation.z = volume_q_rviz(3);
    meshVolume.scale.x = 1.0;
    meshVolume.scale.y = 1.0;
    meshVolume.scale.z = 1.0;
    meshVolume.color.a = 0.2;
    meshVolume.color.r = 1.0;
    meshVolume.color.g = 0.5;
    meshVolume.color.b = 0.5;
    meshVolume.mesh_resource = mesh_resource_volume;
    meshPub3.publish(meshVolume);

    // Velocity
    colvec yprVel(3);
    yprVel(0) = atan2(vel(1), vel(0));
    yprVel(1) = -atan2(vel(2), norm(vel.rows(0,1),2));
    yprVel(2) = 0;
    q = R_to_quaternion(ypr_to_R(yprVel));
    arrowROS.header.frame_id = string("/map");
    arrowROS.header.stamp = t;
    arrowROS.ns = string("head");
    arrowROS.id = 0;
    arrowROS.type = visualization_msgs::Marker::ARROW;
    arrowROS.action = visualization_msgs::Marker::ADD;
    arrowROS.pose.position.x = pose(0);
    arrowROS.pose.position.y = pose(1);
    arrowROS.pose.position.z = pose(2);
    arrowROS.pose.orientation.w = q(0);
    arrowROS.pose.orientation.x = q(1);
    arrowROS.pose.orientation.y = q(2);
    arrowROS.pose.orientation.z = q(3);
    arrowROS.scale.x = norm(vel, 2);
    arrowROS.scale.y = 0.05;
    arrowROS.scale.z = 0.05;
    arrowROS.color.a = 1.0;
    arrowROS.color.r = 1.0;
    arrowROS.color.g = 0.0;
    arrowROS.color.b = 0.0;
    arrowPub.publish(arrowROS);

    arrowROS2.header.frame_id = string("/map");
    arrowROS2.header.stamp = t;
    arrowROS2.ns = string("head");
    arrowROS2.id = 0;
    arrowROS2.type = visualization_msgs::Marker::ARROW;
    arrowROS2.action = visualization_msgs::Marker::ADD;
    arrowROS2.pose.position.x = desired_pose_rviz(0);
    arrowROS2.pose.position.y = desired_pose_rviz(1);
    arrowROS2.pose.position.z = desired_pose_rviz(2);
    arrowROS2.pose.orientation.w = desired_q(0);
    arrowROS2.pose.orientation.x = desired_q(1);
    arrowROS2.pose.orientation.y = desired_q(2);
    arrowROS2.pose.orientation.z = desired_q(3);
    arrowROS2.scale.x = 0.3;
    arrowROS2.scale.y = 0.05;
    arrowROS2.scale.z = 0.05;
    arrowROS2.color.a = 1.0;
    arrowROS2.color.r = 0.0;
    arrowROS2.color.g = 1.0;
    arrowROS2.color.b = 0.0;
    arrowPub2.publish(arrowROS2);


    colvec x(3);
    x(0) = 1; x(1) = 0; x(2) = 0;
    x = quaternion_to_R(volume_q_rviz) * x;
    yprVel(0) = atan2(x(1), x(0));
    yprVel(1) = -atan2(x(2), norm(x.rows(0,1),2));
    yprVel(2) = 0;
    q = R_to_quaternion(ypr_to_R(yprVel));
    volume_x.header.frame_id = string("/map");
    volume_x.header.stamp = t;
    volume_x.ns = string("head");
    volume_x.id = 0;
    volume_x.type = visualization_msgs::Marker::ARROW;
    volume_x.action = visualization_msgs::Marker::ADD;
    volume_x.pose.position.x = volume_pose_rviz(0);
    volume_x.pose.position.y = volume_pose_rviz(1);
    volume_x.pose.position.z = volume_pose_rviz(2);
    volume_x.pose.orientation.w = q(0);
    volume_x.pose.orientation.x = q(1);
    volume_x.pose.orientation.y = q(2);
    volume_x.pose.orientation.z = q(3);
    volume_x.scale.x = 0.3;
    volume_x.scale.y = 0.05;
    volume_x.scale.z = 0.05;
    volume_x.color.a = 1.0;
    volume_x.color.r = 1.0;
    volume_x.color.g = 0.0;
    volume_x.color.b = 0.0;
    volumePubx.publish(volume_x);

    colvec y(3);
    y(0) = 0; y(1) = 1; y(2) = 0;
    y = quaternion_to_R(volume_q_rviz) * y;
    yprVel(0) = atan2(y(1), y(0));
    yprVel(1) = -atan2(y(2), norm(y.rows(0,1),2));
    yprVel(2) = 0;
    q = R_to_quaternion(ypr_to_R(yprVel));
    volume_y.header.frame_id = string("/map");
    volume_y.header.stamp = t;
    volume_y.ns = string("head");
    volume_y.id = 0;
    volume_y.type = visualization_msgs::Marker::ARROW;
    volume_y.action = visualization_msgs::Marker::ADD;
    volume_y.pose.position.x = volume_pose_rviz(0);
    volume_y.pose.position.y = volume_pose_rviz(1);
    volume_y.pose.position.z = volume_pose_rviz(2);
    volume_y.pose.orientation.w = q(0);
    volume_y.pose.orientation.x = q(1);
    volume_y.pose.orientation.y = q(2);
    volume_y.pose.orientation.z = q(3);
    volume_y.scale.x = 0.3;
    volume_y.scale.y = 0.05;
    volume_y.scale.z = 0.05;
    volume_y.color.a = 1.0;
    volume_y.color.r = 0.0;
    volume_y.color.g = 1.0;
    volume_y.color.b = 0.0;
    volumePuby.publish(volume_y);

    colvec z(3);
    z(0) = 0; z(1) = 0; z(2) = 1;
    z = quaternion_to_R(volume_q_rviz) * z;
    yprVel(0) = atan2(z(1), z(0));
    yprVel(1) = -atan2(z(2), norm(z.rows(0,1),2));
    yprVel(2) = 0;
    q = R_to_quaternion(ypr_to_R(yprVel));
    volume_z.header.frame_id = string("/map");
    volume_z.header.stamp = t;
    volume_z.ns = string("head");
    volume_z.id = 0;
    volume_z.type = visualization_msgs::Marker::ARROW;
    volume_z.action = visualization_msgs::Marker::ADD;
    volume_z.pose.position.x = volume_pose_rviz(0);
    volume_z.pose.position.y = volume_pose_rviz(1);
    volume_z.pose.position.z = volume_pose_rviz(2);
    volume_z.pose.orientation.w = q(0);
    volume_z.pose.orientation.x = q(1);
    volume_z.pose.orientation.y = q(2);
    volume_z.pose.orientation.z = q(3);
    volume_z.scale.x = 0.3;
    volume_z.scale.y = 0.05;
    volume_z.scale.z = 0.05;
    volume_z.color.a = 1.0;
    volume_z.color.r = 0.0;
    volume_z.color.g = 0.0;
    volume_z.color.b = 1.0;
    volumePubz.publish(volume_z);
}

void point_cloud_callback(const sensor_msgs::PointCloud::ConstPtr &pc)
{
    colvec volume_pose_rviz(3);
    colvec volume_q_rviz(4);

    mat d2b(3,3,fill::zeros);
    d2b(0,2) = 1; d2b(1,0) = 1; d2b(2,1) = 1;
    volume_q_rviz = R_to_quaternion(quaternion_to_R(q) * d2b * quaternion_to_R(volume_q));
    volume_pose_rviz = quaternion_to_R(q) * d2b * volume_p + p;

    sensor_msgs::PointCloud pc_rviz;
    pc_rviz.header.stamp = ros::Time::now();
    pc_rviz.header.frame_id = "/map";
    pc_rviz.points.resize(pc->points.size());
    for (int i = 0; i < pc->points.size(); i++) {
        colvec volume_coordinate(3);
        colvec rviz_coordinate(3);

        volume_coordinate(0) = pc->points[i].x;
        volume_coordinate(1) = pc->points[i].y;
        volume_coordinate(2) = pc->points[i].z;

        rviz_coordinate = quaternion_to_R(volume_q_rviz) * volume_coordinate + volume_pose_rviz;

        pc_rviz.points[i].x = rviz_coordinate(0);
        pc_rviz.points[i].y = rviz_coordinate(1);
        pc_rviz.points[i].z = rviz_coordinate(2);
    }
    pointcloudPub.publish(pc_rviz);
    if (keyframe == pc->header.seq){
        stringstream ss;
        ss.str(""); ss.width(6); ss.fill('0');
        ss << keyframe;
        string pc_file = output_dir + "/output/mesh/" + ss.str() + "_node.off";
        FILE* fout = fopen(pc_file.c_str(), "w");
        fprintf(fout, "OFF\n");
        fprintf(fout, "%d %d %d\n", (int)pc->points.size(), 0, 0);
        for(int i=0; i<pc->points.size(); i++) {
            fprintf(fout, "%f %f %f\n", pc_rviz.points[i].x, pc_rviz.points[i].y, pc_rviz.points[i].z);
        }
        fclose(fout);
    }
}

inline bool file_exist (const std::string& name)
{
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}


static void * mesh_conversion(void * arg)
{
    while(1) {
        stringstream ss;
        ss.str(""); ss.width(6);ss.fill('0');
        ss << keyframe;
        string object_file = output_dir + "/output/mesh/" + ss.str() + ".stl";
        string temp_file = output_dir + "/output/mesh/tmp" + ss.str() + ".off";
        string export_file = output_dir + "/output/mesh/" + ss.str() + ".off";
        string mat_file = output_dir + "/output/mesh/matrix_" + ss.str() + ".txt";
        int rc;
        if (file_exist(object_file)) {
            printf("~~~~~~~~~~~~~~~~~~~~~~~~~\n");
            string cmd = "meshlabserver -i " + object_file + " -o " + temp_file;
            rc = system(cmd.c_str());
            remove(object_file.c_str());
        }
        if (file_exist(temp_file) && (keyframe_status.frame_idx == keyframe)){
            printf("Converting keyframe mesh and flow to rviz world space...\n");

            FILE* fmat = fopen(mat_file.c_str(), "r");
            mat R_d2bl = zeros<mat>(3,3);                    /// Rotation matrix depth2blender
            colvec T_d2bl(3);                                /// Translation depth2blender
            for(int i=0; i<3; i++)
                rc = fscanf(fmat, "%lf, %lf, %lf, %lf\n", &R_d2bl(i,0), &R_d2bl(i,1), &R_d2bl(i,2), &T_d2bl(i));
            fclose(fmat);

#ifdef ALIGN_TO_VOLUME
            /// transform mesh from blender world space to live volume space
            mat R_bl2d = inv(R_d2bl);
            colvec T_bl2d = R_d2bl * (-T_d2bl);
            mat R_d2v = inv(quaternion_to_R(keyframe_status.volume_q));
            colvec T_d2v = R_d2v * (-keyframe_status.volume_p);

            mat R_d2d = zeros<mat>(3,3);                    /// Rotation matrix from blender depth camera to ordinary depth camera
            R_d2d(0,0) = 1; R_d2d(1,1) = -1; R_d2d(2,2) = -1;

            FILE* fin = fopen(temp_file.c_str(), "r");
            FILE* fout = fopen(export_file.c_str(), "w");
            rc = fscanf(fin, "OFF\n");
            fprintf(fout, "OFF\n");
            int vertices, faces, edges;
            rc = fscanf(fin, "%d %d %d\n", &vertices, &faces, &edges);
            fprintf(fout, "%d %d %d\n", vertices, 0, 0);
            for(int i=0; i<vertices; i++) {
                colvec point(3);
                rc = fscanf(fin, "%lf %lf %lf\n", &point(0), &point(1), &point(2));
                point = R_d2v * (R_d2d * (R_bl2d * point + T_bl2d)) + T_d2v;      /// t
                fprintf(fout, "%f %f %f\n", point(0), point(1), point(2));
            }


            string pc_file = output_dir + "/output/mesh/pc_" + ss.str() + ".off";
            FILE* fpc = fopen(pc_file.c_str(),"w");
            string img_file = output_dir + "/output/depth/Image" + ss.str() + ".png";
            cv::Mat depth = cv::imread(img_file, CV_LOAD_IMAGE_ANYDEPTH);
            fprintf(fpc, "OFF\n");
            int point_cntr;
            for(int i=0; i<320; i++) {
                for (int j = 0; j < 240; j++) {
                    if (depth.at<ushort>(j, i) > 0)
                        point_cntr++;
                }
            }
            fprintf(fpc, "%d %d %d\n", point_cntr, 0, 0);
            colvec point(3);
            float focal_len = 320 / 2 / tan(58.5 / 2 / 180 * 3.14159);
            for(int i=0; i<320; i++) {
                for (int j = 0; j < 240; j++) {
                    if (depth.at<ushort>(j, i) > 0) {
                        point(2) = (float) depth.at<ushort>(j, i) / 1000;
                        point(0) = (i - 320 / 2) / focal_len * point(2);
                        point(1) = (j - 240 / 2) / focal_len * point(2);
                        point = R_d2bl * R_d2d * point + T_d2bl;
                        fprintf(fpc, "%f %f %f\n", point(0), point(1), point(2));
                    }
                }
            }
            fclose(fpc);
#endif

#ifdef ALIGN_TO_RVIZ

            /// transform mesh from blender world space to rviz world space
            mat R_bl2d = inv(R_d2bl);
            colvec T_bl2d = R_d2bl * (-T_d2bl);
            mat R_b2r = quaternion_to_R(keyframe_status.q);
            colvec T_b2r = keyframe_status.p;

            mat R_d2b = zeros<mat>(3,3);                    /// Rotation matrix from blender depth camera to body
            R_d2b(0,2) = -1; R_d2b(1,0) = 1; R_d2b(2,1) = -1;

            FILE* fin = fopen(temp_file.c_str(), "r");
            FILE* fout = fopen(export_file.c_str(), "w");
            rc = fscanf(fin, "OFF\n");
            fprintf(fout, "OFF\n");
            int vertices, faces, edges;
            rc = fscanf(fin, "%d %d %d\n", &vertices, &faces, &edges);
            fprintf(fout, "%d %d %d\n", vertices, faces, 0);
            for(int i=0; i<vertices; i++) {
                colvec point(3);
                rc = fscanf(fin, "%lf %lf %lf\n", &point(0), &point(1), &point(2));
                point = R_b2r * (R_d2b * (R_bl2d * point + T_bl2d)) + T_b2r;      /// t
                rc = fprintf(fout, "%f %f %f\n", point(0), point(1), point(2));
            }
            int face[4];
            for(int j=0; j<faces; j++){
                rc = fscanf(fin, "%d %d %d %d\n", &face[0], &face[1], &face[2], &face[3]);
                rc = fprintf(fout, "%d %d %d %d\n", face[0], face[1], face[2], face[3]);
            }

            /// Record transform matrix from rviz to ordinary depth camera
            string keymat_file = output_dir + "/output/keyframe_Tr2d.txt";
            FILE* fkeymat = fopen(keymat_file.c_str(), "a");
            mat R_b2d = zeros<mat>(3,3);
            R_b2d(0,1) = 1; R_b2d(1,2) = 1; R_b2d(2,0) = 1;
            mat R_r2d = R_b2d * inv(quaternion_to_R(keyframe_status.q));             /// R_rviz2detph = R_body2depth * inv(R_body2rviz)
            colvec T_r2d = R_b2d * inv(quaternion_to_R(keyframe_status.q)) * (-keyframe_status.p);

            fprintf(fkeymat, "%d\n", keyframe);
            for(int i=0; i<3; i++){
                fprintf(fkeymat, "%f\t%f\t%f\t%f\n",R_r2d(i,0), R_r2d(i,1), R_r2d(i,2), T_r2d(i));
            }
            fprintf(fkeymat, "%f\t%f\t%f\t%f\n", 0.0, 0.0, 0.0, 1.0);
            fclose(fkeymat);
#endif
            fclose(fin);
            fclose(fout);
            rc = remove(temp_file.c_str());
            rc = remove(mat_file.c_str());
            keyframe = keyframe + keyframe_stepsize;
        }
    }
}

int start_mesh_conv_thread(void)
{
    int ret;
    pthread_t a;
    ret = pthread_create(&a, 0, mesh_conversion, NULL);
    if(ret != 0)
    {
        return -1;
    }
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle n("~");
    
    std::string quadrotor_name;
    n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
                                                     position_cmd.header.frame_id = "/" + quadrotor_name;
    n.param("mesh_resource", mesh_resource, std::string("package://trajectory_generator/meshes/hummingbird.mesh"));
    n.param("volume_resource", mesh_resource_volume, std::string("package://trajectory_generator/meshes/cube.stl"));

    n.param("init_x", p(0), 0.0);
    n.param("init_y", p(1), 0.0);
    n.param("init_z", p(2), 0.0);
    n.param("init_yaw", yaw, 0.0);
    n.param("odom_rate", odom_rate, 20.0);
    n.param("keyframe_stepsize", keyframe_stepsize, 100);
    n.param("output_dir", output_dir, string("/home/chengwei/Projects/Simulator/result"));

//    printf("##########################################\n");

//    ros::Subscriber sub1    = n.subscribe("DroneOdometry",             10, odom_callback);
    ros::Subscriber sub2    = n.subscribe("Waypoints",    10, waypoints_callback2);
//    ros::Subscriber sub3    = n.subscribe("FlyfieldOrientation", 10, fly_field_callback);
    ros::Subscriber sub4    = n.subscribe("NodePointCloud", 10, point_cloud_callback);
    pubc                    = n.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 10);
    pubo                    = n.advertise<nav_msgs::Path>            ("odom_traj",   1,   true);
    pubct                   = n.advertise<nav_msgs::Path>            ("cmd_traj",    1,   true);
    meshPub                 = n.advertise<visualization_msgs::Marker>("robot",       100, true);
    meshPub2                = n.advertise<visualization_msgs::Marker>("target",      100, true);
    meshPub3                = n.advertise<visualization_msgs::Marker>("volume",      100, true);
    arrowPub                = n.advertise<visualization_msgs::Marker>("arrow",       100, true);
    arrowPub2               = n.advertise<visualization_msgs::Marker>("target_arrow",100, true);
    volumePubx              = n.advertise<visualization_msgs::Marker>("volume_axisx",100, true);
    volumePuby              = n.advertise<visualization_msgs::Marker>("volume_axisy",100, true);
    volumePubz              = n.advertise<visualization_msgs::Marker>("volume_axisz",100, true);
    pointcloudPub           = n.advertise<sensor_msgs::PointCloud>   ("point_cloud", 10,  true);

    start_mesh_conv_thread();

    loc_time_sec = loc_time_nsec = 0;

    v(0) = v(1) = v(2) = 0;

    mat bl2r = zeros<mat>(3,3);             /// blender to rviz
    bl2r(0,0) = -1; bl2r(1,1) = -1; bl2r(2,2) = 1;
    p = bl2r * p;

    ypr(0) = yaw;
    mat axis_flip = zeros<mat>(3,3);
    axis_flip(0,0) = -1; axis_flip(1,1) = 1; axis_flip(2,2) = -1;
    mat b2bl = ypr_to_R(ypr) * axis_flip;   /// body to blender
    q = R_to_quaternion(bl2r * b2bl);       /// body to rviz
    ypr = R_to_ypr(quaternion_to_R(q));     /// renew ypr
    yaw = ypr(0);

    keyframe_status.frame_idx = 100;

    ros::spin();
    
    return 0;
}
