#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

//opencv
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "../../../devel/include/quadrotor_msgs/PositionCommand.h"
//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>

#include <quadrotor_msgs/PositionCommand.h>
#include <Eigen/Geometry>
#include <tf/tf.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include "network.h"
#include "common_utility.h"
#include <fstream>

#define DEPTH_RECT

using namespace std;
using namespace cv;

ros::Publisher waypoints_publisher;
ros::Publisher node_point_cloud_publisher;
ros::Publisher drone_odometry_publisher;


void TimerCallback(const ros::TimerEvent&);

int image_width;
int image_height;
double odom_rate;

double pcmd_des_pos[3];
double pcmd_des_vel[3];
double pcmd_des_q[4];
double pcmd_des_yaw;

string traj_file;
string output_dir;
string master_ip;
int fetch_frame_num = 0;

Mat cvDepthImg;
Mat cvRawImg16U;
//Mat cvExrImg;

// copy in binary mode
bool copyFile(const char *SRC, const char* DEST){
    std::ifstream src(SRC, std::ios::binary);
    std::ofstream dest(DEST, std::ios::binary);
    dest << src.rdbuf();
    return src && dest;
}

inline bool file_exist (const std::string& name){
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

void remove_dir(const char *path){
    struct dirent *entry = NULL;
    DIR *dir = NULL;
    dir = opendir(path);
    while(entry = readdir(dir))
    {
        DIR *sub_dir = NULL;
        FILE *file = NULL;
        char abs_path[100] = {0};
        if(*(entry->d_name) != '.')
        {
            sprintf(abs_path, "%s/%s", path, entry->d_name);
            if(sub_dir = opendir(abs_path))
            {
                closedir(sub_dir);
                remove_dir(abs_path);
            }
            else
            {
                if(file = fopen(abs_path, "r"))
                {
                    fclose(file);
                    remove(abs_path);
                }
            }
        }
    }
    remove(path);
}

#define PI 3.1415926
static void sig_handle(int h){
    pthread_mutex_t status_lock_s = PTHREAD_MUTEX_INITIALIZER;
    // cvDestroyWindow("Depth View");
    printf("releasing resource....\n");
    if(file_exist(traj_file)) {
        char dist[256];
        time_t cur_time = time(NULL);
        struct tm now = *localtime(&cur_time);
        sprintf(dist, "%s/trajectory_20%02d-%02d-%02d %02d:%02d:%02d.txt", output_dir.c_str(),
                now.tm_year % 100, now.tm_mon + 1, now.tm_mday, now.tm_hour, now.tm_min, now.tm_sec);
        copyFile(traj_file.c_str(), dist);
        remove(traj_file.c_str());
    }
    pthread_mutex_unlock(&status_lock_s);
    exit(0);
}


void TrajPositionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd){
    static int local_cntr;
    pcmd_des_pos[0] = cmd->position.x;
    pcmd_des_pos[1] = cmd->position.y;
    pcmd_des_pos[2] = cmd->position.z;

    pcmd_des_vel[0] = cmd->velocity.x;
    pcmd_des_vel[1] = cmd->velocity.y;
    pcmd_des_vel[2] = cmd->velocity.z;

    pcmd_des_yaw = cmd->yaw;

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pcmd_des_yaw);
    pcmd_des_q[0] = q.w;
    pcmd_des_q[1] = q.x;
    pcmd_des_q[2] = q.y;
    pcmd_des_q[3] = q.z;

    /// writie next view spot to renderer
    FILE* fid;
    if(fetch_frame_num > 1) {
        fid = fopen(traj_file.c_str(), "a");
        fseek(fid, 0, SEEK_END);
    }
    else
        fid = fopen(traj_file.c_str(),"w");
    fprintf(fid, "%f\t%f\t%f\t%f\n", pcmd_des_pos[0], pcmd_des_pos[1], pcmd_des_pos[2], pcmd_des_yaw);
    fclose(fid);
    local_cntr++;
//    printf("local cntr = %d, fetch_frame_num = %d\n", local_cntr, fetch_frame_num);
}



static void * ZmqRecvFunction(void * arg){
    nav_msgs::Path waypoints;
    static bool sent_flag = false;
    static int cnt = 0;
    while(1){
        int rc = network_udp_msg_receive();
        if(rc == 0)
            continue;
        waypoints.header.frame_id = std::string("world");
        waypoints.header.stamp = ros::Time::now();
        waypoints.header.seq = fetch_frame_num;
        waypoints.poses.clear();

        geometry_msgs::PoseStamped nbv;
        nbv.pose.position.x = Rxbuffer.nbv[0];
        nbv.pose.position.y = Rxbuffer.nbv[1];
        nbv.pose.position.z = Rxbuffer.nbv[2];
        nbv.pose.orientation = tf::createQuaternionMsgFromYaw(Rxbuffer.nbv[3]);
        waypoints.poses.push_back(nbv);
        float curr_depth = Rxbuffer.nbv[4];

        printf("main.cpp [waypoint debug]: curr_depth: %f, waypoint[i].x: %f, waypoint[i].y: %f,"
                       " waypoint[i].z: %f, waypoint[i].yaw: %f\n",
               curr_depth, Rxbuffer.nbv[0], Rxbuffer.nbv[1], Rxbuffer.nbv[2], Rxbuffer.nbv[3]);

        geometry_msgs::PoseStamped volume;
        float q[4];
        r2q(Rxbuffer.Tv2d, q);
        // printf("received = %f, %f, %f, %f\n", q[0], q[1], q[2], q[3]);
        volume.pose.position.x = Rxbuffer.Tv2d[9];
        volume.pose.position.y = Rxbuffer.Tv2d[10];
        volume.pose.position.z = Rxbuffer.Tv2d[11];
        volume.pose.orientation.w = q[0];
        volume.pose.orientation.x = q[1];
        volume.pose.orientation.y = q[2];
        volume.pose.orientation.z = q[3];
        waypoints.poses.push_back(volume);


        waypoints_publisher.publish(waypoints);

        sensor_msgs::PointCloud pc;
        pc.header.stamp = ros::Time::now();
        pc.header.frame_id = "world";
        pc.header.seq = fetch_frame_num;
        pc.points.resize(Rxbuffer.node_num);
        for(int i = 0; i < Rxbuffer.node_num; i++){
            pc.points[i].x = Rxbuffer.node_pc[i][0];
            pc.points[i].y = Rxbuffer.node_pc[i][1];
            pc.points[i].z = Rxbuffer.node_pc[i][2];
        }

        node_point_cloud_publisher.publish(pc);
    }
}


int StartZmqRecvThread(void){
    int ret;
    pthread_t a;
    ret = pthread_create(&a, 0, ZmqRecvFunction, NULL);
    if(ret != 0)
    {
        return -1;
    }
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc , argv, "virtual_uav");
    ros::NodeHandle n;

    n.param("/virtual_uav_node/image_width", image_width, 320);
    n.param("/virtual_uav_node/image_height", image_height, 240);
    n.param("/virtual_uav_node/odom_rate", odom_rate, 20.0);
    n.param("/virtual_uav_node/output_dir", output_dir, string("/home/chengwei/Projects/Simulator/result"));
    n.param("/virtual_uav_node/master_ip", master_ip, string("10.8.5.254"));

    printf("odom_rate = %f\n", odom_rate);

    traj_file = output_dir + "/trajectory.txt.active";

    waypoints_publisher = n.advertise<nav_msgs::Path>("Waypoints", 1);
    node_point_cloud_publisher = n.advertise<sensor_msgs::PointCloud>("NodePointCloud", 1);
    ros::Subscriber position_cmd_sub_ = n.subscribe("position_cmd", 10, TrajPositionCmdCallback);

    signal(SIGINT, sig_handle);

//    printf("removing previous directory\n");
//    remove_dir((output_dir+"/out").c_str());

    StartZmqRecvThread();

    network_depth_log_initial(master_ip);

    namedWindow("Depth View");
    cvRawImg16U = Mat(image_height, image_width, CV_16UC1);
    cvDepthImg = Mat(image_height, image_width, CV_8UC1);

    ros::Timer timer = n.createTimer(ros::Duration(1/odom_rate), TimerCallback);
    ros::spin();
    return true;
}



int frame_num = 0;
int drone_action_flag = 0;

void ExrToPng(void)
{
    stringstream ss;
    ss.str("");ss.width(4);ss.fill('0');
    ss << fetch_frame_num;
    string file_name = output_dir + "/output/depth/Image" + ss.str() + ".exr";
    ss.str("");ss.width(6);ss.fill('0');
    ss << fetch_frame_num;
    string file_name_out = output_dir + "/output/depth/Image" + ss.str() + ".png";
    string file_name_view = output_dir + "/output/depth/view/Image" + ss.str() + ".png";
    if (file_exist(file_name)) {
        usleep(10000);
        Mat cvExrImg = imread(file_name, IMREAD_UNCHANGED);
        float focal_len = image_width / 2 / tan(58.5 / 2 / 180 * PI);
        float x, y, d;
        for (int i = 0; i < image_width; i++)
            for (int j = 0; j < image_height; j++) {
                d = cvExrImg.at<Vec3f>(j, i)[0];
                x = d / focal_len * (i - image_width / 2);
                y = d / focal_len * (j - image_height / 2);
                cvRawImg16U.at<ushort>(j, i) = (ushort)(sqrt(d * d - x * x - y * y) * 1000);
                cvDepthImg.at<uchar>(j, i) = (cvRawImg16U.at<ushort>(j, i) > 2500) ? 0 : (cvRawImg16U.at<ushort>(j, i) *
                                                                                          255 / 2500.0);
            }
        imwrite(file_name_out.c_str(), cvRawImg16U);
        imwrite(file_name_view.c_str(), cvDepthImg);
        imshow("Depth View", cvDepthImg);
        remove(file_name.c_str());
    }
}



bool SimulatorRead(void)
{
//    static int fetch_frame_num = 0;
    static bool started = false;

    if(file_exist(traj_file)){
        if(!started) {
            printf("main.cpp [simulator debug]: simulator started!\n");
            fetch_frame_num = 1;
        }
        started = true;
    }

#ifdef DEPTH_RECT
    ExrToPng();
#endif

    stringstream ss;
    ss.str("");ss.width(6);ss.fill('0');
    ss << fetch_frame_num;
    string file_name = output_dir + "/output/depth/Image" + ss.str() + ".png";
    if (file_exist(file_name)){
        printf("Virtual UAV fetching : %d\n",fetch_frame_num);
        cvRawImg16U = imread(file_name, CV_LOAD_IMAGE_ANYDEPTH);
        if(started)
            fetch_frame_num++;
        return true;
    }
    return false;
}

void TimerCallback(const ros::TimerEvent&)
{
    if(SimulatorRead()) {
        memcpy(Txbuffer.depth, (char *)cvRawImg16U.data, image_height * image_width * 2);

        frame_num++;
        network_make_buffer(frame_num, pcmd_des_pos, pcmd_des_q, drone_action_flag);
        network_depth_log_send();
//        imshow("RGB", cvBGRImg);
    }
    cv::waitKey(2);
}


