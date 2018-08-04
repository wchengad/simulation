#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

nav_msgs::Path point()
{
    // Circle parameters
    double x0 = 0;
    double y0 = -1;
    double z0 = 0.17;
    double h = 2;
    double r = 2;
    // Init msg
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    pt.pose.position.x = x0;
    pt.pose.position.y = y0;
    pt.pose.position.z = z0+h;
    waypoints.poses.push_back(pt);      
    pt.pose.position.x = x0;
    pt.pose.position.y = y0+r;
    pt.pose.position.z = z0+h;
    waypoints.poses.push_back(pt);      

    // Return
    return waypoints;
}

// Circle trajectory
nav_msgs::Path circle()
{
    // Circle parameters
    double x0 = 0;
    double y0 = 0;
    double z0 = 0;
    double r = 0.707;
    double h = 1.0;
    // double l = 1.0;
    // Init msg
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
 
    // Start point    
    pt.pose.position.x =  x0;
    pt.pose.position.y =  y0;
    pt.pose.position.z =  z0+h;
    //pt.pose.orientation = tf::createQuaternionMsgFromYaw(90*M_PI/180);      
    waypoints.poses.push_back(pt);      
    
    for(int k=0; k<2; ++k) {
        pt.pose.position.x =  r + x0;
        pt.pose.position.y = -r + y0;
        pt.pose.position.z =  h + z0;
        //pt.pose.orientation = tf::createQuaternionMsgFromYaw(90*M_PI/180);      
        waypoints.poses.push_back(pt);      
        
        pt.pose.position.x = -r + x0;
        pt.pose.position.y = -r + y0;
        pt.pose.position.z =  h + z0;
        //pt.pose.orientation = tf::createQuaternionMsgFromYaw(90*M_PI/180);      
        waypoints.poses.push_back(pt);      
 
        pt.pose.position.x = -r + x0;
        pt.pose.position.y =  r + y0;
        pt.pose.position.z =  h + z0;
        //pt.pose.orientation = tf::createQuaternionMsgFromYaw(90*M_PI/180);      
        waypoints.poses.push_back(pt);      
 

        pt.pose.position.x =  r + x0;
        pt.pose.position.y =  r + y0;
        pt.pose.position.z =  h + z0;
        //pt.pose.orientation = tf::createQuaternionMsgFromYaw(90*M_PI/180);      
        waypoints.poses.push_back(pt);      
    }
    
    // End point
    pt.pose.position.x = x0;
    pt.pose.position.y = y0;
    pt.pose.position.z = z0 + h;
    //pt.pose.orientation = tf::createQuaternionMsgFromYaw(0*M_PI/180);              
    waypoints.poses.push_back(pt);     

    // Return
    return waypoints;
}

// Figure 8 trajectory
nav_msgs::Path eight()
{
    // Circle parameters
    double offset_x = 1.0;
    double offset_y = 0.0;
    double r = 0.9;
    double h = 1.0;
    // double l = 1.0;
    // Init msg
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);    
    
    pt.pose.position.x =  0;
    pt.pose.position.y =  0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    for(int i=0; i< 3; ++i)
    {
        // First loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);
        // Second loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0 + offset_x;
        pt.pose.position.y =  0 + offset_y;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);
        
    }
    // return to 0,0
    pt.pose.position.x =  0;
    pt.pose.position.y =  0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);    

    // Return
    return waypoints;   
}  

#endif
