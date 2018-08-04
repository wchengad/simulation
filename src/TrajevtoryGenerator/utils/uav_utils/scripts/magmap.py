#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf import transformations as tfs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField

odom = None
mag = None
data = []


def save_data():
    with open('/tmp/mag.txt','w') as f:
        s = None
        for rec in data:
            f.write('\t'.join(['%.3f'%(x) for x in rec]))
            f.write('\n')
        f.close()

def mag_callback(mag_msg):
    global mag
    mag = [mag_msg.magnetic_field.x,
           mag_msg.magnetic_field.y,
           mag_msg.magnetic_field.z]
    global data
    data.append(odom + mag)

def odom_callback(odom_msg):
    global odom
    odom = [odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
            ]


if __name__ == "__main__":
    rospy.init_node("magmap")

    odom_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
    mag_sub = rospy.Subscriber("/djiros/mag", MagneticField, mag_callback)
    rospy.on_shutdown(save_data)
    rospy.spin()
