#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf import transformations as tfs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose, Twist

pub = None

class Pos_t(np.ndarray):
    def __new__(cls, *args):
        obj = np.zeros((3,))
        successfully_inited = False
        if len(args)==0:
            self.fill(0)
        elif len(args)==1:
            if isinstance(args[0], Point):
                obj[0]=args[0].x
                obj[1]=args[0].y
                obj[2]=args[0].z
                successfully_inited = True
        elif len(args)==3:
            if all([isinstance(x,float) for x in args]):
                obj[0]=args[0]
                obj[1]=args[1]
                obj[2]=args[2]
                successfully_inited = True
        if not successfully_inited:
            raise AttributeError("Invalid constructor argument <%s>"%args)
        return obj

if __name__ == "__main__":
    rospy.init_node("odom_to_euler")
    pm1 = Point(1.0,2.0,3.0)
    pm2 = Point(2.0,3.0,4.0)

    p1 = Pos_t(pm1)
    p2 = Pos_t(pm2)
    print (p1.dot(p2))