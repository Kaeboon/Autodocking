#!/usr/bin/env python  
import roslib
import rospy
import roslaunch
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from std_msgs.msg import Int32
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('rotate_till_see_tag')

    listener = tf.TransformListener()


    cmd_pub = rospy.Publisher('SDR_simulation/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    cmd = geometry_msgs.msg.Twist()

    cmd.angular.z = 0.5

    rate = rospy.Rate(10.0) #rate = 10Hz
    while not rospy.is_shutdown():
        
        cmd_pub.publish(cmd)

        try:
            (trans,rot) = listener.lookupTransform('/front_camera_color_frame', '/ar_marker_255',rospy.Time(0))
            #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
            #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if(-0.3<trans[1] and trans[1]<0.3):
            cmd.angular.z = 0         
            
        rate.sleep()