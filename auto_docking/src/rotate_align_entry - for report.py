#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import Int32
from std_msgs.msg import Float64, String
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import math

#cmd_vel unit: m/s, rad/s

if __name__ == '__main__':
    rospy.init_node('rotate_and_dock')

    #Initialising variables
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    cmd_pub = rospy.Publisher('/sdr/navigation/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    cmd = geometry_msgs.msg.Twist()
    x_pub = rospy.Publisher('/x_value', Float64, queue_size=10)
    y_pub = rospy.Publisher('/y_value', Float64, queue_size=10)
    status_pub = rospy.Publisher('/status', String, queue_size=10)
    operation = 'rotate'
    linear_vel = 0.0
    prev_linear_vel = 0.0
    angular_vel = 0.0
    prev_angular_vel = 0.0
    prev_x_dist=0.0
    prev_y_dist = 0.0
    x_error = 0.0
    prev_x_error = 0.0
    x_error_integral = 0.0
    y_error = 0.0
    prev_y_error = 0.0
    y_error_integral = 0.0
    itr_entry=0
    itr_align=0

    while not rospy.is_shutdown():
        
        if(operation == "rotate"):
            trans = []
            print("operation:" + operation)
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("artag_not_detected")
                trans = [1000000000]

            print("trans[0]:" + str(trans[0]))

            if(trans[0]<0):
                cmd.angular.z = 0.2
            elif(trans[0]>0):
                cmd.angular.z = -0.2

            if(-0.15<trans[0] and trans[0]<0.15):
                cmd.angular.z = 0
                cmd_pub.publish(cmd)
                for i in range(2): #sleep for 0.2 seconds
                    rate.sleep()
                operation = 'entry'
            cmd_pub.publish(cmd)

        elif(operation == "entry"):
            print("operation:" + operation)
            itr_entry+=1
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
            x_dist = -(trans[2] - 0.20)
            y_dist = -(trans[0])

            print("                             " + "\n" + "\n")
            print("iteration: " + str(itr_entry))
            print("x-distance:" + str(trans[2] - 0.20))
            print("y-distance:" + str(trans[0]))
            
            ##PI initialisation:
            K_P_x = 0.2
            K_P_y = 1.0

            K_I_x = 0.001
            K_I_y = 0.01

            x_error = x_dist - 0
            y_error = y_dist - 0

            print("x_error:" + str(x_error))
            print("y_error:" + str(y_error))

            x_error_integral += x_error*0.1
            y_error_integral += y_error*0.1

            print("x_error_integral: " + str(x_error_integral))
            print("y_error_integral: " + str(y_error_integral))

            #PI-algo
            linear_vel = K_P_x*x_error  + K_I_x*x_error_integral
            angular_vel = K_P_y*y_error + K_I_y*y_error_integral

            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            cmd_pub.publish(cmd)

            print("cmd: " + str(cmd))

            prev_x_dist = x_dist
            prev_y_dist = y_dist 
            prev_x_error = x_error
            prev_y_error = y_error

            #for collecting data
            x = Float64()
            y = Float64()
            x = x_dist
            y = y_dist
            x_pub.publish(x)
            y_pub.publish(y)
            
            #end of docking
            if(abs(x_error)<0.035):
                print("auto-docking operation complete")
                break

        rate.sleep()    