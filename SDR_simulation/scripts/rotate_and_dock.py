#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import Int32
from std_msgs.msg import Float64, String

#cmd_vel unit: m/s, rad/s

if __name__ == '__main__':
    rospy.init_node('rotate_and_dock')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0) #rate = 10Hz
    cmd_pub = rospy.Publisher('SDR_simulation/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    cmd = geometry_msgs.msg.Twist()

    operation = 'rotating'
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

    itr=0

    x_pub = rospy.Publisher('/x_value', Float64, queue_size=10)
    y_pub = rospy.Publisher('/y_value', Float64, queue_size=10)
    #--------------------

    #for documentation puspose
    status_pub = rospy.Publisher('/status', String, queue_size=10)

    while not rospy.is_shutdown():

        if(operation == "rotating"):
            try:
                (trans,rot) = listener.lookupTransform('/back_camera_color_optical_frame', '/ar_marker_255',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            #status_pub.publish(operation)
            cmd_pub.publish(cmd)

            if(-0.3<trans[0] and trans[0]<0.3):
                cmd.angular.z = 0
                operation = 'docking' 
                status_pub.publish("changes_operation_to_docking")

            if(trans[0]<-0.3):
                cmd.angular.z = -0.5
            elif(trans[0]>0.3):
                cmd.angular.z = +0.5

        

#start of docking---  

        else:
            itr+=1
            

            try:
                (trans,rot) = listener.lookupTransform('/back_camera_color_optical_frame', '/ar_marker_255',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            status_pub.publish(operation)
        
            x_dist = -(trans[2] - 0.20)
            y_dist = -(trans[0])


#trans[0] is the y-value for the actual robot, trans[1] y-value for this simulation
#trans[2] is the x value for the actual robot, trans[0] x-value for this simulation


            print("                             " + "\n" + "\n")
            print("iteration " + str(itr))
            print("x-distance :" + str(trans[0] - 0.20))
            print("y-distance :" + str(trans[1]))
            

        ##PI initialisation:
            K_P_x = 0.5
            K_P_y = 0.5
            K_I_x = 0.001
            K_I_y = 0.001

            x_error = x_dist - 0
            y_error = y_dist - 0

            print("x_error:" + str(x_error))

        ##P control
        #linear_vel = P_x*x_error + prev_linear_vel
        #angular_vel = P_y*y_error + prev_angular_vel

        ##PI control
            #x_error_integral = (x_error-prev_x_error)*0.1
            #y_error_integral = (y_error-prev_y_error)*0.1

            x_error_integral += x_error*0.1
            y_error_integral += y_error*0.1

            print("x_error_integral: " + str(x_error_integral))

            linear_vel = K_P_x*x_error  + K_I_x*x_error_integral
            angular_vel = K_P_y*y_error + K_I_y*y_error_integral
        
            print("linear_vel: " + str(linear_vel))

        ################# for moving the robot based on the linea_vel or the angular_vel
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            cmd_pub.publish(cmd)


            print("cmd: " + str(cmd))

            prev_x_dist = x_dist
            prev_y_dist = y_dist 
            prev_x_error = x_error
            prev_y_error = y_error

        ################################(below code is for my own documentation purpose, can remove)

            x = Float64()
            y = Float64()
            x = x_dist + 0.25
            y = y_dist
            x_pub.publish(x)
            y_pub.publish(y)
            
            if(abs(x_error)<0.02):
                print("auto-docking operation complete")
                break


#---end of docking

        rate.sleep()
