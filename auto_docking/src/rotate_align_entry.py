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
    rate = rospy.Rate(10.0) #rate = 10Hz
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
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
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
                for i in range(2):
                    rate.sleep()
                if(abs(euler_from_quaternion(rot)[1])<0.3):
                    operation = 'entry'
                    print("changes_operation_to_entry")
                else:
                    if(euler_from_quaternion(rot)[1]<0):
                        operation = 'entry'
                        print("changes_operation_to_cal_L")
                    else:
                        operation ="entry"
                        print("changes_operation_to_cal_R")
            cmd_pub.publish(cmd)

### operation of robot docking from the left
        elif(operation == "calibration_L"):
            for i in range(4):
                rate.sleep()

            trans = []
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                trans = [1000000000]

            print("calibrating_left")
            calibration_left = 1
            left_angle_0 = (euler_from_quaternion(rot))[1]
            time_left_0 = time.time()
            while(calibration_left):
                cmd.angular.z = 0.2
                try:
                    (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

                if(trans[0]>0.4):
                    calibration_left = 0
                cmd_pub.publish(cmd)
                rate.sleep()

            time_left_1 = time.time()
            left_angle_1 = (euler_from_quaternion(rot))[1]
            
            cmd.angular.z = 0
            cmd_pub.publish(cmd)

            print("left_data_collected")

            for i in range(5):
                rate.sleep()

            trans = []
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                trans = [1000000000]  

            print("calibrating_right")
            calibration_right = 1
            right_angle_0 = (euler_from_quaternion(rot))[1]
            time_right_0 = time.time()

            while(calibration_right):
                cmd.angular.z = -0.2
                try:
                    (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

                if(trans[0]<0.05):
                    calibration_right = 0
                cmd_pub.publish(cmd)
                rate.sleep()
            
            time_right_1 = time.time()
            right_angle_1 = (euler_from_quaternion(rot))[1]

            cmd.angular.z = 0
            cmd_pub.publish(cmd)

            for i in range(5):
                rate.sleep()

            print("right_data_collected")

            print("angle_from_to and time")
            print(left_angle_0)
            print(left_angle_1)
            print(time_left_0-time_left_1)

            print("right")
            print(right_angle_0)
            print(right_angle_1)
            print(time_right_0-time_right_1)

            print("calculation")
            actual_left_speed = (left_angle_0-left_angle_1)/(time_left_0-time_left_1)
            print("actual left speed")
            print(actual_left_speed)
            actual_right_speed = ((right_angle_0-right_angle_1)/(time_right_0-time_right_1))
            print("actual_right_speed")
            print(actual_right_speed)
            actual_speed = (max(actual_left_speed, -actual_left_speed) + max(actual_right_speed, -actual_right_speed))/2
            print(actual_speed)
            operation = "align_1_rotate_L"
            print("changes_operation_to_align_1")
            
            for i in range(5):
                rate.sleep()

        elif(operation == "align_1_rotate_L"):
            print("operation:" + operation)
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            euler = euler_from_quaternion(rot)
            print("quaternion:")
            print(rot)
            print("euler")
            print(euler_from_quaternion(rot))
            print("linear")
            print(trans)
            # euler -> (x,y,z) in relation to gazebo, the angle that we want is y
            angle = euler[1]
            print("angle:")
            print(angle)
            print("initiating rotate")
            cmd.angular.z=-0.2
            operation_time = float((math.pi/2.0 + angle)/actual_speed)
            print("rotating_time" + str(operation_time))
            start_time = time.time()
            while(time.time() - start_time <= operation_time ):
                cmd_pub.publish(cmd)
            print("rotate_operation_complete")
            cmd.angular.z = 0
            #stop the rotation
            cmd_pub.publish(cmd)
            rate.sleep()
            operation = 'align_2_forward_L'

        elif(operation == "align_2_forward_L"):
            print("operation:" + operation)
            cmd.linear.x = -0.2
            operation_time = float((trans[2] * math.cos(math.pi/2.0 + angle))/actual_speed)
            print("forward_time" + str(operation_time))
            start_time = time.time()
            while(time.time() - start_time <= operation_time):
                cmd_pub.publish(cmd)
            print("forward_operation_complete")
            cmd.linear.x=0
            cmd_pub.publish(cmd)
            rate.sleep()
            operation = 'align_3_rotate_L'
        
        elif(operation == "align_3_rotate_L"):
            print("operation:" + operation)
            cmd.angular.z = 0.2
            operation_time = float((math.pi/2.0)/actual_speed)
            start_time = time.time()
            while(time.time() - start_time <= operation_time):
                cmd_pub.publish(cmd)
            print("align_3_rotate_operation_completed")
            cmd.angular.z = 0
            cmd_pub.publish(cmd)
            rate.sleep()
            for i in range(5):
                rate.sleep()
            operation = "entry"
            print("change_operation_to_entry")

### operation of robot docking from the right (applicable to robot only, will not work in gazebo for some reason)
        elif(operation == "calibration_R"):
            for i in range(4):
                rate.sleep()

            trans = []
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                trans = [1000000000]

            print("calibrating_right")
            calibration_right = 1
            right_angle_0 = (euler_from_quaternion(rot))[1]
            time_right_0 = time.time()

            while(calibration_right):
                cmd.angular.z = -0.2
                try:
                    (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

                if(trans[0]<0.4):
                    calibration_right = 0
                cmd_pub.publish(cmd)
                rate.sleep()
            
            time_right_1 = time.time()
            right_angle_1 = (euler_from_quaternion(rot))[1]

            cmd.angular.z = 0
            cmd_pub.publish(cmd)

            for i in range(5):
                rate.sleep()

            print("right_data_collected")

            trans = []
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                trans = [1000000000]  

            print("calibrating_left")
            calibration_left = 1
            left_angle_0 = (euler_from_quaternion(rot))[1]
            time_left_0 = time.time()
            while(calibration_left):
                cmd.angular.z = 0.2
                try:
                    (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    pass

                if(trans[0]>-0.05):
                    calibration_left = 0
                cmd_pub.publish(cmd)
                rate.sleep()

            time_left_1 = time.time()
            left_angle_1 = (euler_from_quaternion(rot))[1]
            
            cmd.angular.z = 0
            cmd_pub.publish(cmd)

            print("left_data_collected")

            for i in range(5):
                rate.sleep()

            print("angle_from_to and time")
            print(left_angle_0)
            print(left_angle_1)
            print(time_left_0-time_left_1)

            print("right")
            print(right_angle_0)
            print(right_angle_1)
            print(time_right_0-time_right_1)

            print("calculation")
            actual_left_speed = (left_angle_0-left_angle_1)/(time_left_0-time_left_1)
            print("actual left speed")
            print(actual_left_speed)
            actual_right_speed = ((right_angle_0-right_angle_1)/(time_right_0-time_right_1))
            print("actual_right_speed")
            print(actual_right_speed)
            actual_speed = (max(actual_left_speed, -actual_left_speed) + max(actual_right_speed, -actual_right_speed) )/2
            print(actual_speed)
            operation = "align_1_rotate_R"
            print("changes_operation_to_align_1")
            
            for i in range(5):
                rate.sleep()

        elif(operation == "align_1_rotate_R"):

            #debug, to remove
            '''
            actual_speed = 0.1
            rot = quaternion_from_euler(3.14, 0.9, 0)
            trans = [-0.048, 0, 0.8]
            '''
            #to include below
            
            print("operation:" + operation)
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
            euler = euler_from_quaternion(rot)
            print("quaternion:")
            print(rot)
            print("euler")
            print(euler_from_quaternion(rot))
            print("linear")
            print(trans)
            # euler -> (x,y,z) in relation to gazebo, the angle that we want is y
            angle = euler[1]
            print("angle:")
            print(angle)
            print("initiating rotate")
            cmd.angular.z=0.2
            #this part below might need to double check. just see first what angle it gives lorh. here i only change to negatice angle
            operation_time = float((math.pi/2.0 - angle)/actual_speed)
            print("rotating_time" + str(operation_time))
            start_time = time.time()
            while(time.time() - start_time <= operation_time ):
                cmd_pub.publish(cmd)
            print("rotate_operation_complete")
            cmd.angular.z = 0
            #stop the rotation
            cmd_pub.publish(cmd)
            rate.sleep()
            operation = 'align_2_forward_R'

        elif(operation == "align_2_forward_R"):
            print("operation:" + operation)
            cmd.linear.x = -0.2
            #this part below angle also need to be checked
            operation_time = float((trans[2] * math.cos(math.pi/2.0 - angle))/actual_speed)
            print("forward_time" + str(operation_time))
            start_time = time.time()
            while(time.time() - start_time <= operation_time):
                cmd_pub.publish(cmd)
            print("forward_operation_complete")
            cmd.linear.x=0
            cmd_pub.publish(cmd)
            rate.sleep()
            operation = 'align_3_rotate_R'
        
        elif(operation == "align_3_rotate_R"):
            print("operation:" + operation)
            cmd.angular.z = -0.2
            operation_time = float((math.pi/2.0)/actual_speed)
            start_time = time.time()
            while(time.time() - start_time <= operation_time):
                cmd_pub.publish(cmd)
            print("align_3_rotate_operation_completed")
            cmd.angular.z = 0
            cmd_pub.publish(cmd)
            rate.sleep()
            for i in range(5):
                rate.sleep()
            operation = "entry"
            print("change_operation_to_entry")

        elif(operation == "entry"):
            print("operation:" + operation)
            itr_entry+=1
            try:
                (trans,rot) = listener.lookupTransform('/cameraLower_color_optical_frame', '/ar_marker_8',rospy.Time(0))
                #if use ('/front_camera_color_frame', '/ar_marker_255', ..), then the linear is same as published in the ar_pose_marker, which we want
                #if use ('/ar_marker_255', '/front_camera_color_frame', ..) then the angular is same as published in the ar_pose_marker
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        
            x_dist = -(trans[2] - 0.20)
            y_dist = -(trans[0])

            #trans[0] is the y-value for the actual robot, trans[1] y-value for this simulation
            #trans[2] is the x value for the actual robot, trans[0] x-value for this simulation

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

            ##P control
            #linear_vel = P_x*x_error + prev_linear_vel
            #angular_vel = P_y*y_error + prev_angular_vel

            #PI control
            #x_error_integral = (x_error-prev_x_error)*0.1
            #y_error_integral = (y_error-prev_y_error)*0.1

            x_error_integral += x_error*0.1
            y_error_integral += y_error*0.1

            print("x_error_integral: " + str(x_error_integral))
            print("y_error_integral: " + str(y_error_integral))

            linear_vel = K_P_x*x_error  + K_I_x*x_error_integral
            angular_vel = K_P_y*y_error + K_I_y*y_error_integral

            # moving the robot based on the linea_vel or the angular_vel
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            cmd_pub.publish(cmd)

            print("cmd: " + str(cmd))

            prev_x_dist = x_dist
            prev_y_dist = y_dist 
            prev_x_error = x_error
            prev_y_error = y_error

            #to collect data for graphing
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
            


#---end of docking

        rate.sleep()    
