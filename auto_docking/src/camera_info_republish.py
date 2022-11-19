#!/usr/bin/env python  
import rospy
import sensor_msgs
from sensor_msgs.msg import CameraInfo

if __name__ == '__main__':
    rate = rospy.Rate(10.0) #rate = 10Hz1
    rospy.init_node('auto_docking')
    camera_info_ori = rospy.Subscriber('/cameraUpper/color/camera_info', CameraInfo, queue_size=10)
    camera_info_new = rospy.Publisher('/camera_info_republish/camera_info', CameraInfo, queue_size=10)

    #cam_info = sensor_msgs/CameraInfo

    while not rospy.is_shutdown():
            camera_info_new.publish(camera_info_ori)

    rate.sleep()