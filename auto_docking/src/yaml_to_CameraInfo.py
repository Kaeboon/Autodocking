#!/usr/bin/env python 
"""
pointgrey_camera_driver (at least the version installed with apt-get) doesn't
properly handle camera info in indigo.
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.
The yaml parsing is courtesy ROS-user Stephan:
    http://answers.ros.org/question/33929/camera-calibration-parser-in-python/
This file just extends that parser into a rosnode.
"""
import rospy
import yaml
from sensor_msgs.msg import CameraInfo


"""
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
"""
 
if __name__ == "__main__":

    camera_info_msg = CameraInfo()
    camera_info_msg.width = 1280
    camera_info_msg.height = 720
    camera_info_msg.K = [911.714599609375, 0.0, 638.2435913085938, 0.0, 909.9718017578125, 365.1244812011719, 0.0, 0.0, 1.0]
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [911.714599609375, 0.0, 638.2435913085938, 0.0, 0.0, 909.9718017578125, 365.1244812011719, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info_msg.distortion_model = "plumb_bob"

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()