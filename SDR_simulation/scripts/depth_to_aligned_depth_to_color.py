#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image


def callback(data):
    pub = rospy.Publisher('/cameraUpper/aligned_depth_to_color/image_raw', Image, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(data)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('depth_to_aligned_depth_to_color', anonymous=True)

    rospy.Subscriber("/cameraUpper/depth/image_raw", Image, callback)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()