#!/usr/bin/python3

import rospy
from sensor_msgs.msg import PointCloud2

nube = 0


def callback(data):
    data.header.stamp = rospy.get_rostime()
    nube = data
    pub.publish(nube)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/quanergy/points0", PointCloud2, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    pub = rospy.Publisher('reposted', PointCloud2, queue_size=10)
    listener()
