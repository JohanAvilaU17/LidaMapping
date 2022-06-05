#!/usr/bin/python3

import sys
import numpy as np
import cv2

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import ros_numpy
from sensor_msgs import point_cloud2


class TfSlam:

    def __init__(self, frame_dad, frame_child):
        self.map = np.array([[0, 0, 0]])
        self.header = Header()
        self.header.frame_id = frame_dad
        self.frame_dad = frame_dad
        self.frame_child = frame_child
        self.tf_listener = tf.TransformListener()

    def setting_publisher(self, name_topic, type_msg_topic):
        self.topic_pub = rospy.Publisher(
            name_topic, type_msg_topic, queue_size=1)

    def setting_subscriber(self, name_topic, type_msg_topic):
        self.topic_point = rospy.Subscriber(name_topic,
                                            type_msg_topic,
                                            self.publish_map)

    def publish_map(self, point2_ros):
        try:
            self.tf_listener.waitForTransform(
                self.frame_dad, self.frame_child, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform(
                self.frame_dad, self.frame_child, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf error")

        rot = euler_from_quaternion(rot)
        mp_velo = np.eye(4)
        mp_velo[:3, :3] = cv2.Rodrigues(rot)[0]
        mp_velo[0, 3] = trans[0]
        mp_velo[1, 3] = trans[1]
        mp_velo[2, 3] = trans[2]

        pts_3d_ros = ros_numpy.numpify(point2_ros)

        pts_3d = np.ones((4, 1000))  # pts_3d_ros.shape[0] ))
        pts_3d[0, :] = pts_3d_ros['x'][0:1000]
        pts_3d[1, :] = pts_3d_ros['y'][0:1000]
        pts_3d[2, :] = pts_3d_ros['z'][0:1000]

        mp_tmp = mp_velo.dot(pts_3d).T[:, :3]
        self.map = np.append(self.map, mp_tmp, axis=0)
        self.topic_pub.publish(
            point_cloud2.create_cloud_xyz32(self.header, self.map))


if __name__ == '__main__':
    try:
        print("Python version:\t", sys.version)

        rospy.init_node('tf_slam_kitti')
        print("nodo = ", rospy.get_name())

        tf_slam = TfSlam("world", "velo_link")
        tf_slam.setting_publisher('/tf_slam/kitti/map/pointcloud', PointCloud2)
        tf_slam.setting_subscriber('/kitti/velo/pointcloud', PointCloud2)

        print("Press Ctrl-C to stop.")

        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
