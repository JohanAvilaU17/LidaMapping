#!/usr/bin/python3

import sys
import numpy as np
import cv2

import rospy
import ros_numpy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

import tf
from tf.transformations import euler_from_quaternion


class TfSlam:

    def __init__(self):
        self.frame_dad = "world"
        self.frame_child = "velo_link"

        self.map = np.array([[0, 0, 0]])
        self.header = Header()
        self.header.frame_id = self.frame_dad

        self.tf_listener = tf.TransformListener()

        self.topic_sub_point = rospy.Subscriber(
            '/kitti/velo/pointcloud',
            PointCloud2,
            self.publish_map)

        self.topic_pub_map = rospy.Publisher(
            '/tf_slam/kitti/map/pointcloud',
            PointCloud2)

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

        pts_3d = np.ones((4, pts_3d_ros.shape[0]))
        pts_3d[0, :] = pts_3d_ros['x']
        pts_3d[1, :] = pts_3d_ros['y']
        pts_3d[2, :] = pts_3d_ros['z']

        mp_tmp = mp_velo.dot(pts_3d).T[:, :3]

        self.map = np.append(self.map, mp_tmp, axis=0)

        self.topic_pub_map.publish(
            point_cloud2.create_cloud_xyz32(self.header,  self.map))


if __name__ == '__main__':
    rospy.init_node('tf_slam_kitti')
    tf_slam = TfSlam()
    rospy.loginfo("")
    rospy.loginfo("nodo: tf_slam_kitti Started ")
    rospy.loginfo("Python version: "+sys.version)
    rospy.loginfo("")
    rospy.spin()
