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


class TfSlam:

    def __init__(self):
        self.frame_dad = "world"
        self.frame_child = "quanergy"

        self.map = np.array([[0, 0, 0]])
        self.map_header = Header()
        self.map_header.frame_id = self.frame_dad

        self.tf_listener = tf.TransformListener()

        self.topic_pub_map = rospy.Publisher(
            '/apms_slam/map/pointcloud',
            PointCloud2,
            queue_size=1)

        self.topic_sub_point = rospy.Subscriber(
            'reposted',
            PointCloud2,
            self.publish_map,
            queue_size=10,
            tcp_nodelay=True)

    def publish_map(self, point2_ros):
        try:
            self.tf_listener.waitForTransform(
                self.frame_dad, self.frame_child, point2_ros.header.stamp, rospy.Duration(3.0))

            (trans, rot) = self.tf_listener.lookupTransform(
                self.frame_dad, self.frame_child, point2_ros.header.stamp)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf error")

        rot = tf.transformations.euler_from_quaternion(rot)
        mp_velo = np.eye(4)
        mp_velo[:3, :3] = cv2.Rodrigues(rot)[0]
        mp_velo[0, 3] = trans[0]
        mp_velo[1, 3] = trans[1]
        mp_velo[2, 3] = trans[2]

        pts_3d_ros = ros_numpy.numpify(point2_ros)

        pts_3d_r1 = np.ones((4, pts_3d_ros.shape[1]))
        pts_3d_r1[0, :] = pts_3d_ros['x'][1, :]
        pts_3d_r1[1, :] = pts_3d_ros['y'][1, :]
        pts_3d_r1[2, :] = pts_3d_ros['z'][1, :]

        map_tmp1 = mp_velo.dot(pts_3d_r1).T[:, :3]

        pts_3d_r2 = np.ones((4, pts_3d_ros.shape[1]))
        pts_3d_r2[0, :] = pts_3d_ros['x'][3, :]
        pts_3d_r2[1, :] = pts_3d_ros['y'][3, :]
        pts_3d_r2[2, :] = pts_3d_ros['z'][3, :]

        map_tmp2 = mp_velo.dot(pts_3d_r2).T[:, :3]
        self.map = np.append(self.map, map_tmp1, axis=0)
        self.map = np.append(self.map, map_tmp2, axis=0)

        self.map_header.stamp = point2_ros.header.stamp

        self.topic_pub_map.publish(
            point_cloud2.create_cloud_xyz32(self.map_header, self.map))


if __name__ == '__main__':
    rospy.init_node('apms_slam_velo2Map', anonymous=True)
    tf_slam = TfSlam()
    rospy.loginfo("")
    rospy.loginfo("nodo: apms_slam_velo2Map Started ")
    rospy.loginfo("Python version: "+sys.version)
    rospy.loginfo("")
    rospy.spin()
