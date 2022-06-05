#!/usr/bin/python3
import sys
import numpy as np

import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
import tf


class TfStatePublisher:

    def __init__(self):
        self.frame_dad = 'odom'
        self.frame_child = 'base_link'

        self.tf_br = tf.TransformBroadcaster()

        self.flag_odom = False
        self.flag_gps = False

        self.topic_sub_gps = rospy.Subscriber(
            '/vectornav/GPS',
            NavSatFix,
            self.get_geometry_coordinates,
            queue_size=2000,
            tcp_nodelay=True)

        self.topic_sub_odom = rospy.Subscriber(
            '/vectornav/Odom',
            Odometry,
            self.odometry2BaseLink,
            queue_size=2000,
            tcp_nodelay=True)

    def get_geometry_coordinates(self, NavSatFix_ros):
        self.gps_lat = NavSatFix_ros.latitude*np.pi/180.0
        self.gps_lon = NavSatFix_ros.longitude*np.pi/180.0
        self.flag_gps = True

    def odometry2BaseLink(self, Odometry_ros):
        tf_time_now = rospy.Time.now()
        while self.flag_gps == False:
            pass
        self.flag_gps = False
        gps_lat = self.gps_lat
        gps_lon = self.gps_lon

        if self.flag_odom == False:
            self.flag_odom = True
            self.odom_org_x = Odometry_ros.pose.pose.position.x
            self.odom_org_y = Odometry_ros.pose.pose.position.y
            self.odom_org_z = Odometry_ros.pose.pose.position.z

            qx = Odometry_ros.pose.pose.orientation.x
            qy = Odometry_ros.pose.pose.orientation.y
            qz = Odometry_ros.pose.pose.orientation.z
            qw = Odometry_ros.pose.pose.orientation.w

            rot = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
            self.odom_org_qz = rot[2]
        else:
            mp_ecef2enu = np.zeros((4, 4))
            mp_ecef2enu[0][0] = -np.sin(gps_lon)
            mp_ecef2enu[0][1] = np.cos(gps_lon)

            mp_ecef2enu[1][0] = -np.sin(gps_lat)*np.cos(gps_lon)
            mp_ecef2enu[1][1] = -np.sin(gps_lat)*np.sin(gps_lon)
            mp_ecef2enu[1][2] = np.cos(gps_lat)

            mp_ecef2enu[2][0] = np.cos(gps_lon)*np.cos(gps_lat)
            mp_ecef2enu[2][1] = np.cos(gps_lat)*np.sin(gps_lon)
            mp_ecef2enu[2][2] = np.sin(gps_lat)

            mp_ecef2enu[3][3] = 1

            pts_ecef = np.ones((4, 1))
            pts_ecef[0][0] = (
                Odometry_ros.pose.pose.position.x - self.odom_org_x)
            pts_ecef[1][0] = (
                Odometry_ros.pose.pose.position.y - self.odom_org_y)
            pts_ecef[2][0] = (
                Odometry_ros.pose.pose.position.z - self.odom_org_z)

            pts_enu = mp_ecef2enu.dot(pts_ecef)

            qx = Odometry_ros.pose.pose.orientation.x
            qy = Odometry_ros.pose.pose.orientation.y
            qz = Odometry_ros.pose.pose.orientation.z
            qw = Odometry_ros.pose.pose.orientation.w

            rot = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))
            rot = list(rot)

            rot[0] = -rot[0]
            rot[1] = -rot[1]
            rot[2] = -(rot[2]-self.odom_org_qz)
            rot_q = tf.transformations.quaternion_from_euler(
                rot[0], rot[1], rot[2])

            tf_tfs_tmp = TransformStamped()

            tf_tfs_tmp.header.stamp = tf_time_now
            tf_tfs_tmp.header.frame_id = self.frame_dad
            tf_tfs_tmp.child_frame_id = self.frame_child

            tf_tfs_tmp.transform.translation.x = pts_enu[0][0]
            tf_tfs_tmp.transform.translation.y = pts_enu[1][0]
            tf_tfs_tmp.transform.translation.z = pts_enu[2][0]

            tf_tfs_tmp.transform.rotation.x = rot_q[0]
            tf_tfs_tmp.transform.rotation.y = rot_q[1]
            tf_tfs_tmp.transform.rotation.z = rot_q[2]
            tf_tfs_tmp.transform.rotation.w = rot_q[3]

            self.tf_br.sendTransformMessage(tf_tfs_tmp)


if __name__ == '__main__':
    rospy.init_node('apms_slam_odometry2BaseLink', anonymous=True)
    tf_state_publisher = TfStatePublisher()
    rospy.loginfo("")
    rospy.loginfo("nodo: apms_slam_odometry2BaseLink Started ")
    rospy.loginfo("Python version: "+sys.version)
    rospy.loginfo("")
    rospy.spin()
