#!/usr/bin/python3

import rospy
import rospkg
import tf
from geometry_msgs.msg import TransformStamped

import time
import yaml


class StateTF:

    def __init__(self):
        r = rospkg.RosPack()
        pwd_params = r.get_path('map_hokuyo')+'/config/params.yaml'

        with open(pwd_params, 'r') as f:
            self.params = yaml.load(f, Loader=yaml.FullLoader)["map_hokuyo"]

        self.tf_listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

    def world2motor(self):
        try:
            self.tf_listener.waitForTransform(
                self.params["frames"]["world"],
                self.params["frames"]["motor"],
                rospy.Time.now(),
                rospy.Duration(3.0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("tf error")

        tf_tfs = TransformStamped()

        tf_tfs.header.stamp = rospy.Time.now()
        tf_tfs.header.frame_id = self.params["frames"]["world"]
        tf_tfs.child_frame_id = self.params["frames"]["motor"]

        xyz = self.params["origins"]["world2motor"]["xyz"]

        tf_tfs.transform.translation.x = xyz[0]
        tf_tfs.transform.translation.y = xyz[1]
        tf_tfs.transform.translation.z = xyz[2]

        tf_tfs.transform.rotation.x = 0
        tf_tfs.transform.rotation.y = 0
        tf_tfs.transform.rotation.z = 0
        tf_tfs.transform.rotation.w = 1

        self.tf_br.sendTransformMessage(tf_tfs)

    def motor2lidar(self):
        tf_tfs = TransformStamped()

        tf_tfs.header.stamp = rospy.Time.now()
        tf_tfs.header.frame_id = self.params["frames"]["motor"]
        tf_tfs.child_frame_id = self.params["frames"]["lidar"]

        xyz = self.params["origins"]["motor2lidar"]["xyz"]

        tf_tfs.transform.translation.x = xyz[0]
        tf_tfs.transform.translation.y = xyz[1]
        tf_tfs.transform.translation.z = xyz[2]

        tf_tfs.transform.rotation.x = 0
        tf_tfs.transform.rotation.y = 0
        tf_tfs.transform.rotation.z = 0
        tf_tfs.transform.rotation.w = 1

        self.tf_br.sendTransformMessage(tf_tfs)


if __name__ == '__main__':
    rospy.init_node('map_hokuyo')
    state_tf = StateTF()
    time.sleep(1)
    state_tf.world2motor()
    state_tf.motor2lidar()
    rospy.loginfo("\033[1;32m-> StateTF.\033[0m")
    rospy.spin()
