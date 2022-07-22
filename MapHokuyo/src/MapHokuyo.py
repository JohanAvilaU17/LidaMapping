#!/usr/bin/python3

import rospy
import rospkg
from std_msgs.msg import Int16
from sensor_msgs.msg import MultiEchoLaserScan
from geometry_msgs.msg import PointStamped

import yaml
import cv2
import numpy as np
import open3d as o3d


class MapHokuyo:

    def __init__(self):
        self.sub_lidar = None
        self.sub_motor = None

        self.flag_Lidar = False

        self.pub_lidar_flag = None
        self.pub_motor_flag = None

        r = rospkg.RosPack()
        pwd_params = r.get_path('map_hokuyo')+'/config/params.yaml'

        with open(pwd_params, 'r') as f:
            params = yaml.load(f, Loader=yaml.FullLoader)["map_hokuyo"]

        self.coordinate_xyz = params["coordinate"]["xyz"]
        self.coordinate_rpy = params["coordinate"]["rpy"]

        self.map_echo0 = o3d.geometry.PointCloud()
        self.map_echo1 = o3d.geometry.PointCloud()
        self.map_echo2 = o3d.geometry.PointCloud()

        self.intensities_0 = o3d.utility.DoubleVector()
        self.intensities_1 = o3d.utility.DoubleVector()
        self.intensities_2 = o3d.utility.DoubleVector()

    def setting_subscriber(self):
        self.sub_lidar = rospy.Subscriber(
            "/echoes",
            MultiEchoLaserScan,
            self.scanning,
            queue_size=100)

        self.sub_motor = rospy.Subscriber(
            "/dynamixel_mx/pose",
            PointStamped,
            self.mapping,
            queue_size=100)

    def setting_publisher(self):
        self.pub_lidar_flag = rospy.Publisher(
            "map_hokuyo/lidar/flag",
            Int16)

        self.pub_motor_flag = rospy.Publisher(
            "map_hokuyo/motor/flag",
            Int16)

    def scanning(self, multiEcho_ros):
        self.pub_lidar_flag.publish(1)
        self.multiEcho_ros = multiEcho_ros
        self.flag_Lidar = True
        self.pub_lidar_flag.publish(0)

    def mapping(self, pose_motor_ros):
        self.pub_motor_flag.publish(1)
        while self.flag_Lidar == False:
            pass
        self.flag_Lidar = False
        multiEcho_ros = self.multiEcho_ros

        range_Lidar_ros = multiEcho_ros.ranges
        int_Lidar_ros = multiEcho_ros.intensities

        N = len(range_Lidar_ros)
        tetha = np.linspace(
            multiEcho_ros.angle_min, multiEcho_ros.angle_max, N)

        pcd_echo0 = o3d.geometry.PointCloud()
        pcd_echo1 = o3d.geometry.PointCloud()
        pcd_echo2 = o3d.geometry.PointCloud()

        intensities_0 = o3d.utility.DoubleVector()
        intensities_1 = o3d.utility.DoubleVector()
        intensities_2 = o3d.utility.DoubleVector()

        for idx in range(0, N):
            range_tmp = range_Lidar_ros[idx]
            intensities_tmp = int_Lidar_ros[idx]
            N_tmp = len(intensities_tmp.echoes)

            pcd_echo0.points.append([
                range_tmp.echoes[0]*np.cos(tetha[idx]),
                range_tmp.echoes[0]*np.sin(tetha[idx]),
                0])

            intensities_0.append(intensities_tmp.echoes[0])

            if N_tmp > 1:
                pcd_echo1.points.append([
                    range_tmp.echoes[1]*np.cos(tetha[idx]),
                    range_tmp.echoes[1]*np.sin(tetha[idx]),
                    0])

                intensities_1.append(intensities_tmp.echoes[1])

                if N_tmp > 2:
                    pcd_echo2.points.append([
                        range_tmp.echoes[2]*np.cos(tetha[idx]),
                        range_tmp.echoes[2]*np.sin(tetha[idx]),
                        0])

                    intensities_2.append(intensities_tmp.echoes[2])

        rvec = np.radians([0, -pose_motor_ros.point.x, 0])

        mp_lidar = np.eye(4)
        mp_lidar[:3, :3] = cv2.Rodrigues(rvec)[0]

        pcd_echo0.transform(mp_lidar)
        pcd_echo1.transform(mp_lidar)
        pcd_echo2.transform(mp_lidar)

        self.map_echo0.points.extend(pcd_echo0.points)
        self.map_echo1.points.extend(pcd_echo1.points)
        self.map_echo2.points.extend(pcd_echo2.points)

        self.intensities_0.extend(intensities_0)
        self.intensities_1.extend(intensities_1)
        self.intensities_2.extend(intensities_2)

        self.pub_motor_flag.publish(0)

    def projectCoordinates(self):
        print("projectCoordinates")

        rvec = np.radians(self.coordinate_rpy)

        mp_lidar = np.eye(4)
        mp_lidar[:3, :3] = cv2.Rodrigues(rvec)[0]
        mp_lidar[:3, 3] = self.coordinate_xyz

        self.map_echo0.transform(mp_lidar)
        self.map_echo1.transform(mp_lidar)
        self.map_echo2.transform(mp_lidar)

    def save_mapa(self):
        print("save_mapa")

        r = rospkg.RosPack()
        pwd_src = r.get_path('map_hokuyo')
        pwd_src = pwd_src.split("/")
        N = len(pwd_src)

        file_base = ""

        for idx in range(0, N-1):
            file_base += pwd_src[idx]+"/"

        file = file_base + "map_echoes.txt"
        with open(file, 'w') as f:
            f.write("X Y Z Echo Intensity\n")

        with open(file, 'a') as f:
            for idx, xyz_tmp in enumerate(self.map_echo0.points):
                x, y, z = xyz_tmp
                f.write(str(x)+" "+str(y)+" "+str(z)+" " +
                        str(0)+" "+str(self.intensities_0[idx])+"\n")

            for idx, xyz_tmp in enumerate(self.map_echo1.points):
                x, y, z = xyz_tmp
                f.write(str(x)+" "+str(y)+" "+str(z)+" " +
                        str(1)+" "+str(self.intensities_1[idx])+"\n")

            for idx, xyz_tmp in enumerate(self.map_echo2.points):
                x, y, z = xyz_tmp
                f.write(str(x)+" "+str(y)+" "+str(z)+" " +
                        str(2)+" "+str(self.intensities_2[idx])+"\n")


if __name__ == '__main__':
    rospy.init_node('map_hokuyo')
    map_hokuyo = MapHokuyo()
    map_hokuyo.setting_publisher()
    map_hokuyo.setting_subscriber()
    rospy.loginfo("\033[1;32m-> Map.\033[0m")
    rospy.spin()

    print("")
    map_hokuyo.projectCoordinates()
    map_hokuyo.save_mapa()
