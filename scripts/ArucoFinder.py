#!/usr/bin/env python3 

import rospy 
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Float32MultiArray, Bool

class ArucoFinder:
    def __init__(self):
        # Inicio del Nodo
        rospy.init_node('aruco_finder')
        rospy.on_shutdown(self.cleanup)

        # Subscriptores
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.ft_cb)

        # Publicadores
        self.ar_array_pub = rospy.Publisher("/ar_array", Float32MultiArray, queue_size=1)
        self.arucos_flag_pub = rospy.Publisher("/arucos_flag", Bool, queue_size=1)
    
        # Robot Constants
        self.dt = 0.02
        self.ids = [701, 702, 703, 704, 705, 706, 707]
        self.xy = {"701": [0.0, 1.60],
                   "702": [0.0, 0.8],
                   "703": [1.73, 0.8],
                   "704": [2.63, 0.39],
                   "705": [2.85, 0.0],
                   "706": [2.865, 2.00],
                   "707": [1.735, 1.22]}
        self.camera_t_robot = np.array([
            [0.0, 0.0, 1.0, 0.1],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.065],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Variables
        self.fiducial_transform = FiducialTransformArray()
        self.ar_array = Float32MultiArray()
        self.arucos_flag = Bool()        
        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0:
            print("No time received")

        print("Success")

        while not rospy.is_shutdown():
            self.process_transforms()
            self.publish_data()
            rate.sleep()

    def ft_cb(self, msg):
        self.fiducial_transform = msg

    def process_transforms(self):
        for aruco in self.fiducial_transform.transforms:
            if aruco.fiducial_id in self.ids:
                print("Fiducial ID: ", aruco.fiducial_id, " (Q).")

                cam_p_aruco = np.array([
                    aruco.transform.translation.x, 
                    aruco.transform.translation.y, 
                    aruco.transform.translation.z
                ])

                robot_p_aruco = self.transform_marker_position(cam_p_aruco)

                distance_C_A = np.linalg.norm(cam_p_aruco)
                distance_R_A = np.linalg.norm(robot_p_aruco)
                angle_rad = np.arctan2(robot_p_aruco[1], robot_p_aruco[0])
                angle_deg = np.degrees(angle_rad)

                print(f"Distance camera to marker: {distance_C_A}")
                print(f"Distance robot to marker: {distance_R_A}")
                print(f"Angle to marker: {angle_deg} degrees")
		
		# Publish data
                self.ar_array.data = [self.xy[str(aruco.fiducial_id)][0], self.xy[str(aruco.fiducial_id)][1], distance_R_A, angle_deg]
                self.arucos_flag.data = True

    def publish_data(self):
        self.ar_array_pub.publish(self.ar_array)
        self.arucos_flag_pub.publish(self.arucos_flag)

    def transform_marker_position(self, cam_p_aruco):
        cam_p_aruco_homogeneous = np.append(cam_p_aruco, 1.0)  # Convert to homogeneous coordinates
        robot_p_aruco_homogeneous = np.dot(self.camera_t_robot, cam_p_aruco_homogeneous)
        robot_p_aruco = robot_p_aruco_homogeneous[:3]  # Convert back to Cartesian coordinates
        return robot_p_aruco

    def cleanup(self):
        print("Ending Node")

if __name__ == "__main__":
    ArucoFinder()
