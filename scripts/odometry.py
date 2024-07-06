#!/usr/bin/env python3 

import rospy  
import numpy as np 
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler   

class PuzzlebotOdometry() :  
    def __init__(self) :  
        # INITIATING THE NODE
        rospy.init_node('puzzlebot_odometry')
        print("PUZZLEBOT ODOMETRY Node Initialized")

        rospy.on_shutdown(self.shutdown) 

        # SUBSCRIBERS
        rospy.Subscriber("/puzzlebot_1/wr", Float32, self.wr_cb) 
        rospy.Subscriber("/puzzlebot_1/wl", Float32, self.wl_cb)

        # PUBLISHERS 
        self.odometry_pub = rospy.Publisher("/odom", Odometry, queue_size = 1)
        self.robot_orientation_pub = rospy.Publisher("/orientation", Float32, queue_size = 1) 

        # ROBOT CONSTANTS
        self.r = 0.05       # Puzzlebot wheel radius [m] 
        self.L = 0.19       # Puzzlebot wheel separation [m] 
        self.dt = 0.02      # Desired time to update the robot's pose [s] 

        # VARIABLES 
        self.wr = 0.0
        self.wl = 0.0
        self.robot_X = 0.0              # <-- Change to define where the Robot start
        self.robot_Y = 0.0              # <---┘
        self.robot_THETA = 0.0          # <---┘

        self.odom = Odometry()
        self.robot_orientation = Float32()

        v = 0.0
        w = 0.0

        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0 :
            print("NO simulated time has been received")
        print("TIME RECEIVED")

        prev_time = rospy.get_time()

        while not rospy.is_shutdown() :
            self.dt = rospy.get_time() - prev_time
            prev_time = rospy.get_time()

            v = self.r * ((self.wl + self.wr) / 2.0)
            w = self.r * ((self.wr - self.wl) / self.L)

            self.robot_THETA = self.robot_THETA + (w * self.dt)
            self.robot_THETA = np.arctan2(np.sin(self.robot_THETA), np.cos(self.robot_THETA))
            self.robot_X = self.robot_X + (v * np.cos(self.robot_THETA) * self.dt)
            self.robot_Y = self.robot_Y + (v * np.sin(self.robot_THETA) * self.dt)

            self.odom = self.set_odometry(self.robot_X, self.robot_Y, self.robot_THETA, v, w)
            self.robot_orientation = self.robot_THETA

            self.odometry_pub.publish(self.odom)
            self.robot_orientation_pub.publish(self.robot_orientation)
            rate.sleep()

    def wr_cb(self, msg) :        
        self.wr =  msg.data         # Right wheel angular speed in [rad/s] 
    
    def wl_cb(self, msg) :
        self.wl = msg.data          # Left wheel angular speed in [rad/s]

    def set_odometry(self, X, Y, THETA, v, w) :
        odom = Odometry()
        odom.header.stamp = rospy.Time.now() 
        odom.header.frame_id = "odom" 
        odom.child_frame_id = "base_link" 

        odom.pose.pose.position.x = X
        odom.pose.pose.position.y = Y
        odom.pose.pose.position.z = 0.0 

        quat = quaternion_from_euler(0.0, 0.0, THETA) 
        odom.pose.pose.orientation.x = quat[0] 
        odom.pose.pose.orientation.y = quat[1] 
        odom.pose.pose.orientation.z = quat[2] 
        odom.pose.pose.orientation.w = quat[3] 

        odom.pose.covariance = [0.0] * 36
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        return odom
    
    def shutdown(self) :
        print("PUZZLEBOT ODOMETRY Node Killed")
    
if __name__ == "__main__" :
    PuzzlebotOdometry()