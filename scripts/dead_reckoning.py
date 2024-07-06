#!/usr/bin/env python3 

import rospy  
import numpy as np 
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist  
from tf.transformations import quaternion_from_euler 

class DeadReckoning() :  
    def __init__(self) :  
        # INITIATING THE NODE
        rospy.init_node('puzzlebot_dead_reckoning')
        print("PUZZLEBOT DEAD RECKONING Node Initialized")

        rospy.on_shutdown(self.shutdown) 

        # SUBSCRIBERS
        rospy.Subscriber("/wr", Float32, self.wr_cb) 
        rospy.Subscriber("/wl", Float32, self.wl_cb)

        # PUBLISHERS 
        self.odometry_pub = rospy.Publisher("/odom", Odometry, queue_size = 1)
        self.robot_orientation_pub = rospy.Publisher("/orientation", Float32, queue_size = 1) 

        # ROBOT CONSTANTS
        self.r = 0.05       # Puzzlebot wheel radius [m] 
        self.L = 0.19       # Puzzlebot wheel separation [m] 
        self.dt = 0.02      # Desired time to update the robot's pose [s] 

        # VARIABLES 
        self.w = 0.0  
        self.v = 0.0  
        self.wr = 0.0
        self.wl = 0.0
        
        self.odometry = Odometry()

        self.H = np.identity(3)
        self.Q = np.zeros((3, 3))
        self.Mu = np.zeros((3, 1))
        self.SDK = np.zeros((2, 2))
        self.Sigma = np.array([[63.43446, -22.4406, -4.16125], 
                               [-22.4406, 91.001282, -31.8], 
                               [-4.16125, -31.8, 44.72821]])
        
        self.last_Mu = np.zeros((3, 1))         # <-- Change to define where the Robot start
        self.last_Sigma = np.zeros((3, 3))

        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0 :
            print("NO simulated time has been received")
        print("TIME RECEIVED")

        self.prev_time = rospy.get_time()

        while not rospy.is_shutdown() : 
            self.v = self.r * ((self.wl + self.wr) / 2.0)
            self.w = self.r * ((self.wr - self.wl) / self.L)

            self.update_robot_pose(self.v, self.w, self.wl, self.wr) 
            self.odometry = self.get_odometry(self.Mu, self.Sigma, self.v, self.w)  

            self.robot_orientation_pub.publish(self.Mu[2, 0])
            self.odometry_pub.publish(self.odometry) 
            rate.sleep() 

    def wr_cb(self, msg) :        
        self.wr =  msg.data         # Right wheel angular speed in [rad/s] 
    
    def wl_cb(self, msg) :
        self.wl = msg.data          # Left wheel angular speed in [rad/s]

    def get_odometry(self, Mu, Sigma, v, w) : 
        odom = Odometry() 
        odom.header.stamp = rospy.Time.now() 
        odom.header.frame_id = "odom" 
        odom.child_frame_id = "base_link" 

        odom.pose.pose.position.x = Mu[0, 0] 
        odom.pose.pose.position.y = Mu[1, 0] 
        odom.pose.pose.position.z = 0.0 

        quat = quaternion_from_euler(0.0, 0.0, Mu[2, 0]) 
        odom.pose.pose.orientation.x = quat[0] 
        odom.pose.pose.orientation.y = quat[1] 
        odom.pose.pose.orientation.z = quat[2] 
        odom.pose.pose.orientation.w = quat[3] 

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = Sigma[0][0] * 10      # Variance in x sxx
        odom.pose.covariance[1] = Sigma[0][1]           # Variance in x sxy
        odom.pose.covariance[5] = Sigma[0][2]           # Variance in x sxtheta
        odom.pose.covariance[6] = Sigma[1][0]           # Variance in y syx
        odom.pose.covariance[7] = Sigma[1][1] * .5      # Variance in y syy
        odom.pose.covariance[11] = Sigma[1][2]          # Variance in y sytheta
        odom.pose.covariance[30] = Sigma[2][0]          # Variance in theta sthetax
        odom.pose.covariance[31] = Sigma[2][1]          # Variance in theta sthetay 
        odom.pose.covariance[35] = Sigma[2][2] * .2     # Variance in theta sthetatheta
        
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w   
        
        return odom 

    def update_robot_pose(self, v, w, wl, wr) :
        self.dt = rospy.get_time() - self.prev_time
        self.prev_time = rospy.get_time()

        # Estimated Robot Position
        self.Mu[0, 0] = self.last_Mu[0, 0] + self.dt * v * np.cos(self.last_Mu[2, 0])
        self.Mu[1, 0] = self.last_Mu[1, 0] + self.dt * v * np.sin(self.last_Mu[2, 0])
        self.Mu[2, 0] = self.last_Mu[2, 0] + self.dt * w

        # Linearised Model
        self.H = np.array([[1.0, 0.0, -self.dt * v * np.sin(self.last_Mu[2, 0])],
                        [0.0, 1.0, self.dt * v * np.cos(self.last_Mu[2, 0])],
                        [0.0, 0.0, 1.0]])

        # Propagation of Uncertainity
        self.SDK = [[(12 * abs(wr)), 0], [0, (12 * abs(wl))]]         # <--- kr and kl
        gWk = (1/2 * self.r * self.dt) * np.array([[np.cos(self.last_Mu[2, 0]), np.cos(self.last_Mu[2, 0])], 
                                                   [np.sin(self.last_Mu[2, 0]), np.sin(self.last_Mu[2, 0])], 
                                                   [2/self.L, -(2/self.L)]])
        self.Q = gWk @ self.SDK @ gWk.T
        self.Sigma = self.H @ self.last_Sigma @ self.H.T + self.Q

        self.last_Mu = np.copy(self.Mu)
        self.last_Sigma = np.copy(self.Sigma)

    def shutdown(self) :
        print("PUZZLEBOT DEAD RECKONING Node Killed")

if __name__ == "__main__" :  
    DeadReckoning()