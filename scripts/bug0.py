#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class BUG_0():  
    def __init__(self) :
        # INITIATING THE NODE
        rospy.init_node("bug_0")
        print("BUG 0 Node Initialized")

        rospy.on_shutdown(self.shutdown) 

        # PUBLISHERS
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        # SUBSCRIBERS
        rospy.Subscriber("/goal", Point, self.goal_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/scan", LaserScan, self.lidar_cb)

        # ROBOT CONSTANTS
        self.r = 0.05 #wheel radius [m]
        self.L = 0.19 #wheel separation [m]
        self.goal_received = 0
        goal_X = 0.0
        goal_Y = 0.0
        kw = 1.2
        kv = 0.07
        self.theta_avd = 0.0
        self.robot_THETA = 0.0
        self.robot_X = 0.0
        self.robot_Y = 0.0
        v = 0.0
        w = 0.0
        dt = 0.1
        self.wl = 0.0
        self.wr = 0.0
        d = 1e100
        d_min = 0.07
        self.closest_angle = 0.0 #Angle to the closest object
        self.closest_d = np.inf #Distance to the closest object
        robot_vel = Twist() #The robot's velocity 
        self.avs_dist = 0.26

        rate = rospy.Rate(int(1.0 / dt))

        while rospy.get_time() == 0 :
            print("NO simulated time has been received")
        print("TIME RECEIVED")

        while not rospy.is_shutdown():
            if self.goal_received :
                d = np.sqrt(((self.goal_X - self.robot_X) ** 2) + ((self.goal_Y - self.robot_Y) ** 2))
                self.goal_THETA = np.arctan2((self.goal_Y - self.robot_Y), (self.goal_X - self.robot_X))
                etheta = self.goal_THETA - self.robot_THETA
                etheta = np.arctan2(np.sin(etheta), np.cos(etheta)) 

                print("Position x: " + str(self.robot_X))
                print("Position y: " + str(self.robot_Y))

                if (self.closest_d < self.avs_dist and abs(self.closest_angle) < np.pi - 0.2):
                    robot_vel.linear.x, robot_vel.angular.z = self.avoid_obstacles_controller()
                    print("Avoiding")

                elif ((d > d_min) and ((self.closest_d > self.avs_dist) or abs(self.closest_angle) > np.pi - 0.2)):
                    v = kv * d
                    w = kw * etheta
                    if abs(etheta) > 0.185:
                        v = 0.0

                    robot_vel.linear.x = v
                    robot_vel.angular.z = w
                    print("Moving to goal")

                else:
                    print("Stop")
                    robot_vel.linear.x = 0.0
                    robot_vel.angular.z = 0.0

                if robot_vel.linear.x > 0.15:
                    robot_vel.linear.x = 0.15

                elif robot_vel.linear.x < -0.15:
                    robot_vel.linear.x = -0.15
                    
                if robot_vel.angular.z > 1.2:
                    robot_vel.angular.z = 1.2

                elif robot_vel.angular.z < -1.2:
                    robot_vel.angular.z = -1.2
            
            else:
                print("No goal yet...")

            self.pub_cmd_vel.publish(robot_vel) #publish the robot's speed 
            rate.sleep() 

    def odom_cb(self, Odometry):
        self.robot_X = Odometry.pose.pose.position.x
        self.robot_Y = Odometry.pose.pose.position.y

        self.robot_THETA = euler_from_quaternion([Odometry.pose.pose.orientation.x, Odometry.pose.pose.orientation.y, Odometry.pose.pose.orientation.z, Odometry.pose.pose.orientation.w])[2]

    def avoid_obstacles_controller(self):
        self.theta_avd = self.closest_angle - (np.pi / 2.0)
        self.theta_avd = np.arctan2(np.sin(self.theta_avd), np.cos(self.theta_avd)) #Limit the angle from [-pi, pi]
        k_avd_v = 0.4
        k_avd = 1.0 #Proportionality constant
        fw_THETA = (np.pi / 2) + self.theta_avd 
        fw_THETA = np.arctan2(np.sin(fw_THETA), np.cos(fw_THETA))
        v = k_avd_v * self.closest_d
        w = k_avd * self.theta_avd

        if w > 1.2:
            w = 1.2
        
        elif w < -1.2:
            w = -1.2

        if (self.closest_d < self.avs_dist) and (abs(fw_THETA - self.goal_THETA) > 0.0001):
            print("Spinning CounterClockwise.")
            if w > 0:
                w = -w
        elif (self.closest_d < self.avs_dist) and (abs(fw_THETA - self.goal_THETA) <= 0.0001):
            print("Spinning Clockwise.")
            if w < 0:
                w = -w

        return v, w

    def goal_cb(self, goal) :
        self.goal_received = 1
        self.goal_X = goal.x
        self.goal_Y = goal.y

    def lidar_cb(self, lidar) :
        self.lidar_received = 1
        ranges = np.array(lidar.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        self.closest_d = valid_ranges.min() if valid_ranges.size > 0 else float('inf')
        idx = ranges.tolist().index(self.closest_d) if valid_ranges.size > 0 else 0
        self.closest_angle = lidar.angle_min + idx * lidar.angle_increment
        self.closest_angle = np.arctan2(np.sin(self.closest_angle), np.cos(self.closest_angle))

    def shutdown(self) :    
        stop_msg = Twist() 
        self.pub_cmd_vel.publish(stop_msg)
        print("BUG 0 Node Killed")
 
if __name__ == "__main__":  
    BUG_0()


