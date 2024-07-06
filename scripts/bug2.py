#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class BUG2() :
    def __init__(self) :
        # INITIATING THE NODE
        rospy.init_node("bug_2")
        print("BUG 2 Node Initialized")

        rospy.on_shutdown(self.shutdown) 

        # PUBLISHERS
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        # SUBSCRIBERS
        rospy.Subscriber("/goal", Point, self.goal_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/orientation", Float32, self.orientation_cb)
        rospy.Subscriber("/scan", LaserScan, self.lidar_cb)

        # ROBOT CONSTANT
        self.dt = 0.2               # Desired time to update the robot pose [s] 

        # VARIABLES        
        self.robot_X = 0.0          # Robot x position [m]
        self.robot_Y = 0.0          # Robot y position [m]
        self.robot_THETA = 0.0      # Robot angle position [rad]

        self.x0 = rospy.get_param("robot_X", 0.0)    # <-- Change to define where the Robot start
        self.y0 = rospy.get_param("robot_Y", 0.0)    # <---┘

        self.goal_received = 0
        self.goal_X = 0.0           # Goal position in X
        self.goal_Y = 0.0           # Goal position in Y
        goal_THETA = 0.0 
        
        d = 0.0
        error_THETA = 0.0
        
        vel = Twist()

        v_AO = 0.08
        w_AO = 0.0
        v_GTG = 0.0
        w_GTG = 0.0
        
        k_v = 0.2
        k_w = 0.5
        k_w1_AO = 0.08
        k_w2_AO = 1.7

        d_min = 0.15                # Tolerated Distance to the Goal [m]
        epsilon = 0.1
        fw_THETA = 0.0
        stopping_d = 0.16
        avoiding_d = 0.27
        self.closest_d = 0.0
        self.hit_point_d = 0.0
        hit_point_reached = False
        self.avoiding_THETA = 0.0

        self.state = "STOP"
        self.lidar_received = 0

        rate = rospy.Rate(int(1.0 / self.dt))

        while rospy.get_time() == 0 :
            print("NO simulated time has been received")
        print("TIME RECEIVED")

        prev_time = rospy.get_time()

        while not rospy.is_shutdown() :
            self.dt = rospy.get_time() - prev_time
            prev_time = rospy.get_time()
            print("DT: " + str(self.dt) + "\n")

            if self.goal_received :
                d = np.sqrt(((self.goal_X - self.robot_X) ** 2) + ((self.goal_Y - self.robot_Y) ** 2))
                goal_THETA = np.arctan2((self.goal_Y - self.robot_Y), (self.goal_X - self.robot_X))
                error_THETA = goal_THETA - self.robot_THETA
                error_THETA = np.arctan2(np.sin(error_THETA), np.cos(error_THETA))

                if self.lidar_received :
                    self.lidar_received = 0
                    print("CLOSEST D: " + str(self.closest_d) + "\n")
                    if self.state == "STOP" :
                        if stopping_d > self.closest_d :
                            print("ROBOT STOPPED")
                            print("ROBOT TOO CLOSE TO AN OBJECT \n")
                        elif d < d_min :
                            print("GOAL REACHED")
                            print("ROBOT STOPPED \n")
                            self.goal_received = 0
                        else :
                            self.state = "GO TO GOAL"
                        vel.linear.x = 0.0
                        vel.angular.z = 0.0

                    elif self.state == "GO TO GOAL" :
                        fw_THETA = (np.pi / 2) + self.avoiding_THETA 
                        fw_THETA = np.arctan2(np.sin(fw_THETA), np.cos(fw_THETA))
                        if (self.closest_d < avoiding_d) and (abs(fw_THETA - goal_THETA) <= (np.pi / 2.0)) :
                            self.state = "AVOIDING OBSTACLE CCW"
                            self.hit_point_d = d
                            hit_point_reached = True
                        elif (self.closest_d < avoiding_d) and (abs(fw_THETA - goal_THETA) > (np.pi / 2.0)) :
                            self.state = "AVOIDING OBSTACLE CW"
                            self.hit_point_d = d
                            hit_point_reached = True
                        elif d < d_min :
                            self.state = "STOP"
                        else :
                            print("MOVING TO GOAL \n")
                            v_GTG = k_v * d

                            if self.closest_d <= 0.4 :
                                v_GTG = self.closest_d * 0.14
                            
                            w_GTG = k_w * error_THETA

                            vel.linear.x = v_GTG
                            vel.angular.z = w_GTG

                    elif self.state == "AVOIDING OBSTACLE CCW" :
                        if ((d < (self.hit_point_d - epsilon)) and 
                            ((abs(self.avoiding_THETA - goal_THETA) > (np.pi / 2.0)) or 
                                self.on_mline(self.goal_X, self.goal_Y))) :
                            self.state = "GO TO GOAL"
                            hit_point_reached = False
                        elif (d < d_min) or (self.closest_d < stopping_d) :
                            self.state = "STOP"
                        else :
                            print("AVOIDING THE OBSTACLE CCW \n")
                            fw_THETA = (np.pi / 2) + self.avoiding_THETA

                            if (-0.06 > fw_THETA > 0.06) and hit_point_reached :
                                w_AO = k_w1_AO * fw_THETA
                            else :
                                w_AO = k_w2_AO * fw_THETA

                        vel.linear.x = v_AO
                        vel.angular.z = w_AO                          

                    elif self.state == "AVOIDING OBSTACLE CW" :
                        if ((d < (self.hit_point_d - epsilon)) and 
                                ((abs(self.avoiding_THETA - goal_THETA) < (np.pi / 2.0)) or 
                                    self.on_mline(self.goal_X, self.goal_Y))) :
                            self.state = "GO TO GOAL"
                            hit_point_reached = False
                        elif (d < d_min) or (self.closest_d < stopping_d) :
                            self.state = "STOP"
                        else :
                            print("AVOIDING THE OBSTACLE CW \n")
                            fw_THETA = -(np.pi / 2) + self.avoiding_THETA

                            if (-0.06 > fw_THETA > 0.06) and hit_point_reached :
                                w_AO = k_w1_AO * fw_THETA
                            else :
                                w_AO = k_w2_AO * fw_THETA

                            vel.linear.x = v_AO
                            vel.angular.z = w_AO

                    else :
                        print("ROBOT OUT OF STATE MACHINE \n")
                        vel.linear.x = 0.0
                        vel.angular.z = 0.0 

                    print("Robot X position: " + str(self.robot_X))
                    print("Robot Y position: " + str(self.robot_Y))
                    print("Robot angle: " + str(self.robot_THETA) + "\n")
                    print("Distance: " + str(d))
                    print("ERROR-Theta: " + str(error_THETA) + "\n")
                else :
                    print("LIDAR NOT WORKING")
            else :
                print("GOAL NOT RECEIVED YET")

            if vel.linear.x >= 0.15 :
                vel.linear.x = 0.15
            elif vel.linear.x <= -0.15 :
                vel.linear.x = -0.15
            
            if vel.angular.z >= 1.4 :
                vel.angular.z = 1.4
            elif vel.angular.z <= -1.4 :
                vel.angular.z = -1.4

            self.pub_cmd_vel.publish(vel)
            rate.sleep()

    def goal_cb(self, goal) :
        self.goal_received = 1
        self.goal_X = goal.x
        self.goal_Y = goal.y

    def odom_cb(self, odom) :
        self.robot_X = odom.pose.pose.position.x
        self.robot_Y = odom.pose.pose.position.y

    def orientation_cb(self, orientation) :
        self.robot_THETA = orientation.data

    def lidar_cb(self, lidar) :
        self.lidar_received = 1
        ranges = np.array(lidar.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        self.closest_d = valid_ranges.min() if valid_ranges.size > 0 else float('inf')
        idx = ranges.tolist().index(self.closest_d) if valid_ranges.size > 0 else 0
        self.avoiding_THETA = lidar.angle_min + idx * lidar.angle_increment
        self.avoiding_THETA = np.arctan2(np.sin(self.avoiding_THETA), np.cos(self.avoiding_THETA))

    def on_mline(self, goal_X, goal_Y) :     
        A = goal_Y - self.y0
        B = self.x0 - goal_X
        C = (goal_X * self.x0) - (self.y0 * goal_Y)
        d = abs((A * self.robot_X) + (B * self.robot_Y) + C) / np.sqrt((A ** 2) + (B ** 2))
        return d < 0.1

    def shutdown(self) :    
        stop_msg = Twist() 
        self.pub_cmd_vel.publish(stop_msg)
        print("BUG 2 Node Killed")

if __name__ == "__main__" :
    BUG2()
