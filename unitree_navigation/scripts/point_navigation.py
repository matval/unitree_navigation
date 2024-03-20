#!/usr/bin/env python3
import os
import time
import numpy as np

import rospy
from fpn_msgs.msg import MPCInput, GoalMultiArray, MHEOutput
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from utils.mhe_utils import dlatlon2dxy, euler_from_quaternion

class POINT_NAVIGATION:
    def __init__(self):
        # Params
        self.max_error          = rospy.get_param('~max_error', 0.5)
        self.navigation_speed   = rospy.get_param('~navigation_speed', 0.6)
        self.verbose            = rospy.get_param('~verbose', False)
        self.min_acc            = rospy.get_param('~min_acc', 10.0)
        wps_path                = rospy.get_param('~waypoints_path', 'south_quad.csv')
        # self.recovery_time      = 4.0
        # self.trav_threshold     = 0.2

        # Topic params
        auto_mode_topic     = rospy.get_param("~auto_mode_topic", "enable_auto_mode")
        output_path_topic   = rospy.get_param("~output_path_topic", "path")
        goals_file          = rospy.get_param("~goals_topic", "goals.csv")
        mhe_topic           = rospy.get_param("~mhe_topic", 'mhe_output')

        # Load waypoints file
        self.goals_list = []
        wps_file = open(wps_path)
        text = wps_file.readline()
        while text != "":
            point = text.split(" ")  
            self.goals_list.append([float(point[0]), float(point[1])])
            text = wps_file.readline()
        wps_file.close()
        
        # Publishers:
        self.pub_path = rospy.Publisher(output_path_topic, Path, queue_size=1)
        self.pub_mpc_input = rospy.Publisher(output_path_topic + "2", MPCInput, queue_size=1)

        # Subscribers:
        rospy.Subscriber(auto_mode_topic, Int8, self.enableAutoModeCb, queue_size=1)
        rospy.Subscriber(mhe_topic, MHEOutput, self.mheCallback, queue_size=1)

        self.goals          = []
        self.goal_count     = 0
        self.auto_mode      = -1
        self.trav           = 1.0
        self.odom           = None

        rate = rospy.Rate(50) # ROS Rate at 100 Hz

        while not rospy.is_shutdown():
            self.run_navigation()
            rate.sleep()

    def run_navigation(self):
        mpc_input = MPCInput()
        path_msg = Path()

        mpc_input.header.stamp = rospy.Time.now()
        mpc_input.header.frame_id = "base_link"
        mpc_input.reference_speed = self.navigation_speed

        # End of navigation
        if self.goal_count == len(self.goals):
            self.auto_mode = -1
        
        # If not in auto mode:
        if (self.auto_mode == -1) or len(self.goals) == 0:
            if self.verbose:
                rospy.logerr("NAV is not in auto mode...")
                rospy.logerr("self.auto_mode: {}".format(self.auto_mode))
                rospy.logerr("len(self.goals): {}".format(len(self.goals)))
                rospy.logerr("self.goal_count: {}".format(self.goal_count))
            
            path_msg.poses.clear()  # send empty waypoint list to mpc to stop robot
            path_msg.header.stamp = rospy.Time.now()

        else:
            if self.odom is None:
                rospy.logerr("Odometry message not received yet...")
                return

            # if self.trav < self.trav_threshold:
            #     init_time = time.time()
            #     while time.time() - init_time < 1.0:
            #         mpc_input.reference_speed = 0.0
            #         path_msg = Path()
            #         path_msg.poses.clear()
            #         path_msg.header = mpc_input.header
            #         mpc_input.poses = path_msg.poses
            #         self.pub_path.publish(path_msg)
            #         self.pub_mpc_input.publish(mpc_input)

            #     init_time = time.time()
            #     while time.time() - init_time < self.recovery_time:
            #         mpc_input.reference_speed = -0.4
            #         goal_pose = PoseStamped()
            #         goal_pose.header = mpc_input.header
            #         goal_pose.pose.position.x = -5.0
            #         goal_pose.pose.position.y = 0.0

            #         path_msg = Path()
            #         path_msg.poses.append(goal_pose)
            #         path_msg.header = mpc_input.header

            #         mpc_input.poses = path_msg.poses

            #         self.pub_path.publish(path_msg)
            #         self.pub_mpc_input.publish(mpc_input)

            #     init_time = time.time()
            #     while time.time() - init_time < 1.0:
            #         mpc_input.reference_speed = 0.0
            #         path_msg = Path()
            #         path_msg.poses.clear()
            #         path_msg.header = mpc_input.header
            #         mpc_input.poses = path_msg.poses
            #         self.pub_path.publish(path_msg)
            #         self.pub_mpc_input.publish(mpc_input)
            
            dx = self.goals[self.goal_count][0] - self.odom.position.x
            dy = self.goals[self.goal_count][1] - self.odom.position.y
            _,_,heading = euler_from_quaternion(self.odom.orientation.x,
                                                self.odom.orientation.y,
                                                self.odom.orientation.z,
                                                self.odom.orientation.w)
            # Calculate the error from current waypoint
            if np.sqrt(dx**2 + dy**2) < self.max_error:
                self.goal_count += 1
            
            goal_pose = PoseStamped()
            goal_pose.header = mpc_input.header
            goal_pose.pose.position.x = np.cos(heading)*dx + np.sin(heading)*dy
            goal_pose.pose.position.y = -np.sin(heading)*dx + np.cos(heading)*dy
            path_msg.poses.append(goal_pose)

        path_msg.header = mpc_input.header
        mpc_input.poses = path_msg.poses

        self.pub_path.publish(path_msg)
        self.pub_mpc_input.publish(mpc_input)

    def enableAutoModeCb(self, msg):
        if msg.data != self.auto_mode:
            rospy.loginfo("Navigation::enableAutoModeCb received " + str(msg.data))

            if msg.data == 1:
                self.goal_count = 0
        
        self.auto_mode = msg.data

    def mheCallback(self, msg):
        self.trav = msg.mu
        self.odom = msg.pose

        if msg.gps_accuracy <= self.min_acc:
            goals = []
            zero_lat = msg.zero_lat
            zero_lon = msg.zero_lon

            # Transform latitude/longitude to meters
            for i in range(len(self.goals_list)):
                lat_goal = self.goals_list[i][0]
                lon_goal = self.goals_list[i][1]
                x, y = dlatlon2dxy(zero_lat, zero_lon, lat_goal, lon_goal)
                goals.append([x, y])
            
            self.goals = goals

if __name__ == '__main__':
    rospy.init_node('point_navigation')
    POINT_NAVIGATION()