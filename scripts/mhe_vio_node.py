#!/usr/bin/env python3
import time
import numpy as np

import rospy
from fpn_msgs.msg import MHEOutput
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from utils.mhe_utils import euler_from_quaternion, quaternion_from_euler

from mhe_odom.mhe import MHE_ESTIMATOR

class MHE_NODE:
    def __init__(self):
        mhe_configs = {}
        mhe_configs['rate'] = rospy.get_param('~mhe_rate', 10)
        mhe_configs['N']    = rospy.get_param('~mhe_N', 16)
        mhe_configs['Px']   = np.diag([1, 1, 1, 1, 1, 1])
        mhe_configs['Pp']   = np.diag([0.1, 0.1, 1, 1, 1])
        # mhe_configs['Pv']   = np.diag([1, 1, 1, 1, 1, 1])
        mhe_configs['Pvio'] = np.diag([10, 10, 10, 10, 10, 10])
        mhe_configs['Pgps'] = np.diag([1, 1])
        mhe_configs['min_acc'] = 5.0

        # Set GPS antenna offset
        mhe_configs['d_gps_x'] =  0.0       # GPS offset in the X axis
        mhe_configs['d_gps_y'] =  0.0       # GPS offset in the Y axis
        mhe_configs['d_gps_z'] =  0.215     # GPS offset in the Z axis

        # Create MHE object
        mhe = MHE_ESTIMATOR(mhe_configs)

        # Set subscribers
        #rospy.Subscriber("/terrasentia/zed2/zed_node/odom", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/terrasentia/zed2/zed_node/pose_with_covariance", PoseWithCovarianceStamped, self.odom_callback, queue_size=1)
        rospy.Subscriber("/terrasentia/motion_command", Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/unitree/fix", GPSPosition, self.gnss_callback, queue_size=1)
        
        # Set publishers
        self.pub = rospy.Publisher('mhe_output', MHEOutput, queue_size=1)
        self.odom_pub = rospy.Publisher('mhe_odom', Odometry, queue_size=1)
    
        # Set node rate
        rate = rospy.Rate(mhe_configs['rate']) # ROS Rate at mhe_rate

        # Initialize data
        self.cmd_data       = [0.0, 0.0]
        self.gps_data       = [0.0, 0.0]
        self.odom_data      = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gps_accuracy   = 10.0
        self.gps_new_data   = False

        # Create dictionaries to organize the data
        cmd_dict = {'stamp': np.array([]), 'data': []}
        gps_dict = {'stamp': np.array([]), 'data': [], 'available': []}
        odom_dict = {'stamp': np.array([]), 'data': []}
        horizon_data = {
            'cmd': cmd_dict,
            'gnss': gps_dict,
            'odom': odom_dict}

        while not rospy.is_shutdown():
            # Add new values to the horizon
            horizon_data['cmd']['data'].append(self.cmd_data)
            horizon_data['odom']['data'].append(self.odom_data)
            horizon_data['gnss']['data'].append(self.gps_data)
            if self.gps_new_data and (self.gps_accuracy <= mhe_configs['min_acc']):
                horizon_data['gnss']['available'].append(1)
                self.gps_new_data = False
            else:
                horizon_data['gnss']['available'].append(0)

            # Remove old values
            if len(horizon_data['cmd']['data']) > mhe_configs['N']:
                horizon_data['cmd']['data'].pop(0)
                horizon_data['odom']['data'].pop(0)
                horizon_data['gnss']['data'].pop(0)
                horizon_data['gnss']['available'].pop(0)
            
            # Check if data buffer has the necessary length to run MHE
            if len(horizon_data['gnss']['data']) == mhe_configs['N']:
                #print('roll: %.3f, pitch: %.3f, yaw: %.3f' % (self.odom_data[3], self.odom_data[4], self.odom_data[5]))
                #print('GPS availability:', horizon_data['gnss']['available'])

                start = time.time()
                states, mu, nu, x_off, y_off, theta_off = mhe.run_mhe(horizon_data)
                time_ellapsed = time.time()-start
                
                if time_ellapsed > 0.2:
                    print('run_mhe time:', time_ellapsed)

                # Let's correct the states
                corr_x = states[0]*np.cos(theta_off) - states[1]*np.sin(theta_off) + x_off
                corr_y = states[0]*np.sin(theta_off) + states[1]*np.cos(theta_off) + y_off
                states[0] = corr_x
                states[1] = corr_y
                states[5] = states[5] + theta_off

                # Publish data
                data_to_send = MHEOutput()
                data_to_send.header.stamp = rospy.Time.now()
                data_to_send.mu = mu
                data_to_send.nu = nu
                data_to_send.delta_heading = theta_off
                data_to_send.gps_accuracy = self.gps_accuracy
                data_to_send.position.x = states[0]
                data_to_send.position.y = states[1]
                data_to_send.position.z = states[2]
                data_to_send.heading = states[5]
                self.pub.publish(data_to_send)

                mhe_odom = Odometry()
                mhe_odom.header.stamp = rospy.Time.now()
                mhe_odom.header.frame_id = 'map'
                mhe_odom.child_frame_id = 'base_link'
                mhe_odom.pose.pose.position = data_to_send.position
                q = quaternion_from_euler(states[3],states[4],states[5])
                mhe_odom.pose.pose.orientation.w = q[0]
                mhe_odom.pose.pose.orientation.x = q[1]
                mhe_odom.pose.pose.orientation.y = q[2]
                mhe_odom.pose.pose.orientation.z = q[3]
                self.odom_pub.publish(mhe_odom)

            rate.sleep()

    def gnss_callback(self, msg):
        # add new gnss reading to the buffer
        self.gps_data = [msg.position.x, msg.position.y]
        self.gps_accuracy = msg.gps_accuracy
        self.gps_new_data = True

    def cmd_callback(self, msg):
        # add new command reading to the buffer
        self.cmd_data = [msg.linear.x, -msg.angular.z]

    def odom_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        
        # add new odometry reading to the buffer
        self.odom_data = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            roll,
            pitch,
            yaw]

if __name__ == '__main__':
    rospy.init_node('mhe_estimator')
    MHE_NODE()