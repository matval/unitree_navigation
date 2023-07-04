#!/usr/bin/env python3
import time
import numpy as np

import rospy
from fpn_msgs.msg import MHEOutput
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, PointStamped
from utils.mhe_utils import euler_from_quaternion, quaternion_from_euler, dlatlon2dxy

from mhe_vio.mhe import MHE_ESTIMATOR

class MHE_NODE:
    def __init__(self):
        mhe_configs = {}
        mhe_configs['rate'] = rospy.get_param('~mhe_rate', 10)
        mhe_configs['N']    = rospy.get_param('~mhe_N', 20)
        mhe_configs['Px']   = np.diag([1, 1, 1, 1, 1, 1])
        mhe_configs['Pp']   = np.diag([1, 1, 1, 1, 1])
        # mhe_configs['Pv']   = np.diag([1, 1, 1, 1, 1, 1])
        mhe_configs['Pvio'] = np.diag([1, 1, 1, 1, 1, 1])
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
        rospy.Subscriber("/unitree/zed2i/zed_node/pose_with_covariance", PoseWithCovarianceStamped, self.odom_callback, queue_size=1)
        rospy.Subscriber("/unitree/motion_command", TwistStamped, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/unitree/fix", NavSatFix, self.gnss_callback, queue_size=1)
        
        # Set publishers
        self.pub = rospy.Publisher('mhe_output', MHEOutput, queue_size=1)
        self.odom_pub = rospy.Publisher('mhe_odom', Odometry, queue_size=1)
        self.pub_point = rospy.Publisher('gps_point', PointStamped, queue_size=1)
    
        # Set node rate
        rate = rospy.Rate(mhe_configs['rate']) # ROS Rate at mhe_rate

        # Initialize data
        self.cmd_data       = [0.0, 0.0]
        self.gps_data       = [0.0, 0.0]
        self.odom_data      = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.gps_accuracy   = 10.0
        self.gps_new_data   = False

        # Initialize with unknown GPS position
        self.first_lat = -1.0
        self.first_lon = -1.0
        self.init_x = -1.0
        self.init_y = -1.0
        self.init_z = -1.0

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
                stamp = rospy.Time.now()

                mhe_odom = Odometry()
                mhe_odom.header.stamp = stamp
                mhe_odom.header.frame_id = 'map'
                mhe_odom.child_frame_id = 'base_link'
                mhe_odom.pose.pose.position.x = states[0]
                mhe_odom.pose.pose.position.y = states[1]
                mhe_odom.pose.pose.position.z = 0.0
                q = quaternion_from_euler(states[3],states[4],states[5])
                mhe_odom.pose.pose.orientation.w = q[0]
                mhe_odom.pose.pose.orientation.x = q[1]
                mhe_odom.pose.pose.orientation.y = q[2]
                mhe_odom.pose.pose.orientation.z = q[3]
                mhe_odom.twist.twist.linear.x = self.cmd_data[0]*mu
                mhe_odom.twist.twist.angular.z = self.cmd_data[1]*nu
                self.odom_pub.publish(mhe_odom)

                data_to_send = MHEOutput()
                data_to_send.header = mhe_odom.header
                data_to_send.mu = mu
                data_to_send.nu = nu
                data_to_send.delta_heading = theta_off
                data_to_send.gps_accuracy = self.gps_accuracy
                data_to_send.pose = mhe_odom.pose.pose
                data_to_send.twist = mhe_odom.twist.twist
                self.pub.publish(data_to_send)

                # # Publish the odom->map transformation
                # odom_trans = TransformStamped()
                # odom_trans.header.stamp = stamp
                # odom_trans.header.frame_id = "map"
                # odom_trans.child_frame_id = "odom"

                # odom_trans.transform.translation.x = custom.high_state.position[0];
                # odom_trans.transform.translation.y = custom.high_state.position[1];
                # odom_trans.transform.translation.z = 0.0
                # odom_trans.transform.rotation.w = custom.high_state.imu.quaternion[0];
                # odom_trans.transform.rotation.x = custom.high_state.imu.quaternion[1];
                # odom_trans.transform.rotation.y = custom.high_state.imu.quaternion[2];
                # odom_trans.transform.rotation.z = custom.high_state.imu.quaternion[3];
                # br.sendTransform(odom_trans)

            rate.sleep()

    def gnss_callback(self, gnss_msg):
        cur_time = rospy.get_time()
        setWithHighGnssAcc = False

        lat = gnss_msg.latitude
        lon = gnss_msg.longitude
        alt = gnss_msg.altitude
        gps_accuracy = (gnss_msg.position_covariance[0]+gnss_msg.position_covariance[5])**0.5

        if lat != 0.0 and lon != 0.0 and np.isfinite(lat) and np.isfinite(lon):
            # If first GPS measurement is good, set first lat/lon
            if (self.first_lat == -1.0 and self.first_lon == -1.0):
                self.first_lat = lat
                self.first_lon = lon
            
            gps_x, gps_y = dlatlon2dxy(self.first_lat, self.first_lon, lat, lon)

            if (self.init_x == -1.0 and self.init_y == -1.0):
                self.init_x = gps_x
                self.init_y = gps_y
                self.init_z = alt

            gps_point = PointStamped()
            gps_point.header.stamp = rospy.Time.now()
            gps_point.header.frame_id = 'map'
            gps_point.point.x = gps_x + self.init_x
            gps_point.point.y = gps_y + self.init_y
            gps_point.point.z = alt - self.init_z

            self.pub_point.publish(gps_point)

            # add new gnss reading to the buffer
            self.gps_data = [gps_point.point.x, gps_point.point.y]
            self.gps_accuracy = gps_accuracy
            self.gps_new_data = True

    def cmd_callback(self, msg):
        # add new command reading to the buffer
        self.cmd_data = [msg.twist.linear.x, msg.twist.angular.z]

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