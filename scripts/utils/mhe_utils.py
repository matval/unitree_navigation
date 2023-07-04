#!/usr/bin/env python3
import math
import numpy as np

EPS = 1e-9

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in first place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def normalize_angle(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi

def estimate_compass_offset(data_buffer):
    """
    This function estimates Compass offset from the data buffer.
    """
    lin_vel = np.asarray(data_buffer['cmd']['data'])[:,0]
    imu_heading = np.asarray(data_buffer['compass']['data'])
    # we can calculate GNSS heading using atan2
    gps_data = np.asarray(data_buffer['gnss']['data'])
    # check for unique values
    # gps_data, idxs = np.unique(gps_data, return_index=True, axis=0)
    gps_data2 = np.roll(gps_data, 1, axis=0)
    # get corresponding measurements from other sources
    # lin_vel = lin_vel[idxs]
    # imu_heading = imu_heading[idxs]
    # ignore the first value because of roll operation
    lin_vel = lin_vel[:-1]
    imu_heading = imu_heading[:-1]
    gps_heading = np.arctan2(gps_data[:,1]-gps_data2[:,1], gps_data[:,0]-gps_data2[:,0])[1:]
    gps_heading = gps_heading + (np.sign(lin_vel)-1)/2 * np.pi
    # and we calculate the angle offset 
    offset_array = normalize_angle(gps_heading - imu_heading)
    # the angle_offset is a weighted average where the weights are the abs value of the linear speed
    angle_offset = np.sum(np.abs(lin_vel) * offset_array) / (np.sum(np.abs(lin_vel)) + EPS)

    return angle_offset

def dlatlon2dxy(lat1, lon1, lat2, lon2):
    R = 6371000.0
    rlat1 = lat1*math.pi/180.0
    rlat2 = lat2*math.pi/180.0
    rlon1 = lon1*math.pi/180.0
    rlon2 = lon2*math.pi/180.0

    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1

    dx = R*dlon*math.cos((rlat1+rlat2)/2)
    dy = R*dlat
    return dx, dy

class FreqFilter:
    def __init__(self, init_value, cutoff_frequency, sampling_rate, window_size=5):
        assert cutoff_frequency <= sampling_rate
        self.alpha = cutoff_frequency/sampling_rate
        self.cutoff = cutoff_frequency
        self.sampling = sampling_rate
        self.min = min
        self.max = max
        self.last_output = init_value
        self.input_arr = []
        self.window_size = window_size

    def filter(self, input):
        # Use a median filter to remove outliers
        # First we sort the array
        if not np.isfinite(input):
            return self.last_output

        self.input_arr.append(input)
        if len(self.input_arr) > self.window_size:
            self.input_arr = self.input_arr[1:self.window_size+1]
        
        sorted_arr = sorted(self.input_arr)
        input = sorted_arr[int(len(sorted_arr)/2)]
        output = self.alpha*input + (1.0 - self.alpha)*self.last_output
        self.last_output = output

        return output