#!/usr/bin/env python3
import subprocess
import signal
import os

import rospy
import rospkg
from std_msgs.msg import Bool
from std_msgs.msg import String

class BagRecorder:
    def __init__(self):
        rospy.init_node('bag_recorder_trigger')
        self.bag_process = None
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('unitree_navigation')
        self.launch_file_path = package_path + '/launch/rosbag_record.launch'

        # Subscriber
        self.subscriber = rospy.Subscriber('trigger_recording', Bool, self.callback)
        # Publisher
        self.status_pub = rospy.Publisher('recording_status', String, queue_size=10)

        self.prev_trigger = False
        self.status_message = String()

    def callback(self, msg):
        if self.prev_trigger != msg.data:
            if msg.data and self.bag_process is None:
                # Start recording using roslaunch
                rospy.loginfo("Starting to record rosbag via roslaunch...")
                self.bag_process = subprocess.Popen(['roslaunch', self.launch_file_path], preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
                self.status_message.data = "Recording rosbag."
            elif msg.data and self.bag_process is not None:
                # Stop recording
                rospy.loginfo("Stopping rosbag recording...")
                self.bag_process.send_signal(subprocess.signal.SIGINT)
                self.bag_process = None
                self.status_message.data = "Recording stopped."

        self.status_pub.publish(self.status_message)

        self.prev_trigger = msg.data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    recorder = BagRecorder()
    recorder.run()