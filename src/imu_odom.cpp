// Credits to aatb-ch
// Code from: https://github.com/aatb-ch/go1_republisher

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <chrono>
#include <pthread.h>

using namespace UNITREE_LEGGED_SDK;
class Custom
{
	public:
		UDP high_udp;

		HighCmd high_cmd = {0};
		HighState high_state = {0};

	public:
		Custom():
			high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
			{
				high_udp.InitCmdData(high_cmd);
			}

			void highUdpSend()
			{
				// printf("high udp send is running\n");

				high_udp.SetSend(high_cmd);
				high_udp.Send();
			}

			void highUdpRecv()
			{
				int status = high_udp.Recv();
				high_udp.GetRecv(high_state);

				// printf("high udp recv is running %d\n", status);
				// printf("%i", high_state.gaitType);
			}
};

Custom custom;
ros::Publisher pub_imu;
ros::Publisher pub_odom;
ros::Publisher pub_cmd;
ros::Subscriber sub_cmd_vel;

void cmdCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    UNITREE_LEGGED_SDK::HighCmd cmd;

    custom.high_cmd.head[0] = 0xFE;
    custom.high_cmd.head[1] = 0xEF;
    custom.high_cmd.levelFlag = UNITREE_LEGGED_SDK::HIGHLEVEL;
    custom.high_cmd.mode = 0;
    custom.high_cmd.speedLevel = 0;
    custom.high_cmd.footRaiseHeight = 0;
    custom.high_cmd.bodyHeight = 0;
    custom.high_cmd.euler[0] = 0;
    custom.high_cmd.euler[1] = 0;
    custom.high_cmd.euler[2] = 0;
    custom.high_cmd.reserve = 0;
	// Commands
    custom.high_cmd.velocity[0] = msg->twist.linear.x;
    custom.high_cmd.velocity[1] = msg->twist.linear.y;
    custom.high_cmd.yawSpeed = msg->twist.angular.z;
	// Gaits
    custom.high_cmd.mode = 2;
    custom.high_cmd.gaitType = 1;

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "go1_imu");
	ros::NodeHandle nh;
	// Subscriber
	sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdCallback);
	//  Publishers
	pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);
	pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
	pub_cmd = nh.advertise<geometry_msgs::TwistStamped>("motion_command", 1);
	static tf2_ros::TransformBroadcaster br;

	LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
	LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

	loop_udpSend.start();
	loop_udpRecv.start();

	ros::Rate loop_rate(100);

	long count = 0;

	while (ros::ok())
	{
		if(custom.high_udp.udpState.RecvCount <= 0)
		{
			ROS_WARN("UDP not connected to the robot!");
			continue;
		}

		ros::Time current_time;

		current_time = ros::Time::now();
			
		sensor_msgs::Imu msg_imu;

		msg_imu.header.seq = count;
		msg_imu.header.stamp = current_time;
		msg_imu.header.frame_id = "imu_link";

		msg_imu.orientation.w = custom.high_state.imu.quaternion[0];
		msg_imu.orientation.x = custom.high_state.imu.quaternion[1];
		msg_imu.orientation.y = custom.high_state.imu.quaternion[2];
		msg_imu.orientation.z = custom.high_state.imu.quaternion[3];

		msg_imu.angular_velocity.x = custom.high_state.imu.gyroscope[0];
		msg_imu.angular_velocity.y = custom.high_state.imu.gyroscope[1];
		msg_imu.angular_velocity.z = custom.high_state.imu.gyroscope[2];

		msg_imu.linear_acceleration.x = custom.high_state.imu.accelerometer[0];
		msg_imu.linear_acceleration.y = custom.high_state.imu.accelerometer[1];
		msg_imu.linear_acceleration.z = custom.high_state.imu.accelerometer[2];

		pub_imu.publish(msg_imu);

		nav_msgs::Odometry msg_odom;

		msg_odom.header.seq = count;
		msg_odom.header.stamp = current_time;
		msg_odom.header.frame_id = "odom";
		msg_odom.child_frame_id = "base_link";
	
		msg_odom.pose.pose.position.x = custom.high_state.position[0];
		msg_odom.pose.pose.position.y = custom.high_state.position[1];
		msg_odom.pose.pose.position.z = custom.high_state.position[2];
		
		msg_odom.pose.pose.orientation.w = custom.high_state.imu.quaternion[0];
		msg_odom.pose.pose.orientation.x = custom.high_state.imu.quaternion[1];
		msg_odom.pose.pose.orientation.y = custom.high_state.imu.quaternion[2];
		msg_odom.pose.pose.orientation.z = custom.high_state.imu.quaternion[3];

		msg_odom.twist.twist.linear.x = custom.high_state.velocity[0];
		msg_odom.twist.twist.linear.y = custom.high_state.velocity[1];
		msg_odom.twist.twist.angular.z = custom.high_state.yawSpeed;

		pub_odom.publish(msg_odom);

		geometry_msgs::TwistStamped msg_cmd;

		msg_cmd.header.seq = count;
		msg_cmd.header.stamp = current_time;
		msg_cmd.header.frame_id = "base_link";

		msg_cmd.twist.linear.x = custom.high_state.velocity[0];
		msg_cmd.twist.linear.y = custom.high_state.velocity[1];
		msg_cmd.twist.angular.z = custom.high_state.yawSpeed;

		pub_cmd.publish(msg_cmd);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = custom.high_state.position[0];
		odom_trans.transform.translation.y = custom.high_state.position[1];
		odom_trans.transform.translation.z = custom.high_state.position[2];
		odom_trans.transform.rotation.w = custom.high_state.imu.quaternion[0];
		odom_trans.transform.rotation.x = custom.high_state.imu.quaternion[1];
		odom_trans.transform.rotation.y = custom.high_state.imu.quaternion[2];
		odom_trans.transform.rotation.z = custom.high_state.imu.quaternion[3];

		//send the transform
		br.sendTransform(odom_trans);

		ros::spinOnce();

		loop_rate.sleep();
  	}

    return 0;
}