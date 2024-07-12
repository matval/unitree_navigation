/* 	Credits to aatb-ch
 * 	Code adapted from: https://github.com/aatb-ch/go1_republisher
 */

#include <chrono>
#include <numeric>
#include <pthread.h>

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2_ros/transform_broadcaster.h>

// SDK includes
#include "unitree_legged_sdk/joystick.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

class Custom
{
	public:
		UDP high_udp;

		int motiontime = 0;

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
				high_udp.SetSend(high_cmd);
				high_udp.Send();
			}

			void highUdpRecv()
			{
				int status = high_udp.Recv();
				high_udp.GetRecv(high_state);
			}
};

Custom custom;

void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
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
    custom.high_cmd.velocity[0] = msg->linear.x;
    custom.high_cmd.velocity[1] = msg->linear.y;
    custom.high_cmd.yawSpeed = msg->angular.z;
	// Gaits
    custom.high_cmd.mode = 2;
    custom.high_cmd.gaitType = 1;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "unitree_state_control");
	ros::NodeHandle nh;

	// Subscriber
	ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdCallback);

	//  Publishers
	ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);
	ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
	ros::Publisher pub_rec = nh.advertise<std_msgs::Bool>("trigger_recording", 1);
	ros::Publisher pub_cmd = nh.advertise<geometry_msgs::TwistStamped>("motion_command", 1);
	ros::Publisher battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 10);
	// static tf2_ros::TransformBroadcaster br;

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

		ros::Time current_time = ros::Time::now();
		
		/*
		 * Publish IMU message
		 */
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

		/*
		 * Publish odometry message
		 */
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

		/*
		 * Publish commanded speed
		 */
		geometry_msgs::TwistStamped msg_cmd;
		// Get joystick data
		xRockerBtnDataStruct keyData;
		memcpy(&keyData, &custom.high_state.wirelessRemote[0], 40);

		msg_cmd.header.seq = count;
		msg_cmd.header.stamp = current_time;
		msg_cmd.header.frame_id = "base_link";

		if (keyData.ly > 0) {
			msg_cmd.twist.linear.x = 1.5*keyData.ly;
		}
		else {
			msg_cmd.twist.linear.x = 1.1*keyData.ly;
		}
		msg_cmd.twist.linear.y = keyData.lx;
		msg_cmd.twist.angular.z = -M_PI*keyData.rx;

		pub_cmd.publish(msg_cmd);

		/*
		 * Publish record signal
		 */
		std_msgs::Bool msg_rec;
		if (keyData.btn.components.L1 > 0 && keyData.btn.components.R1 > 0) {
			msg_rec.data = true;
		}
		else {
			msg_rec.data = false;
		}
		pub_rec.publish(msg_rec);

		/*
		 * Publish Battery message
		 */
        sensor_msgs::BatteryState battery_msg;
        // Populate the BatteryState message
        battery_msg.header.stamp = ros::Time::now();
		float total_voltage = 0.0;
		total_voltage = accumulate(std::begin(custom.high_state.bms.cell_vol), std::end(custom.high_state.bms.cell_vol), total_voltage);
        battery_msg.voltage = total_voltage / 1000.0; // Convert voltage from mV to V
        battery_msg.current = float(custom.high_state.bms.current) / 1000.0; // Convert current from mA to A
        battery_msg.charge = NAN; // No charge information, set to NaN
        battery_msg.capacity = NAN; // No capacity information, set to NaN
        battery_msg.design_capacity = NAN; // No design capacity, set to NaN
        battery_msg.percentage = float(custom.high_state.bms.SOC) / 100.0; // Convert SOC from percentage
        battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN; // Set according to your system status
        battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN; // Set according to your system health
        battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN; // Set according to your battery technology

        // Assuming BQ_NTC and MCU_NTC are temperature values, average them for example
        float average_temp = (custom.high_state.bms.BQ_NTC[0] + custom.high_state.bms.MCU_NTC[0]) / 2;
        battery_msg.temperature = average_temp;
		// Populate cell_voltage vector

        for (auto &vol : custom.high_state.bms.cell_vol) {
            battery_msg.cell_voltage.push_back(vol / 1000.0f); // Convert from mV to V
        }

        // Publish the BatteryState message
        battery_pub.publish(battery_msg);

		/*
		 * Publish TF transform
		 */
		// geometry_msgs::TransformStamped odom_trans;
		// odom_trans.header.stamp = current_time;
		// odom_trans.header.frame_id = "odom";
		// odom_trans.child_frame_id = "base_link";

		// odom_trans.transform.translation.x = custom.high_state.position[0];
		// odom_trans.transform.translation.y = custom.high_state.position[1];
		// odom_trans.transform.translation.z = custom.high_state.position[2];
		// odom_trans.transform.rotation.w = custom.high_state.imu.quaternion[0];
		// odom_trans.transform.rotation.x = custom.high_state.imu.quaternion[1];
		// odom_trans.transform.rotation.y = custom.high_state.imu.quaternion[2];
		// odom_trans.transform.rotation.z = custom.high_state.imu.quaternion[3];

		//send the transform
		// br.sendTransform(odom_trans);

		ros::spinOnce();
		loop_rate.sleep();
  	}

    return 0;
}
