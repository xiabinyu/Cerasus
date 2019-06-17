//
// Created by xiabi on 2019/4/8.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include "cerasus_odom_class.h"

CerasusOdom ceo;
void Callback(sensor_msgs::Imu imu);

void Callback(sensor_msgs::Imu imu,std_msgs::Float64 _rpm);
int main(int argc, char** argv){
	ros::init(argc, argv, "cerasus_odom");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/imu", 1, Callback);
	ros::spin();
	return 0;
}

void Callback(sensor_msgs::Imu imu){
	static tf::TransformBroadcaster br;
	static int t=0;
	if(t<200) {
		if (t = 0) {
			ROS_INFO_STREAM("Initializing……Please leave the imu still");
		}
		ceo.OdomInit(imu);
	}else if(t=200) {
		ceo.ReadInit();
	}else{

		tf::Transform transform = ceo.OdomUpdate(imu);
		br.sendTransform(tf::StampedTransform(transform,
											  ros::Time::now(),
											  "odom",
											  "base_link"));
	}
	t++;

}

void Callback(sensor_msgs::Imu imu,std_msgs::Float64 _rpm){

}