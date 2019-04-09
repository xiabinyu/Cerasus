//
// Created by xiabi on 2019/4/8.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include "cerasus_odom_class.h"

CerasusOdom ceo;
void Callback(const sensor_msgs::Imu imu);
int main(int argc, char** argv){
	ros::init(argc, argv, "cerasus_odom");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/imu", 1, Callback);
	ros::spin();
	return 0;
}

void Callback(const sensor_msgs::Imu imu){
	static tf::TransformBroadcaster br;
	tf::Transform transform=ceo.OdomUpdate(imu);
	br.sendTransform(tf::StampedTransform(transform,
										  ros::Time::now(),
										  "odom",
										  "base_link"));

}
