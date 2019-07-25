//
// Created by xiabinyu on 7/16/19.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include "cerasus_odom_class.h"
//#include "cerasus_odom_class.h"


void Callback(sensor_msgs::LaserScan scan);

ros::NodeHandle* nhp;
int main(int argc, char** argv){

    ros::init(argc, argv, "cerasus_slam");

    ros::NodeHandle nh;
    nhp=&nh;
	ros::Subscriber sub = nh.subscribe("/scan", 1, Callback);
	ros::spin();
	return 0;
}

void Callback(sensor_msgs::LaserScan scan) {

    scan.header.stamp=ros::Time::now();
    static ros::Publisher root_pub = nhp->advertise<sensor_msgs::LaserScan>("/laser_scan", 1000);//Root Command
    root_pub.publish(scan);
    //int Total=2*M_PI/scan.angle_increment;
  //  int Total=360;
  //  float z[720];
  //  //int z_size[2]={2,Total};
  //  for (unsigned int i=0;i<Total;i++){
  //      z[0+(i<<1u)]=scan.ranges[i];
  //      z[1+(i<<1u)]=i*scan.angle_increment;
  //  }
  //
}