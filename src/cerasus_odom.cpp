//
// Created by xiabi on 2019/4/8.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include "cerasus_odom_class.h"

CerasusOdom ceo;
sensor_msgs::Imu myimu;
std_msgs::Float64 myrpm;
bool sign=0;
bool sign0=0;
bool sign1=0;
void Callback(sensor_msgs::Imu imu);
void realcall();
void Callback2(std_msgs::Float64 _rpm);
int main(int argc, char** argv){
	ros::init(argc, argv, "cerasus_odom");
    //ROS_INFO_STREAM("1");
	ros::NodeHandle nh;
    //ROS_INFO_STREAM("2");
	ros::Subscriber sub = nh.subscribe("/imu", 1, Callback);

    ros::Subscriber sub2 = nh.subscribe("/motor/rpm", 1, Callback2);
	ros::spin();
	return 0;
}

void Callback(sensor_msgs::Imu imu){
    //ROS_INFO_STREAM("3");
    myimu=imu;

    tf::Transform transform = ceo.OdomUpdate_New(myimu);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform,
                                          ros::Time::now(),
                                          "odom",
					"laser"
                                          ));
}

void Callback2(std_msgs::Float64 _rpm){
    myrpm=_rpm;


    tf::TransformBroadcaster br;
    tf::Transform transform = ceo.OdomUpdate_New(myrpm);      }
    //br.sendTransform(tf::StampedTransform(transform,
      //                                    ros::Time::now(),
        //                                  "odom",
          //                                "base_link"));
//}
//
//void realcall() {
//    static int t = 0;
//    if (t < 2) {
//        if (t == 0) {
//            ROS_INFO_STREAM("Initializing... ... Please leave the imu still");
//        }
//        ceo.OdomInit(myimu);
//    } else if (t == 200) {
//        ceo.ReadInit();
//    } else {
//
//        tf::Transform transform = ceo.OdomUpdate_New(myimu,myrpm);
//        br.sendTransform(tf::StampedTransform(transform,
//                                              ros::Time::now(),
//                                              "odom",
//                                              "base_link"));
//    }
//    t++;
//}
