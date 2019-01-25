//
// Created by ipp on 18-12-26.
//
// (What is going to do)This node take the imu data, map data as in put and out put the location of the car as tree of frame (tf).
// (What is it doing) For now, it just out put a fixed tf.
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>





int main(int argc, char** argv){
    ros::init(argc, argv, "ippcore_tf");

    ros::NodeHandle node;

    ros::Rate rate(2);
    while(ros::ok()) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));//设定坐标
        tf::Quaternion q;
        q.setRPY(0, 0, 0);//设定方向
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform,
                                              ros::Time::now(),
                                              "odom",
                                              "base_link"));
        ROS_INFO_STREAM("laser tf location:"<<0.0f<<" "<<0.0f<<" "<<0.0f<<" direction "<<0.0f<<" "<<0.0f<<" "<<0.0f);
        rate.sleep();
    }

    return 0;
};