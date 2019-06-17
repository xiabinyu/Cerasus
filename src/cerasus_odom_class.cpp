// Class Source
//F(x,y)=(x*cos(a)-y*sin(a),x*sin(a)+y*cos(a))
#include "cerasus_odom_class.h"
#include <ros/ros.h>
#include <math.h>
CerasusOdom::CerasusOdom(const tf::Transform _position,const float _cycle){
    CYCLE=_cycle;
    Odom=_position;
    Angle=0;
};
CerasusOdom::CerasusOdom(){
    CYCLE=0.01f;
    Velocity.x=0;Velocity.y=0;Velocity.z=0;
    Position.x=0;Position.y=0;Position.z=0;
    Angle=0;
    t=0;
}
void CerasusOdom::NewCycle(float _cycle) {
    CYCLE=_cycle;
}
geometry_msgs::Vector3 CerasusOdom::TransAc(double x,double y){
    geometry_msgs::Vector3 NewAc;
    NewAc.x=x*cosf(Angle)-y*sinf(Angle);
    NewAc.y=x*sinf(Angle)+y*cosf(Angle);
    NewAc.z=0;
    return NewAc;
};

void CerasusOdom::OdomInit(const sensor_msgs::Imu _imu){
    Acceleration_Avg.x=(Acceleration_Avg.x*inittime+_imu.linear_acceleration.x)/(inittime+1);
    Acceleration_Avg.y=(Acceleration_Avg.y*inittime+_imu.linear_acceleration.y)/(inittime+1);
    Acceleration_Avg.z=(Acceleration_Avg.z*inittime+_imu.linear_acceleration.z)/(inittime+1);
    z_Avg=(z_Avg*inittime+_imu.angular_velocity.z)/(inittime+1);
    inittime++;
}
void CerasusOdom::ReadInit() {
    ROS_INFO_STREAM("**********Initialized:*********");
    ROS_INFO_STREAM("Ac.x:"<<Acceleration_Avg.x);
    ROS_INFO_STREAM("Ac.y:"<<Acceleration_Avg.y);
    ROS_INFO_STREAM("Ac.z:"<<Acceleration_Avg.z);
    ROS_INFO_STREAM("z:"<<z_Avg);

}
tf::Transform CerasusOdom::OdomUpdate(const sensor_msgs::Imu _imu) {
    Angle+=(_imu.angular_velocity.z-z_Avg)*CYCLE*2;//Double it during the Unit Reason
    //Angle=_imu.orientation.w

    Acceleration.x=_imu.linear_acceleration.x-Acceleration_Avg.x;
    Velocity.x+=Acceleration.x*CYCLE;
    geometry_msgs::Vector3 tVelocity=TransAc(Velocity.x,0);
    //Velocity.x+=Acceleration.x*CYCLE;
    //Velocity.y+=Acceleration.y*CYCLE;
    //Velocity.z+=Acceleration.z*CYCLE;
    //Position.x+=tVelocity.x*CYCLE;
    //Position.y+=tVelocity.y*CYCLE;
    tf::Transform temp_transform;
    temp_transform.setOrigin(tf::Vector3(Position.x,Position.y,Position.z));//设定坐标
    tf::Quaternion temp_quaternion;
    temp_quaternion.setRPY(0, 0, Angle);//设定方向
    temp_transform.setRotation(temp_quaternion);
    LastTime=ros::Time::now().toSec();
    return temp_transform;
}
geometry_msgs::Vector3 CerasusOdom::TransAc_New(const std_msgs::Float64 _rpm){


    geometry_msgs::Vector3 NewAc;
    NewAc.x=0.0135*cosf(_rpm.data);
    NewAc.y=0.0135*sinf(_rpm.data);
    NewAc.z=0;
    LastTime=ros::Time::now().toSec();
    return NewAc;
}
tf::Transform CerasusOdom::OdomUpdate_New(const sensor_msgs::Imu _imu,const std_msgs::Float64 _rpm){
    CYCLE=(ros::Time::now().toSec()-LastTime);
    Angle+=(_imu.angular_velocity.z-z_Avg)*CYCLE*2;//Double it during the Unit Reason
    //Angle=_imu.orientation.w

    Acceleration.x=_imu.linear_acceleration.x-Acceleration_Avg.x;
    Velocity.x+=0.0135*_rpm.data*CYCLE;
    geometry_msgs::Vector3 tVelocity=TransAc(Velocity.x,0);
    //Velocity.x+=Acceleration.x*CYCLE;
    //Velocity.y+=Acceleration.y*CYCLE;
    //Velocity.z+=Acceleration.z*CYCLE;
    //Position.x+=tVelocity.x*CYCLE;
    //Position.y+=tVelocity.y*CYCLE;
    tf::Transform temp_transform;
    temp_transform.setOrigin(tf::Vector3(Position.x,Position.y,Position.z));//设定坐标
    tf::Quaternion temp_quaternion;
    temp_quaternion.setRPY(0, 0, Angle);//设定方向
    temp_transform.setRotation(temp_quaternion);
    LastTime=ros::Time::now().toSec();
    return temp_transform;
    //Angle+=_imu.angular_velocity.z*CYCLE;
    //Velocity=TransAc_New(_rpm);
    //Position.x+=Velocity.x*CYCLE;
    //Position.y+=Velocity.y*CYCLE;
    //Position.z+=Velocity.z*CYCLE;
    //tf::Transform temp_transform;
    //temp_transform.setOrigin(tf::Vector3(Position.x,Position.y,Position.z));//设定坐标
    //tf::Quaternion temp_quaternion;
    //temp_quaternion.setRPY(0, 0, Angle);//设定方向
    //temp_transform.setRotation(temp_quaternion);
    //return temp_transform;
}
void CerasusOdom::EndOdom(){};
