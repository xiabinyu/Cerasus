// Class Source
//F(x,y)=(x*cos(a)-y*sin(a),x*sin(a)+y*cos(a))
#include "cerasus_odom_class.h"
#include <math.h>
CerasusOdom::CerasusOdom(tf::Transform _position,float _cycle){
    CYCLE=_cycle;
    Odom=_position;
};
CerasusOdom::CerasusOdom(){
    CYCLE=0.01f;
    Velocity.x=0;Velocity.y=0;Velocity.z=0;
    Position.x=0;Position.y=0;Position.z=0;
}
geometry_msgs::Vector3 CerasusOdom::TransAc(double x,double y){
    geometry_msgs::Vector3 NewAc;
    NewAc.x=x*cosf(Angle)-y*sinf(Angle);
    NewAc.y=x*sinf(Angle)+y*cosf(Angle);
    NewAc.z=0;
    return NewAc;
};
tf::Transform CerasusOdom::OdomUpdate(sensor_msgs::Imu _imu) {
    Angle+=_imu.angular_velocity.z*CYCLE;
    Acceleration=TransAc(_imu.linear_acceleration.x,_imu.linear_acceleration.y);
    Velocity.x+=Acceleration.x*CYCLE;
    Velocity.y+=Acceleration.y*CYCLE;
    Velocity.z+=Acceleration.z*CYCLE;
    Position.x+=Velocity.x;
    Position.y+=Velocity.y;
    Position.z+=Velocity.z;
    tf::Transform temp_transform;
    temp_transform.setOrigin(tf::Vector3(Position.x,Position.y,Position.z));//设定坐标
    tf::Quaternion temp_quaternion;
    temp_quaternion.setRPY(0, 0, Angle);//设定方向
    temp_transform.setRotation(temp_quaternion);
    return temp_transform;
}
geometry_msgs::Vector3 CerasusOdom::TransAc_New(std_msgs::Float64 _rpm){


    geometry_msgs::Vector3 NewAc;
    NewAc.x=0.0135*cosf(_rpm.data);
    NewAc.y=0.0135*sinf(_rpm.data);
    NewAc.z=0;
    return NewAc;
}
tf::Transform CerasusOdom::OdomUpdate_New(sensor_msgs::Imu _imu,std_msgs::Float64 _rpm){
    Angle+=_imu.angular_velocity.z*CYCLE;
    Velocity=TransAc_New(_rpm);
    Position.x+=Velocity.x*CYCLE;
    Position.y+=Velocity.y*CYCLE;
    Position.z+=Velocity.z*CYCLE;
    tf::Transform temp_transform;
    temp_transform.setOrigin(tf::Vector3(Position.x,Position.y,Position.z));//设定坐标
    tf::Quaternion temp_quaternion;
    temp_quaternion.setRPY(0, 0, Angle);//设定方向
    temp_transform.setRotation(temp_quaternion);
    return temp_transform;
}
void CerasusOdom::EndOdom(){};
