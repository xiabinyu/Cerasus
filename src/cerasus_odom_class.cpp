// Class Source
//F(x,y)=(x*cos(a)-y*sin(a),x*sin(a)+y*cos(a))
#include "cerasus_odom_class.h"
#include <math.h>
CerasusOdom::CerasusOdom(tf::Transform _position,float _cycle=0.01){
    CYCLE=_cycle;
    Odom=_position;
};

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
void CerasusOdom::EndOdom(){};