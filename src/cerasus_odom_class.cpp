// Class Source
//F(x,y)=(x*cos(a)-y*sin(a),x*sin(a)+y*cos(a))
#include "cerasus_odom_class.h"
#include <math.h>
CerasusOdom::CerasusOdom(tf::Transform _position,float _cycle=0){
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
    Angle+=_imu.angular_velocity.z;
    Acceleration=TransAc(_imu.linear_acceleration.x,_imu.linear_acceleration.y);
    Velocity.x+=Acceleration.x;
    Velocity.y+=Acceleration.y;
    Velocity.z+=Acceleration.z;
    Position.x+=Velocity.x;
    Position.y+=Velocity.y;
    Position.z+=Velocity.z;
}
void CerasusOdom::EndOdom(){};