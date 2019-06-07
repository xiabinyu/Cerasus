//Class Head file
#ifndef CERASUS_ODOM_CLASS
#define CERASUS_ODOM_CLASS

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Float64.h>
class CerasusOdom{
public:
    CerasusOdom(tf::Transform _position,float _cycle);
    CerasusOdom();
    tf::Transform OdomUpdate(sensor_msgs::Imu _imu);
    tf::Transform OdomUpdate_New(sensor_msgs::Imu _imu,std_msgs::Float64 _rpm);
    void EndOdom();
private:
    float Angle;
    geometry_msgs::Vector3 TransAc(double x,double y);
    tf::Transform Odom;

    geometry_msgs::Vector3 TransAc_New(std_msgs::Float64 _rpm);
    geometry_msgs::Vector3 Position;
    geometry_msgs::Vector3 Velocity;
    geometry_msgs::Vector3 Acceleration;
    float CYCLE;

};

#endif
