//Class Head file
#include "cerasus_odom_class.cpp"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
class CerasusOdom{
public:
    CerasusOdom(tf::Transform _position,float _cycle=0);
    tf::Transform OdomUpdate(sensor_msgs::Imu _imu);
    void EndOdom();

private:
    float Angle;
    geometry_msgs::Vector3 TransAc(double x,double y);
    tf::Transform Odom;
    geometry_msgs::Vector3 Position;
    geometry_msgs::Vector3 Velocity;
    geometry_msgs::Vector3 Acceleration;
    float CYCLE;

};