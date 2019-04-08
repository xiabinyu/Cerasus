//Class Head file
#include "cerasus_odom_class.cpp"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
class CerasusOdom{
public:
    CerasusOdom(tf::Transform _position);
    void EndOdom();

private:
    tf::Transform Odom;
    geometry_msgs::Vector3 Position;
    geometry_msgs::Vector3 Velocity;

};