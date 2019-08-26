#define CONST_LIDARMIN 0.1
#define CONST_LIDARMAX 1.0
#define CONST_ZERO_ANG 180
#define CONST_MAX_ANG 100
#define CONST_LANDMARK_GATE 0.5
#define CONST_ROAD_GATE 2.5
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include "cerasus_odom_class.h"
#include <cmath.h>
//#include "cerasus_odom_class.h"

struct Dot_a{
    double X;
    double Y;
};
class LandMark{
private:
    Dot_a *Dot;
    Dot_a LandMark_Dot;
    int DotNum;
    double SqrDis(Dot_a d1,Dot_a d2){
        return (d1.X-d2.X)*(d1.X-d2.X)+(d1.Y-d2.Y)*(d1.Y-d2.Y);
    }
    void Update(){
        LandMark_Dot.X+=Dot[DotNum].X;
        LandMark_Dot.Y+=Dot[DotNum].Y;
    }
public:
    LandMark(){
        Dot= new Dot_a(50);
        DotNum=0;
        LandMark_Dot={0,0};
    }
    int insertDot(Dot_a tdot){
        if(DotNum==0){
            Dot[DotNum]=tdot;
            DotNum++;
            Update();
            return 0;
        }
        if(SqrDis(Dot[0],tdot)<=CONST_LANDMARK_GATE*CONST_LANDMARK_GATE){
            Dot[DotNum]=tdot;
            DotNum++;
            Update();
            return 0;
        } else if(SqrDis(Dot[0],tdot)<=CONST_ROAD_GATE*CONST_ROAD_GATE){
            return 1;
        }
        return -1;
    }

};

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
    Dot_a Data[360];
    double* LandMark[180];
    int count=0;
    for(int i=0;i<360;i++){
        if(lidar.ranges[i]>=CONST_LIDARMIN&&lidar.ranges[i]<=CONST_LIDARMAX){
            Data[count].X=lidar.ranges[i]*;
        }
    }

}