#define CONST_LIDARMIN 0.1
#define CONST_LIDARMAX 1.0
#define CONST_ZERO_ANG 180
#define CONST_MAX_ANG 100
#define CONST_LANDMARK_GATE 0.5
#define CONST_ROAD_GATE 2.5
#define CONST_ADD 1
#define CONST_RIGHT -1
#define CONST_LEFT 1
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include "cerasus_odom_class.h"
#include <cmath.h>
//#include "cerasus_odom_class.h"
static int Direction = 1;
struct Dot_a{
    double X;
    double Y;
};
class LandMark{
private:
    Dot_a *Dot;
    Dot_a LandMark_Dot;
    int DotNum;

    void Update(){
        LandMark_Dot.X+=Dot[DotNum].X;
        LandMark_Dot.Y+=Dot[DotNum].Y;
    }
public:
    double SqrDis(Dot_a d1,Dot_a d2=LandMark(0,0)){
        return (d1.X-d2.X)*(d1.X-d2.X)+(d1.Y-d2.Y)*(d1.Y-d2.Y);
    }
    LandMark(){
        Dot= new Dot_a(50);
        DotNum=0;
        LandMark_Dot={0,0};
    }
    LandMark(double x,double y){
        Dot= nullptr;
        DotNum=1;
        LandMark_Dot={x,y};
    }
    void getRF(LandMark L1,LandMark L2,int Direction,Dot_a* ptr){
        x1=L1.LandMark_Dot.X/L1.DotNum;
        y1=L1.LandMark_Dot.Y/L1.DotNum;
        x2=L2.LandMark_Dot.X/L2.DotNum;
        y2=L2.LandMark_Dot.Y/L2.DotNum;
        x0=(x1+x2)/2;
        y0=(y1+y2)/2;
        kt=-(x2-x1)/(y2-y1);
        xd=Direction*d/sqrt(1+kt*kt);
        ptr->X=x0+xd;
        ptr->Y=y0+xd*kt;
    }
    bool insertDot(Dot_a tdot){
        if(DotNum==0){
            Dot[DotNum]=tdot;
            DotNum++;
            Update();
            return false;
        }
        if(SqrDis(Dot[0],tdot)<=CONST_LANDMARK_GATE*CONST_LANDMARK_GATE){
            Dot[DotNum]=tdot;
            DotNum++;
            Update();
            return false;
        } else if(SqrDis(Dot[0],tdot)<=CONST_ROAD_GATE*CONST_ROAD_GATE){
            return true;
        }
        throw -1;
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
    Dot_a Data;
    LandMark* LM= new LandMark[20];
    Dot_a* RF= new Dot_a[20];
    int count=0;
    for(int i=0;i<360;i++){
        if(lidar.ranges[i]>=CONST_LIDARMIN&&lidar.ranges[i]<=CONST_LIDARMAX){
            Data.X=lidar.ranges[i]*cos(i/180*M_PI);
            Data.Y=lidar.ranges[i]*sin(i/180*M_PI);
            try {
                while (LM[count]->insertDot(Data)) {
                    count++;
                }
            }
            catch(int i){
                break;
            }
        }
    }
    for (int j = 0; j < count-1; ++j) {
        LandMark::getRF(LM[j],LM[j+1],Direction,RF+j);

    }

}