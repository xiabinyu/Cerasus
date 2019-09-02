#define CONST_LIDARMIN 0.1
#define CONST_LIDARMAX 5.0
#define CONST_ZERO_ANG 180
#define CONST_MAX_ANG 100
#define CONST_LANDMARK_GATE 0.5
#define CONST_ROAD_GATE 2.5
#define CONST_ADD 1
#define CONST_RIGHT 1
#define CONST_LEFT -1

#define CONST_LM_RANGE 20
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include "cerasus_odom_class.h"
#include <math.h>
//#include "cerasus_odom_class.h"
static int Direction = CONST_RIGHT;
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
        Dot= 0;
        DotNum=1;
        LandMark_Dot={x,y};
    }
    void getRF(LandMark L1,LandMark L2,int Direction,Dot_a* ptr){
        double x1=L1.LandMark_Dot.X/L1.DotNum;
        double y1=L1.LandMark_Dot.Y/L1.DotNum;
        double x2=L2.LandMark_Dot.X/L2.DotNum;
        double y2=L2.LandMark_Dot.Y/L2.DotNum;
        double x0=(x1+x2)/2;
        double y0=(y1+y2)/2;
        double kt=-(x2-x1)/(y2-y1);
        double xd=Direction*CONST_HF_ROAD/sqrt(1+kt*kt);
        ptr->X=x0+xd;
        ptr->Y=y0+xd*kt;
    }/*
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
    */
    bool insertDot(Dot_a tdot){
            Dot[DotNum]=tdot;
            DotNum++;
            Update();
    }

};

void Callback(sensor_msgs::LaserScan scan);

void Callback2(std_msgs::Int16 dir);
ros::NodeHandle* nhp;
int main(int argc, char** argv){

    ros::init(argc, argv, "cerasus_slam");

    ros::NodeHandle nh;
    nhp=&nh;
	ros::Subscriber sub = nh.subscribe("/scan", 1, Callback);
    ros::Subscriber sub2 = nh.subscribe("/hw_dir", 1, Callback2);
	ros::spin();
	return 0;
}
/*
void Callback(sensor_msgs::LaserScan lidar) {
    Dot_a Data;
    LandMark* LM= new LandMark[20];
    Dot_a* RF= new Dot_a[20];
    int count=0;
    for(int j=0;j<1440;j++){
        if(Direction==CONST_LEFT){
            i=1440-j;
        }else{
            i=j;
        }
        if(lidar.ranges[i]>=CONST_LIDARMIN&&lidar.ranges[i]<=CONST_LIDARMAX){
            Data.X=lidar.ranges[i]*sin(i/4/180*M_PI);
            Data.Y=-lidar.ranges[i]*cos(i/4/180*M_PI);
            try {
                while (LM[count].insertDot(Data)) {
                    count++;
                }
            }
            catch(int i){
                break;
            }
        }
    }
    //for (int j = 0; j < count-1; ++j) {
    j=3;
    LandMark::getRF(LM[j],LM[j+1],Direction,RF+j);

    //}

}
*/
void Callback(sensor_msgs::LaserScan lidar) {
    Dot_a Data;
    LandMark* LM= new LandMark[20];
    Dot_a* RF= new Dot_a[20];
    int count=0;
    int countL=0;
    for(int j=0;j<1440;j++){
        if(Direction==CONST_LEFT){
            i=1440-j;
        }else{
            i=j;
        }
        if(lidar.ranges[i]>=CONST_LIDARMIN&&lidar.ranges[i]<=CONST_LIDARMAX){
            Data.X=lidar.ranges[i]*sin(i/4/180*M_PI);
            Data.Y=-lidar.ranges[i]*cos(i/4/180*M_PI);
            LM[count].insertDot(Data);
        }
        if(countL<CONST_LM_RANGE){
            countL++;
        } else{
            countL=0;
            count++;
        }
    }
    //for (int j = 0; j < count-1; ++j) {
    j=6;
    Dot_a* temp_RF= new Dot_a;
    LandMark::getRF(LM[j],LM[j+1],Direction,temp_RF);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(temp_RF->X,temp_RF->Y,0));
    tf::Transform transform = ceo.OdomUpdate_New(myimu);

    static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transform,
                                          ros::Time::now(),
                                          "odom",
                                          "laser"
    ));
    //}

}
void Callback2(std_msgs::Int16 dir){
    Direction=dir.data;
}