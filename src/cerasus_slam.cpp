#define CONST_LIDARMIN 0.050
#define CONST_LIDARMAX 1.2
#define CONST_MAX_ANG 100
#define CONST_LANDMARK_GATE 0.5
#define CONST_ROAD_GATE 2.50
#define CONST_ADD 1
#define CONST_RIGHT 1
#define CONST_LEFT -1
#define CONST_HF_ROAD 0.5
#define CONST_LM_RANGE 60
#define PI 3.14159265
#define MAXANG 25
#define MINANG -25
#define CONST_VEC 0.7
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
//#include "cerasus_odom_class.h"
static int Direction = CONST_RIGHT;
static double R0=0.45;
static int SSign=1;
static double R1=0.45;
static double ang=0;
static double vec=CONST_VEC;
static double LLdis=0.4;
static double LLLdis=0.52;
static int O_Direction=1;
static double i_vec=0;
static ros::Publisher cmd_pub;
static int ParkSign=0;
static double PNum=0;
static int AddVecSign=0;
static double PSum=0;
static int PPSign=0;
static int ParkStart=0;
void Callback(sensor_msgs::LaserScan scan);

void Callback2(std_msgs::Int32 dir);
ros::NodeHandle* nhp;
int main(int argc, char** argv){
    ros::init(argc, argv, "detect_target_slam");
    if (argc>1){
        LLLdis=0.4;
    }
    ros::NodeHandle nh;
    nhp=&nh;
    ros::Subscriber sub = nh.subscribe("/scan", 1, Callback);
    ros::Subscriber sub2 = nh.subscribe("/hw_dir", 1, Callback2);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::spin();
    return 0;
}

void Callback(sensor_msgs::LaserScan lidar) {
    if(ParkStart){

        vec=0.52;
//AddVecSign=0;
        static int countP=0;
        countP++;
        std::cout<<"p:"<<countP<<"||"<<ParkSign<<std::endl;
        if(ParkSign==1){
            if (countP < 0.4 / vec * 10)
                ang = 0;
            else if(countP > 0.40 / vec * 10 && countP < 10 / vec * 10)//2.8
            {

                vec=0.6;
                ang = 25;
                int countF=0;
                for(int j=1440-40;j<(1440+40);j++){
                    int i=j%1440;

                    if((lidar.ranges[i]<=0.50)&&(lidar.ranges[i]>=0.4)){
                        countF++;
                    }
                }
                if(countF>65){
                    ang = 0.0;
                    i_vec=0;
                }
            }else{
                ang = 0;
                i_vec=0;
            }
        }else{
            if (countP < 1.35 / vec * 10)
                ang = 0;
            else if(countP > 1.35 / vec * 10 && countP < 10 / vec * 10)//3.75
            {

                vec=0.6;
                ang = 25;
                int countF=0;
                for(int j=1440-40;j<(1440+40);j++){
                    int i=j%1440;

                    if((lidar.ranges[i]<=0.50)&&(lidar.ranges[i]>=0.4)){
                        countF++;
                    }
                }if(countF>65){
                    ang = 0;
                    i_vec=0;
                }
            }else{
                ang = 0;
                i_vec=0;
            }
        }
    }else{
        static int countT=0;
        countT++;
        bool signS=1;
        int countTTS;
        if((i_vec&&signS)||!signS){
            static int countTS=countT;
            countTTS=countTS;
            signS=0;
        }
        if(!signS&&(countT-countTTS)>=75&&(countT-countTTS)<=85){
            LLdis=LLLdis;
        }
        if(!signS&&(countT-countTTS)>=190&&(countT-countTTS)<=200){
            LLdis=0.4;
        }
        if(!signS&&(countT-countTTS)>=155&&(countT-countTTS)<=165){
            Direction=CONST_LEFT;
            LLdis=0.4;
        }

        if(!signS&&(countT-countTTS)>=280&&(countT-countTTS)<=286){
            PPSign=1;
//AddVecSign=1;
            vec=0.8;
        }
        std::cout<<"&|"<<countT-countTTS<<"|&";
        int count=0;

        int countInv=00;
        int countLM=0;
        double R[30]={0};
        double N[30]={0};
        int Num[100]={0};
        for(int i=29;i>=1;i--){
            R[i]=R[i-1];
            N[i]=N[i-1];
        }
        int countOb=0;
        for(int j=1440-360;j<(1440+360);j++){
            int i=j%1440;
            double tempR=lidar.ranges[i]*cos((i-1440+360)/720.0*3.1415);
            if((lidar.ranges[i]<=CONST_LIDARMAX)&&(tempR<=0.6)&&(tempR>=-0.6)){
                countOb++;
            }
        }
        if(countOb>=590){
            vec=0;
        }else{
            vec=CONST_VEC;
        }

        for(int j=1440-360-60;j<(1440-120);j++){
            int i;
            if(Direction==CONST_LEFT){
                i=1440-j;
            }else{
                i=j;
            }
            double tempR=lidar.ranges[i]*cos((i-1440+360)/720.0*3.1415);
            if((lidar.ranges[i]>=CONST_LIDARMIN)&&(lidar.ranges[i]<=CONST_LIDARMAX)&&(tempR<=0.65)&&(tempR>=-0.65)){
                R[0]+=tempR;
                N[0]++;
            }

            tempR=lidar.ranges[j]*cos((j-1440+360)/720.0*3.1415);
            if((lidar.ranges[j]>=CONST_LIDARMIN)&&(lidar.ranges[j]<=CONST_LIDARMAX)&&(tempR<=0.65)&&(tempR>=-0.65)){
                countInv++;
            }
        }
        if(ParkSign){
            Direction=CONST_LEFT;
            if(N[0]<=200){
                ParkStart=1;
            }
        }else{
            if(countInv<=150){
//Direction=-Direction;
            }
        }

        std::cout<<countOb<<"|||"<<N[0]<<"|"<<Direction<<std::endl;

        double k=400*Direction;
        double sR=0;
        double sN=0;
        for(int i=3;i>=0;i--){
            sR+=R[i]*Direction;
            sN+=N[i];
        }
        ang=k*1.0*(((LLdis)-(sR)/(sN)));//Direction==CONST_LEFT?0.6:
        //std::cout<<R0<<"||"<<R1<<"||"<<ang<<"\n";
    }


    double steer_angle = ang;//atan(2 * L * y / (x * x + y * y)) * 180 / PI;
    if(steer_angle > MAXANG)
        steer_angle = MAXANG;
    else if(steer_angle < MINANG)
        steer_angle = MINANG;
    else
        steer_angle = steer_angle;
    geometry_msgs::Twist twist;
    twist.linear.x = vec*i_vec;
    twist.linear.y = twist.linear.z = 00;
    twist.angular.x = twist.angular.y = 0;
    twist.angular.z = steer_angle / MAXANG * 1;
    cmd_pub.publish(twist);
}
void Callback2(std_msgs::Int32 dir){
    if(dir.data==1){
//i_vec=1;
    }else if(dir.data==2&&SSign){
        i_vec=1;
        SSign=0;
        Direction=CONST_LEFT;
    }else if(dir.data==3&&SSign){
        i_vec=1;

        SSign=0;
        Direction=CONST_RIGHT;
    }else if(dir.data==4&&PPSign){
//exit(-3);*
        PNum=PNum+1;
        PSum=PSum+1;
        ParkSign=(PSum/PNum<1.5?1:2);
    }else if(dir.data==5&&PPSign){

        PNum=PNum+1;
        PSum=PSum+2;
        ParkSign=(PSum/PNum<1.5?1:2);
//exit(-4);
    }
}
