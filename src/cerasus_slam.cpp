#define LIDARMIN 0.050
#define LIDARMAX 1.2
#define MAX_ANG 100
#define LANDMARK_GATE 0.5
#define ROAD_GATE 2.50
#define ADD 1
#define RIGHT 1
#define LEFT -1
#define HF_ROAD 0.5
#define LM_RANGE 60
#define PI 3.14159265
#define MAXANG 25
#define MINANG -25
#define VEC 0.7

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>

static int Direction = RIGHT;//Set default direction.
static double ang=0;//Steering Angle.
static double vec=VEC;//Velocity.
static double LLdis=0.4;//The distance to the side.
static double LLLdis=0.52;//The larger distance to the side.
static int i_vec=0;//Decides whether to move
// Park Signs:PPSign -> ParkStart -> ParkSign
static int PPSign=0;//Prepare to find the park sign: After this sign, car will look for the park sign.
static int ParkStart=0;//Found the park sign: Ready to park any time.
static int ParkSign=0;//The lidar found the sign that the car is near the park and
// then the park program will take care of the rest.

static ros::Publisher cmd_pub;//Preset for ROS.
static ros::NodeHandle* nhp;//Preset for ROS.

void Callback(sensor_msgs::LaserScan scan);
void Callback2(std_msgs::Int32 dir);

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
        static bool signS=0;
        int countTTS;
        if(i_vec||signS){
            static int countTS=countT;
            countTTS=countTS;
            signS=1;
        }
        if(!signS&&(countT-countTTS)>=75&&(countT-countTTS)<=85){
            LLdis=LLLdis;
        }
        if(!signS&&(countT-countTTS)>=190&&(countT-countTTS)<=200){
            LLdis=0.4;
        }
        if(!signS&&(countT-countTTS)>=155&&(countT-countTTS)<=165){
            Direction=LEFT;
            LLdis=0.4;
        }

        if(!signS&&(countT-countTTS)>=280&&(countT-countTTS)<=286){
            PPSign=1;
            vec=0.8;
        }
        std::cout<<"&|"<<countT-countTTS<<"|&";


        double R[4]={0};
        double N[4]={0};
        for(int i=3;i>=1;i--){
            R[i]=R[i-1];
            N[i]=N[i-1];
        }

        int countOb=0;//count Obstacle
        for(int j=1440-360;j<(1440+360);j++){
            int i=j%1440;
            double tempR=lidar.ranges[i]*cos((i-1440+360)/720.0*3.1415);
            if((lidar.ranges[i]<=LIDARMAX)&&(tempR<=0.6)&&(tempR>=-0.6)){
                countOb++;
            }
        }
        if(countOb>=590){
            vec=0;
        }else{
            vec=VEC;
        }

        for(int j=1440-360-60;j<(1440-120);j++){
            int i;
            if(Direction==LEFT){
                i=1440-j;
            }else{
                i=j;
            }
            double tempR=lidar.ranges[i]*cos((i-1440+360)/720.0*3.1415);
            if((lidar.ranges[i]>=LIDARMIN)&&(lidar.ranges[i]<=LIDARMAX)&&(tempR<=0.65)&&(tempR>=-0.65)){
                R[0]+=tempR;
                N[0]++;
            }


        }
        if(ParkSign){
            Direction=LEFT;
            if(N[0]<=200){
                ParkStart=1;
            }
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
    ang=k*1.0*(((LLdis)-(sR)/(sN)));
}


double steer_angle = ang;//atan(2 * L * y / (x * x + y * y)) * 180 / PI;
if(steer_angle > MAXANG)steer_angle = MAXANG;
else if(steer_angle < MINANG)steer_angle = MINANG;
else steer_angle = steer_angle;

geometry_msgs::Twist twist;
twist.linear.x = vec*i_vec;
twist.linear.y = twist.linear.z = 00;
twist.angular.x = twist.angular.y = 0;
twist.angular.z = steer_angle / MAXANG * 1;
cmd_pub.publish(twist);
}
void Callback2(std_msgs::Int32 dir){
    static double PNum=0;
    static double PSum=0;
    static int SSign=1;//Ensure the car only start once.
    if(dir.data==1){
        //Simply do nothing.
    }else if(dir.data==2&&SSign){
        i_vec=1;
        SSign=0;
        Direction=LEFT;
    }else if(dir.data==3&&SSign){
        i_vec=1;
        SSign=0;
        Direction=RIGHT;
    }else if(dir.data==4&&PPSign){
        PNum=PNum+1;
        PSum=PSum+1;
        ParkSign=(PSum/PNum<1.5?1:2);
    }else if(dir.data==5&&PPSign){
        PNum=PNum+1;
        PSum=PSum+2;
        ParkSign=(PSum/PNum<1.5?1:2);
    }
}
