//This node send a ros command to the system.
//If you need to set up a new command, please add here.
// Created by ipp on 19-1-2.
//

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
using namespace std;
int main(int argc, char *argv[]){
    ros::init(argc,argv,"cerasus_root");
    ros::NodeHandle nh;



    ros::Publisher root_pub = nh.advertise<std_msgs::Int16>("Cerasus/root", 1000);//Root Command

    ros::Publisher PWM_pub = nh.advertise<std_msgs::Float32>("Cerasus/PWM", 1000);//PWM Information

    /*Command Table
     * Command Name |Argument|Number Code
     * Shutdown     |-s      |-1
     * Start        |-start  | 0
     * Close        |-c      | 1
     * Self check   |-sfck   | 2
     * Help         |-h      | /
     * PWM          |-P      | 3
     */
    if (argc>2){
        ROS_FATAL_STREAM("Too many arguments!");
        ros::shutdown();
    }
    short int CCode;
    string shutdown="-s";//-1
    string start="-start";//0
    string close="-c";//1
    string self_check="-sfck";//2
    string help="-h";
    string _PWM="-P";
    if (argc == 1){
        CCode=0;
    }else if (argv[1]==shutdown){
        CCode=-1;
    }else if (argv[1]==start){
        CCode=0;
    }else if (argv[1]==close){
        CCode=1;
    }else if (argv[1]==self_check){
        CCode=2;
    }else if (argv[1]==_PWM){
        CCode=3;
    }else if (argv[1]==help){
        ROS_INFO_STREAM("Command Name |Argument|Number Code");
        ROS_INFO_STREAM("Shutdown     |-s      |-1         ");
        ROS_INFO_STREAM("Start        |-start  | 0         ");
        ROS_INFO_STREAM("Close        |-c      | 1         ");
        ROS_INFO_STREAM("Self check   |-sfck   | 2         ");
        ROS_INFO_STREAM("Help         |-h      | /         ");
        ROS_INFO_STREAM("PWM          |-P      | 3         ");
        ros::shutdown();
    }else {
        ROS_WARN_STREAM("Unknown Command!");
    }
    ROS_INFO_STREAM("Sending Command...Code:"<<CCode);
    ros::Rate loop_rate(10);
    if (CCode==3){//This part publish a Duty Cycle which can be subscribed by the cerasus_pwm_node
        char quit;
        while (ros::ok()){
            cout<<"Input a DutyCycle (0-100),(-1 to quit):";
            std_msgs::Float32 msg;
            cin>>msg.data;
            if (msg.data>=0 && msg.data<=100) {
                PWM_pub.publish(msg);
            }else{
                ros::shutdown();
            }
        }
    }else {
        while (ros::ok()) {
            std_msgs::Int16 msg;
            msg.data = CCode;
            root_pub.publish(msg);
        }
    }
}
