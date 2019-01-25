// This node take data from imu and publish it as two topics in ros.
// Most of the code is taken from the website:  https://blog.csdn.net/qq_16583687/article/details/55259778 {No license mentioned}
// Where I have changed the message type from string to imu and MagneticField. Also, I disabled the function of writing into the serials.
// By Binyu Xia


#include <ros/ros.h>
#include <serial/serial.h>  //ROS已经内置了的串口包
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <stdio.h>
//#include <std_msgs/String.h>

serial::Serial ser; //声明串口对象

//回调函数
//void write_callback(const std_msgs::String::ConstPtr& msg){
//ROS_INFO_STREAM("Writing to serial port" << msg->data);
//ser.write(msg->data);   //发送串口数据
//}

int main (int argc, char** argv){
    //初始化节点
    ros::init(argc, argv, "cerasus_imu");
    //声明节点句柄
    ros::NodeHandle nh;

    //订阅主题，并配置回调函数
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);
    try
    {
        //设置串口属性，并打开串口
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    //检测串口是否已经打开，并给出提示信息
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    //指定循环的频率
    ros::Rate loop_rate(100);
    while(ros::ok()){

        //处理ROS的信息，比如订阅消息,并调用回调函数
        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            sensor_msgs::Imu imu_msg;
            sensor_msgs::MagneticField mag_msg;
            imu_msg.header.frame_id="imu";
            mag_msg.header.frame_id="imu";
            std::sscanf(ser.read(ser.available()).c_str(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",&imu_msg.linear_acceleration.x,
                    &imu_msg.linear_acceleration.y,&imu_msg.linear_acceleration.z,&imu_msg.angular_velocity.x,
                   &imu_msg.angular_velocity.y,&imu_msg.angular_velocity.z,&mag_msg.magnetic_field.x,&mag_msg.magnetic_field.y,&mag_msg.magnetic_field.z);
            imu_msg.angular_velocity.z+=0.03816;
            imu_msg.angular_velocity.y-=0.03002;
            imu_msg.angular_velocity.x+=0.05755;

            imu_pub.publish(imu_msg);
            mag_pub.publish(mag_msg);
        }
        loop_rate.sleep();

    }
}