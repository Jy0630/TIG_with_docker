#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cstdint>


ros::Publisher motorActualSpeedPub;


void rpmCallback(const std_msgs::Float64::ConstPtr& msg)
{

    double wheel_diameter = 0.1;
    double wheel_circumference = M_PI * wheel_diameter;
    double rpm = msg->data;

    double actual_speed = (rpm / 60.0) * wheel_circumference;

    std_msgs::Float64 actual_speed_msg;
    actual_speed_msg.data = actual_speed;
    motorActualSpeedPub.publish(actual_speed_msg);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "motor_respond_pub");
    ros::NodeHandle rosNh;
    motorActualSpeedPub = rosNh.advertise<std_msgs::Float64>("/motor/actual_speed", 20);

    ros::Subscriber rpmSubscriber = rosNh.subscribe("/front_left_wheel/rpm", 20, rpmCallback);
    
    ros::spin();  // 进入循环处理回调
    return 0;
}

