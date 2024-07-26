#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
// void Callback_right(const std_msgs::Float64::ConstPtr &msg)
// {
//     ROS_INFO("I get the right wheel rpm = %f",msg->data);
// }

// void Callback_left(const std_msgs::Float64::ConstPtr &msg)
// {
//     ROS_INFO("I get the left wheel rpm = %f",msg->data);
// }

void Callback_front_right(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("I get the front right wheel rpm = %f",msg->data);
}

void Callback_front_left(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("I get the front left wheel rpm = %f",msg->data);
}

void Callback_rear_right(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("I get the rear right wheel rpm = %f",msg->data);
}

void Callback_rear_left(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("I get the rear left wheel rpm = %f",msg->data);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_sub");
    ros::NodeHandle nh_front_right_sub,nh_front_left_sub,nh_rear_right_sub,nh_rear_left_sub;
    ros::Subscriber sub_front_right,sub_front_left,sub_rear_right,sub_rear_left;
    // sub_right = nh_right_sub.subscribe<std_msgs::Float64>("/right_wheel/rpm",1,Callback_right);
    // sub_left = nh_left_sub.subscribe<std_msgs::Float64>("/left_wheel/rpm",1,Callback_left);
    sub_front_right = nh_front_right_sub.subscribe<std_msgs::Float64>("/front_right_wheel/rpm",1,Callback_front_right);
    sub_front_left = nh_front_left_sub.subscribe<std_msgs::Float64>("/front_left_wheel/rpm",1,Callback_front_left);
    sub_rear_right = nh_rear_right_sub.subscribe<std_msgs::Float64>("/rear_right_wheel/rpm",1,Callback_rear_right);
    sub_rear_left = nh_rear_left_sub.subscribe<std_msgs::Float64>("/rear_left_wheel/rpm",1,Callback_rear_left);

    ros::spin();
    return 0;

}
// int main(int argc,char **argv)
// {
//     ros::init(argc,argv,"test_sub");
//     ros::NodeHandle nh_right_sub,nh_left_sub;
//     ros::Subscriber sub_right,sub_left;
//     sub_right = nh_right_sub.subscribe<std_msgs::Float64>("/right_wheel/rpm",1,Callback_right);
//     sub_left = nh_left_sub.subscribe<std_msgs::Float64>("/left_wheel/rpm",1,Callback_left);

//     ros::spin();
//     return 0;

// }