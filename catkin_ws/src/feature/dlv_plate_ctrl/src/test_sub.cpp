#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
void Callback_right(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("I get the right wheel rpm = %f",msg->data);
}
void Callback_left(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("I get the left wheel rpm = %f",msg->data);
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_sub");
    ros::NodeHandle nh_right_sub,nh_left_sub;
    ros::Subscriber sub_right,sub_left;
    sub_right = nh_right_sub.subscribe<std_msgs::Float64>("/right_wheel/rpm",1,Callback_right);
    sub_left = nh_left_sub.subscribe<std_msgs::Float64>("/left_wheel/rpm",1,Callback_left);

    ros::spin();
    return 0;

}