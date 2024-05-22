#include "odom_pub.h"
#include <tf/transform_datatypes.h>

OdomPub::OdomPub(ros::NodeHandle& nh) :
    x(0.0), y(0.0), th(0.0),
    vx(0.0), vy(0.0), vth(0.0)
{
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    odom_broadcaster = new tf::TransformBroadcaster();
    last_time = ros::Time::now();
}

void OdomPub::updateOdom(double linear_x, double angular_z)
{
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (linear_x * cos(th)) * dt;
    double delta_y = (linear_x * sin(th)) * dt;
    double delta_th = angular_z * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster->sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.angular.z = angular_z;

    odom_pub.publish(odom);

    last_time = current_time;
}
