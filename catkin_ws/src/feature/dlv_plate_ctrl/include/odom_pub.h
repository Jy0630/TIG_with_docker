#ifndef __ODOM_PUB_H__
#define __ODOM_PUB_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

class OdomPub
{
public:
    OdomPub(ros::NodeHandle& nh);
    void updateOdom(double linear_x, double angular_z);
private:
    ros::Publisher odom_pub;
    tf::TransformBroadcaster* odom_broadcaster;
    double x = 0.0, y = 0.0, th = 0.0;
    double vx = 0.0, vy = 0.0, vth = 0.0;
    ros::Time current_time, last_time;
};

#endif // __ODOM_PUB_H__