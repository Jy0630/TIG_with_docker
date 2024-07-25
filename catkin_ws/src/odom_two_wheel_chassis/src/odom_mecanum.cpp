#include <iostream>
#include <vector>
#include <cstdint>
#include <chrono>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>

// #define DEBUG
typedef struct {
    double vx;  // x 方向速度
    double vy;  // y 方向速度
    double omega;   // 角速度
} MotionState;

typedef struct {
    double omega_fl;
    double omega_fr;
    double omega_rl;
    double omega_rr;
} AngularVel;

typedef struct {
    double x;
    double y;
    double theta;
} Position;

AngularVel angularvel;
Position position;

void CallBack_fl(const std_msgs::Float64::ConstPtr& Omega);
void CallBack_fr(const std_msgs::Float64::ConstPtr& Omega);
void CallBack_rl(const std_msgs::Float64::ConstPtr& Omega);
void CallBack_rr(const std_msgs::Float64::ConstPtr& Omega);
void CalculatePosition(AngularVel* angularvel, Position* position);
MotionState CalculateMotion(AngularVel* angularvel, double R, double L, Position* position, double dt);

void CallBack_fl(const std_msgs::Float64::ConstPtr& Omega){
    angularvel.omega_fl = Omega->data;
    angularvel.omega_fl /= 60; // RoundPerSecond
    angularvel.omega_fl *= (0.1 * M_PI); // achieve omega of front-left wheel
}

void CallBack_fr(const std_msgs::Float64::ConstPtr& Omega){
    angularvel.omega_fr = Omega->data;
    angularvel.omega_fr /= 60;
    angularvel.omega_fr *= (0.1 * M_PI);
}

void CallBack_rl(const std_msgs::Float64::ConstPtr& Omega){
    angularvel.omega_rl = Omega->data;
    angularvel.omega_rl /= 60;
    angularvel.omega_rl *= (0.1 * M_PI);
}

void CallBack_rr(const std_msgs::Float64::ConstPtr& Omega){
    angularvel.omega_rr = Omega->data;
    angularvel.omega_rr /= 60;
    angularvel.omega_rr *= (0.1 * M_PI);
}

void CalculatePosition(AngularVel* angularvel, Position* position){
    static ros::Time last_time = ros::Time::now();

    double R = 0.075;  // 轮子半径 0.075
    double Lx = 0.62;  // 轮子间的轴距 (横向)
    double Ly = 0.62;  // 轮子间的轴距 (纵向)

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    MotionState motion = CalculateMotion(angularvel, R, Lx, Ly, position, dt);

    position->x += motion.vx * dt;
    position->y += motion.vy * dt;
    position->theta += motion.omega * dt;

    #ifdef DEBUG
    std::cout << "x: " << position->x << ", y: " << position->y << ", theta (degrees): " << (position->theta * 180 / M_PI) << "\n";
    #endif

    last_time = current_time;
}

MotionState CalculateMotion(AngularVel* angularvel, double R, double Lx, double Ly, Position* position, double dt){
    double v_fl = angularvel->omega_fl * R;
    double v_fr = angularvel->omega_fr * R;
    double v_rl = angularvel->omega_rl * R;
    double v_rr = angularvel->omega_rr * R;

    double vx = (v_fl + v_fr + v_rl + v_rr) / 4;
    double vy = (-v_fl + v_fr + v_rl - v_rr) / 4;
    double omega = (-v_fl + v_fr - v_rl + v_rr) / (4 * (Lx + Ly));

    return {vx, vy, omega};
}

int main(int argc, char **argv){
    position.x = 0.0;
    position.y = 0.0;
    position.theta = 0.0;

    ros::init(argc, argv, "Odom_Calc");
    ros::NodeHandle rosNh_odom;
    ros::Subscriber Sub_fl = rosNh_odom.subscribe("/front_left_wheel/rpm", 1, CallBack_fl);
    ros::Subscriber Sub_fr = rosNh_odom.subscribe("/front_right_wheel/rpm", 1, CallBack_fr);
    ros::Subscriber Sub_rl = rosNh_odom.subscribe("/rear_left_wheel/rpm", 1, CallBack_rl);
    ros::Subscriber Sub_rr = rosNh_odom.subscribe("/rear_right_wheel/rpm", 1, CallBack_rr);
    ros::Publisher Pub_pos = rosNh_odom.advertise<nav_msgs::Odometry>("/odom", 1);
    tf::TransformBroadcaster odom_broadcaster;

    while (ros::ok()) {
        CalculatePosition(&angularvel, &position);

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";  // child frame id

        odom.pose.pose.position.x = position.x;
        odom.pose.pose.position.y = position.y;
        odom.pose.pose.position.z = 0.0;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(position.theta);

        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        Pub_pos.publish(odom);

        // Broadcast transform from odom to base_link
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = position.x;
        odom_trans.transform.translation.y = position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        ros::spinOnce();
    }

    return 0;
}
