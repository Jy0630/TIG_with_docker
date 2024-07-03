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

#define DEBUG
typedef struct {
    double v;
    double vx;  // x 方向速度
    double vy;  // y 方向速度
    double omega;   // 角速度
} MotionState;

typedef struct {
    double omega_left;
    double omega_right;
} AngularVel;

typedef struct {
    double x;
    double y;
    double th;
} Position;

AngularVel angularvel;
Position position;

void CallBack_left(const std_msgs::Float64::ConstPtr& Omega);
void CallBack_right(const std_msgs::Float64::ConstPtr& Omega);
void CalculatePosition(AngularVel* angularvel, Position* position);
MotionState CalculateMotion(AngularVel* angularvel, double R, double L, Position* position, double dt);

void CallBack_left(const std_msgs::Float64::ConstPtr& Omega){
    angularvel.omega_left = Omega->data;
    angularvel.omega_left /= 60; // RoundPerSecond
    angularvel.omega_left *= (0.1 * M_PI); // achieve omega of left wheel
}

void CallBack_right(const std_msgs::Float64::ConstPtr& Omega){
    angularvel.omega_right = Omega->data;
    angularvel.omega_right /= 60;
    angularvel.omega_right *= (0.1 * M_PI);
}

void CalculatePosition(AngularVel* angularvel, Position* position){
    static ros::Time last_time = ros::Time::now();

    double R = 0.075;  // 轮子半径 0.075
    double L = 0.62;  // 轮子间的轴距

    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    MotionState motion = CalculateMotion(angularvel, R, L, position, dt);

    if (motion.omega != 0) {
        // 車輛正在轉彎
        double radius = motion.v / motion.omega;   // 使用旋轉半徑計算
        double delta_th = motion.omega * dt;
        position->x += radius * (sin(position->th + delta_th) - sin(position->th));
        position->y -= radius * (cos(position->th + delta_th) - cos(position->th)); // 注意，這裡需要減去而不是加上
    } else {
        // 車輛直行
        position->x += motion.vx * dt;
        position->y += motion.vy * dt;
    }
    position->th += motion.omega * dt;

    #ifdef DEBUG
    std::cout << "x: " << position->x << ", y: " << position->y << ", theta (degrees): " << (position->th * 180 / M_PI) << "\n";
    #endif

    last_time = current_time;
}

MotionState CalculateMotion(AngularVel* angularvel, double R, double L, Position* position, double dt){
    double v_L = angularvel->omega_left * R;
    double v_R = angularvel->omega_right * R;

    double v = (v_L + v_R) / 2;
    double omega = (v_R - v_L) / L;

    double vx = v * cos(position->th);
    double vy = v * sin(position->th);

    return {v, vx, vy, omega};
}

int main(int argc, char **argv){
    position.x = 0.0;
    position.y = 0.0;
    position.th = 0.0;

    ros::init(argc, argv, "Odom_Calc");
    ros::NodeHandle rosNh_odom;
    ros::Subscriber Sub_left = rosNh_odom.subscribe("/left_wheel/rpm", 1, CallBack_left);
    ros::Subscriber Sub_right = rosNh_odom.subscribe("/right_wheel/rpm", 1, CallBack_right);
    ros::Publisher Pub_pos = rosNh_odom.advertise<geometry_msgs::Point>("/car_position", 1);

    while (ros::ok()){
        CalculatePosition(&angularvel, &position);
        geometry_msgs::Point msg;
        msg.x = position.x;
        msg.y = position.y;
        msg.z = position.th;
        Pub_pos.publish(msg);
        ros::spinOnce();
    }

    return 0;
}
