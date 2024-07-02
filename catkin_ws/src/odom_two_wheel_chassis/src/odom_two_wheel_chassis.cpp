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
    double vx;  // x 方向速度
    double vy;  // y 方向速度
    double omega; // 角速度
}MotionState;

typedef struct {
    double omega_left;
    double omega_right;
}AngularVel;

typedef struct{
    double x;
    double y;
    double th;
}Position;

AngularVel angularvel;
Position position;
void CallBack_left(const std_msgs::Float64::ConstPtr& Omega);
void CallBack_right(const std_msgs::Float64::ConstPtr& Omega);
void CalculatePostion(AngularVel* angularvel, Position* position);
MotionState CalculateMotion(AngularVel* angularvel, double R, double L, Position* position, double dt);

void CallBack_left(const std_msgs::Float64::ConstPtr& Omega)
{
    angularvel.omega_left = Omega->data;
    angularvel.omega_left /= 60;//RoundPerSecond
    angularvel.omega_left *= (0.1 * M_PI);//acheive omega of left wheel

}

void CallBack_right(const std_msgs::Float64::ConstPtr& Omega)
{
    angularvel.omega_right = Omega->data;
    angularvel.omega_right /= 60;
    angularvel.omega_right *= (0.1 * M_PI);
}

void CalculatePostion(AngularVel* angularvel, Position* position){

    // ros::Time current_time, last_time;
    
    static ros::Time last_time = ros::Time::now();

    double R = 0.075;  // 轮子半径 ?0.075
    double L = 0.62;  // 轮子间的轴距

    ros::Rate r(10);
   
    ros::Time current_time = ros::Time::now();

    // Compute time step
    double dt = (current_time - last_time).toSec();       

    // #ifdef DEBUG
    // std::cout<<"omega_L is " << angularvel->omega_left <<"\n";
    // std::cout<<"omega_R is " << angularvel->omega_right <<"\n";
    // #endif

    // Calculate motion state
    MotionState motion = CalculateMotion(angularvel, R, L, position, dt);

    // Update position and orientation
    position->x += motion.vx * dt;
    position->y += motion.vy * dt;
    position->th += motion.omega * dt;
    // #ifdef DEBUG
    // if(position->th >= 360){
    //     position->th -= 360;
    // }
    // else if(position->th < 0){
    //     position->th += 360;
    // }
    // #endif

    #ifdef DEBUG
    std::cout << position->x <<' '<< position->y << ' '<<((position->th) * 180 / 3.1416) << "\n";
    // std::cout << "x is " << position->x << ", y is " << position->y <<", theta_z is " << ((position->th) * 180 / 3.1416) << "\n";
    // std::cout<<"-------------------------------------------------------------\n";
    #endif

    last_time = current_time;
    return;
}

MotionState CalculateMotion(AngularVel* angularvel, double R, double L, Position* position, double dt) {//L是兩輪間距
    // 计算左右轮的线速度
    double v_L = angularvel->omega_left * R;
    double v_R = angularvel->omega_right * R;

    // 计算车辆的线速度和角速度
    double v = (v_L + v_R) / 2;
    double omega = (v_R - v_L) / L;
    

    // 计算 x 和 y 方向的速度
    double vx = v * cos(position->th );
    double vy = v * sin(position->th );



    // #ifdef DEBUG
    // std::cout<<"vx is "<<vx<<", vy is "<<vy<<", omega_z is "<<omega<<"\n";
    // std::cout<<"-------------------------------------------------------------\n";
    // #endif

    return {vx, vy, omega};
}

int main(int argc,char **argv)
{
    position.x = 0.0;
    position.y = 0.0;
    position.th = 0.0;
    
    ros::init(argc,argv,"Odom_Cacl");
    ros::NodeHandle rosNh_odom;
    ros::Subscriber Sub_left = rosNh_odom.subscribe("/right_wheel/rpm",1,CallBack_left);
    ros::Subscriber Sub_right = rosNh_odom.subscribe("/left_wheel/rpm",1,CallBack_right);
    ros::Publisher Pub_pos = rosNh_odom.advertise<geometry_msgs::Point>("/car_postion",1);

    while(ros::ok())
    {

        CalculatePostion(&angularvel,&position);
        geometry_msgs::Point msg;
        #ifdef DEBUG
        // std::cout<<"x is "<<position.x<<", y is "<<position.y<<"theta_z is "<<position.th<<"\n";
        #endif
        msg.x = position.x;
        msg.y = position.y;
        msg.z = position.th;
        Pub_pos.publish(msg);
        // #ifdef DEBUG
        // std::cout<<"x is "<<msg.x<<", y is "<<msg.y<<"theta_z is "<<(msg.z * 180 / M_PI)<<"\n";
        // #endif


        ros::spinOnce();
    }

}

