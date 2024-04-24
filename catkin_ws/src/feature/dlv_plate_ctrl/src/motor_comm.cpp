#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <cstdint>

extern "C"
{
  #include <motor_function.h>
}

#define DEBUG

// Msg {Data, Length, Status}
void initMsg(carInfo *car_info);
void clearMsg(carInfo *car_info);
void processMsg(carInfo *car_info);
void sendMsg(carInfo *car_info);
void clearData(serialData *targetMsg);
void velCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);

carInfo car_info_;

int main(int argc, char **argv)
{
  // ros::Rate r(20);
  ros::init(argc, argv, "motor_comm");
  ros::NodeHandle rosNh;
  ros::Subscriber velCmdSub = rosNh.subscribe("/dlv/cmd_vel", 1, velCmdCallback);
  serialInit();
  initMsg(&car_info_);
  ros::spin();
}

void velCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  car_info_.linear_x = msg->linear.x;
  car_info_.angular_z = msg->angular.z;
  processMsg(&car_info_);
  sendMsg(&car_info_);
  return;
}

void processMsg(carInfo *car_info)
{
  double linear_x = car_info->linear_x;
  double angular_z = car_info->angular_z;

  double wheel_radius = 0.15;
  double axis_length = 0.63;

  double right_wheel_vel = -120 * (linear_x + angular_z * axis_length) / wheel_radius;
  double left_wheel_vel = 120 * (linear_x  - angular_z * axis_length) / wheel_radius;

  #ifdef DEBUG
  std::cout<<"---------------------"<<'\n';
  std::cout<<"right_wheel_vel: "<<right_wheel_vel<<'\n';
  std::cout<<"left_wheel_vel: "<<left_wheel_vel<<'\n';
  std::cout<<"---------------------"<<'\n';
  #endif

  car_info->right_wheel.data[4] = (0xff & (int(right_wheel_vel) >> 8));
  car_info->left_wheel.data[4] = (0xff & (int(left_wheel_vel) >> 8));

  car_info->right_wheel.data[5] = (0xff & int(right_wheel_vel));
  car_info->left_wheel.data[5] = (0xff & int(left_wheel_vel));

  CRC16Generate(&car_info->right_wheel);
  CRC16Generate(&car_info->left_wheel);
  return;
}

void sendMsg(carInfo *car_info)
{
  transmitData(&car_info->right_wheel);
  receiveData(&car_info->right_wheel);

  transmitData(&car_info->left_wheel);
  receiveData(&car_info->left_wheel);

  return;
}

void initMsg(carInfo *car_info)
{
  clearMsg(car_info);

  car_info->right_wheel.length = 8;
  car_info->left_wheel.length = 8;

  // 從站
  car_info->right_wheel.data[0] = 1;
  car_info->left_wheel.data[0] = 2;

  // 寫入模式 
  car_info->right_wheel.data[1] = 6;
  car_info->left_wheel.data[1] = 6;


  // 速度控制 dec(43)
  car_info->right_wheel.data[2] = 0;
  car_info->left_wheel.data[2] = 0;

  car_info->right_wheel.data[3] = 67;
  car_info->left_wheel.data[3] = 67;

  // 初始速度 0
  car_info->right_wheel.data[4] = 0;
  car_info->left_wheel.data[4] = 0;

  car_info->right_wheel.data[5] = 0;
  car_info->left_wheel.data[5] = 0;
}

void clearMsg(carInfo *car_info)
{
  clearData(&car_info->right_wheel);
  clearData(&car_info->left_wheel);
  return;
}

void clearData(serialData *targetMsg)
{
  for(int i = 0; i < 20; i++)
  {
    targetMsg->data[i]= 0;
  }
  targetMsg->length = 0;
  return;
}
