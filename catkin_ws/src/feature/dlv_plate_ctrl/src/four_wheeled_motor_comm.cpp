#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <cstdint>

extern "C" {
  #include <motor_function.h>
}

// #define DEBUG

void initMsg(carInfo *car_info);
void clearMsg(carInfo *car_info);
void processMsg(carInfo *car_info);
void sendMsg(carInfo *car_info);
void receiveMsg(carInfo *car_info);
void clearData(serialData *targetMsg);
void velCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
void readRegister_wheel(serialData *targetMsg, int wheel_id);
void writePidToController(double p,double i, double d, int wheel_id);
void calcRpm(carInfo *car_info);

carInfo car_info_;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_comm");
  ros::NodeHandle rosNh_sub;
  rosNh_sub.getParam("/pid/front_right/p", car_info_.front_right_wheel_p);
  rosNh_sub.getParam("/pid/front_right/i", car_info_.front_right_wheel_i);
  rosNh_sub.getParam("/pid/front_right/d", car_info_.front_right_wheel_d);
  rosNh_sub.getParam("/pid/front_left/p", car_info_.front_left_wheel_p);
  rosNh_sub.getParam("/pid/front_left/i", car_info_.front_left_wheel_i);
  rosNh_sub.getParam("/pid/front_left/d", car_info_.front_left_wheel_d);
  rosNh_sub.getParam("/pid/rear_right/p", car_info_.rear_right_wheel_p);
  rosNh_sub.getParam("/pid/rear_right/i", car_info_.rear_right_wheel_i);
  rosNh_sub.getParam("/pid/rear_right/d", car_info_.rear_right_wheel_d);
  rosNh_sub.getParam("/pid/rear_left/p", car_info_.rear_left_wheel_p);
  rosNh_sub.getParam("/pid/rear_left/i", car_info_.rear_left_wheel_i);
  rosNh_sub.getParam("/pid/rear_left/d", car_info_.rear_left_wheel_d);

  ros::Publisher frontRightRpmPub = rosNh_sub.advertise<std_msgs::Float64>("/front_right_wheel/rpm", 1);
  ros::Publisher frontLeftRpmPub = rosNh_sub.advertise<std_msgs::Float64>("/front_left_wheel/rpm", 1);
  ros::Publisher rearRightRpmPub = rosNh_sub.advertise<std_msgs::Float64>("/rear_right_wheel/rpm", 1);
  ros::Publisher rearLeftRpmPub = rosNh_sub.advertise<std_msgs::Float64>("/rear_left_wheel/rpm", 1);
  ros::Subscriber velCmdSub = rosNh_sub.subscribe("/dlv/cmd_vel", 1, velCmdCallback);
  serialInit();
  initMsg(&car_info_);
  
  std_msgs::Float64 front_right_rpm_msg, front_left_rpm_msg, rear_right_rpm_msg, rear_left_rpm_msg;

  writePidToController(car_info_.front_right_wheel_p,car_info_.front_right_wheel_i,car_info_.front_right_wheel_d,1);
  writePidToController(car_info_.front_left_wheel_p,car_info_.front_left_wheel_i,car_info_.front_left_wheel_d,2);
  writePidToController(car_info_.rear_right_wheel_p,car_info_.rear_right_wheel_i,car_info_.rear_right_wheel_d,3);
  writePidToController(car_info_.rear_left_wheel_p,car_info_.rear_left_wheel_i,car_info_.rear_left_wheel_d,4);
  
  while (ros::ok())
  {
    receiveMsg(&car_info_);
    calcRpm(&car_info_);
    front_right_rpm_msg.data = car_info_.front_right_rpm;
    front_left_rpm_msg.data = car_info_.front_left_rpm;
    rear_right_rpm_msg.data = car_info_.rear_right_rpm;
    rear_left_rpm_msg.data = car_info_.rear_left_rpm;

    if (front_right_rpm_msg.data > 32767) front_right_rpm_msg.data -= 65536;
    if (front_left_rpm_msg.data > 32767) front_left_rpm_msg.data -= 65536;
    if (rear_right_rpm_msg.data > 32767) rear_right_rpm_msg.data -= 65536;
    if (rear_left_rpm_msg.data > 32767) rear_left_rpm_msg.data -= 65536;

    front_right_rpm_msg.data *= 0.05;
    front_left_rpm_msg.data *= 0.05;
    rear_left_rpm_msg.data *= 0.05;
    rear_right_rpm_msg.data *= 0.05; 
    front_left_rpm_msg.data *= -1;
    rear_right_rpm_msg.data *= -1;
    front_right_rpm_msg.data *= -1;

    frontRightRpmPub.publish(front_right_rpm_msg);
    frontLeftRpmPub.publish(front_left_rpm_msg);
    rearRightRpmPub.publish(rear_right_rpm_msg);
    rearLeftRpmPub.publish(rear_left_rpm_msg);

    ros::spinOnce();
  }
}

void velCmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  car_info_.linear_x = msg->linear.x;
  car_info_.linear_y = msg->linear.y;
  car_info_.angular_z = msg->angular.z;
  processMsg(&car_info_);
  sendMsg(&car_info_);
}

void processMsg(carInfo *car_info)
{
  double linear_x = car_info->linear_x;
  double linear_y = car_info->linear_y;
  double angular_z = car_info->angular_z;

  double wheel_radius = 0.075;
  double axis_length = 0.5325;

  double front_right_vel = (linear_x + linear_y + angular_z * axis_length) / wheel_radius *200;
  double front_left_vel = (linear_x - linear_y - angular_z * axis_length) / wheel_radius *200; // Negative due to the direction
  double rear_right_vel = (linear_x - linear_y + angular_z * axis_length) / wheel_radius *200; // Negative due to the direction
  double rear_left_vel = (linear_x + linear_y - angular_z * axis_length) / wheel_radius *200;

  front_right_vel *= -1.0;
  front_left_vel *= -1.0;
  rear_right_vel *= -1.0;

  double max_vel = 3000.0;

  front_right_vel = std::clamp(front_right_vel, -max_vel, max_vel);
  front_left_vel = std::clamp(front_left_vel, -max_vel, max_vel);
  rear_right_vel = std::clamp(rear_right_vel, -max_vel, max_vel);
  rear_left_vel = std::clamp(rear_left_vel, -max_vel, max_vel);

  #ifdef DEBUG
  std::cout << "Front Right Wheel Velocity: " << front_right_vel << '\n';
  std::cout << "Front Left Wheel Velocity: " << front_left_vel << '\n';
  std::cout << "Back Right Wheel Velocity: " << back_right_vel << '\n';
  std::cout << "Back Left Wheel Velocity: " << back_left_vel << '\n';
  #endif

  clearMsg(car_info);

  car_info->front_right_wheel.length = 8;
  car_info->front_left_wheel.length = 8;
  car_info->rear_right_wheel.length = 8;
  
  car_info->rear_left_wheel.length = 8;

  car_info->front_right_wheel.data[0] = 1;
  car_info->front_left_wheel.data[0] = 2;
  car_info->rear_right_wheel.data[0] = 3;
  car_info->rear_left_wheel.data[0] = 4;

  car_info->front_right_wheel.data[1] = 6;
  car_info->front_left_wheel.data[1] = 6;
  car_info->rear_right_wheel.data[1] = 6;
  car_info->rear_left_wheel.data[1] = 6;

  car_info->front_right_wheel.data[2] = 0;
  car_info->front_left_wheel.data[2] = 0;
  car_info->rear_right_wheel.data[2] = 0;
  car_info->rear_left_wheel.data[2] = 0;

  car_info->front_right_wheel.data[3] = 67;
  car_info->front_left_wheel.data[3] = 67;
  car_info->rear_right_wheel.data[3] = 67;
  car_info->rear_left_wheel.data[3] = 67;

  car_info->front_right_wheel.data[4] = (0xff & (int(front_right_vel) >> 8));
  car_info->front_left_wheel.data[4] = (0xff & (int(front_left_vel) >> 8));
  car_info->rear_right_wheel.data[4] = (0xff & (int(rear_right_vel) >> 8));
  car_info->rear_left_wheel.data[4] = (0xff & (int(rear_left_vel) >> 8));

  car_info->front_right_wheel.data[5] = (0xff & int(front_right_vel));
  car_info->front_left_wheel.data[5] = (0xff & int(front_left_vel));
  car_info->rear_right_wheel.data[5] = (0xff & int(rear_right_vel));
  car_info->rear_left_wheel.data[5] = (0xff & int(rear_left_vel));

  CRC16Generate(&car_info->front_right_wheel);
  CRC16Generate(&car_info->front_left_wheel);
  CRC16Generate(&car_info->rear_right_wheel);
  CRC16Generate(&car_info->rear_left_wheel);
}

void sendMsg(carInfo *car_info)
{
  transmitData(&car_info->front_right_wheel);
  receiveData(&car_info->front_right_wheel);
  transmitData(&car_info->front_left_wheel);
  receiveData(&car_info->front_left_wheel);
  transmitData(&car_info->rear_right_wheel);
  receiveData(&car_info->rear_right_wheel);
  transmitData(&car_info->rear_left_wheel);
  receiveData(&car_info->rear_left_wheel);
}

void initMsg(carInfo *car_info)
{
  clearMsg(car_info);

  car_info->front_right_wheel.length = 8;
  car_info->front_left_wheel.length = 8;
  car_info->rear_right_wheel.length = 8;
  car_info->rear_left_wheel.length = 8;

  car_info->front_right_wheel.data[0] = 1;
  car_info->front_left_wheel.data[0] = 2;
  car_info->rear_right_wheel.data[0] = 3;
  car_info->rear_left_wheel.data[0] = 4;

  car_info->front_right_wheel.data[1] = 6;
  car_info->front_left_wheel.data[1] = 6;
  car_info->rear_right_wheel.data[1] = 6;
  car_info->rear_left_wheel.data[1] = 6;

  car_info->front_right_wheel.data[2] = 0;
  car_info->front_left_wheel.data[2] = 0;
  car_info->rear_right_wheel.data[2] = 0;
  car_info->rear_left_wheel.data[2] = 0;

  car_info->front_right_wheel.data[3] = 67;
  car_info->front_left_wheel.data[3] = 67;
  car_info->rear_right_wheel.data[3] = 67;
  car_info->rear_left_wheel.data[3] = 67;

  car_info->front_right_wheel.data[4] = 0;
  car_info->front_left_wheel.data[4] = 0;
  car_info->rear_right_wheel.data[4] = 0;
  car_info->rear_left_wheel.data[4] = 0;

  car_info->front_right_wheel.data[5] = 0;
  car_info->front_left_wheel.data[5] = 0;
  car_info->rear_right_wheel.data[5] = 0;
  car_info->rear_left_wheel.data[5] = 0;
}

void receiveMsg(carInfo *car_info) 
{
  clearMsg(car_info);
  
  readRegister_wheel(&car_info->front_right_wheel, 1);
  readRegister_wheel(&car_info->front_left_wheel, 2);
  readRegister_wheel(&car_info->rear_right_wheel, 3);
  readRegister_wheel(&car_info->rear_left_wheel, 4);

  CRC16Generate(&car_info->front_right_wheel);
  CRC16Generate(&car_info->front_left_wheel);
  CRC16Generate(&car_info->rear_right_wheel);
  CRC16Generate(&car_info->rear_left_wheel);

  transmitData(&car_info->front_right_wheel);
  receiveData(&car_info->front_right_wheel);

  transmitData(&car_info->front_left_wheel);
  receiveData(&car_info->front_left_wheel);

  transmitData(&car_info->rear_right_wheel);
  receiveData(&car_info->rear_right_wheel);

  transmitData(&car_info->rear_left_wheel);
  receiveData(&car_info->rear_left_wheel);
}

void clearMsg(carInfo *car_info)
{
  clearData(&car_info->front_right_wheel);
  clearData(&car_info->front_left_wheel);
  clearData(&car_info->rear_right_wheel);
  clearData(&car_info->rear_left_wheel);
}

void clearData(serialData *targetMsg)
{
  for(int i = 0; i < 20; i++)
  {
    targetMsg->data[i] = 0;
  }
  targetMsg->length = 0;
}

void readRegister_wheel(serialData *targetMsg, int wheel_id) 
{
  uint16_t controller_address = wheel_id;
  uint16_t function_code = 0x03;
  uint16_t start_address = 0x22;
  uint16_t registers_amount = 0x01;

  targetMsg->length = 8;

  targetMsg->data[0] = controller_address;
  targetMsg->data[1] = function_code;
  targetMsg->data[2] = 0x00;
  targetMsg->data[3] = start_address;
  targetMsg->data[4] = 0x00;
  targetMsg->data[5] = registers_amount;

  return;
}

void calcRpm(carInfo *car_info)
{
  auto calcSingleRpm = [](serialData& wheel) -> int {
    if (wheel.length >= 7) {
      int rpm_local = 0;
      rpm_local |= wheel.data[3];
      rpm_local = (rpm_local << 8);
      rpm_local += wheel.data[4];
      return rpm_local;
    }
    return 0;
  };

  car_info->front_right_rpm = calcSingleRpm(car_info->front_right_wheel);
  car_info->front_left_rpm = calcSingleRpm(car_info->front_left_wheel);
  car_info->rear_right_rpm = calcSingleRpm(car_info->rear_right_wheel); 
  car_info->rear_left_rpm = calcSingleRpm(car_info->rear_left_wheel);
}

void writePidToController(double p, double i, double d, int wheel_id)
{
    uint16_t controller_address = wheel_id;
    serialData msg;
    uint32_t p_int, i_int, d_int;

    // 將 double (64-bit) 轉換為 float (32-bit) 並使用 IEEE 754 格式存儲
    float p_float = (float)p;
    float i_float = (float)i;
    float d_float = (float)d;

    // 使用 memcpy 將 float 轉換為其對應的 32 位二進制格式
    memcpy(&p_int, &p_float, sizeof(p_float));
    memcpy(&i_int, &i_float, sizeof(i_float));
    memcpy(&d_int, &d_float, sizeof(d_float));

    // 發送 P 參數 (32 位)
    clearData(&msg);
    msg.length = 8;
    msg.data[0] = controller_address;
    msg.data[1] = 0x06;
    msg.data[2] = 0x00;
    msg.data[3] = 0xc0;  // KP 高位
    msg.data[4] = (0xff & (p_int >> 24)); // 發送高位
    msg.data[5] = (0xff & (p_int >> 16));
    CRC16Generate(&msg);
    transmitData(&msg);
    receiveData(&msg);

    clearData(&msg);
    msg.length = 8;
    msg.data[0] = controller_address;
    msg.data[1] = 0x06;
    msg.data[2] = 0x00;
    msg.data[3] = 0xc1;  // KP 低位
    msg.data[4] = (0xff & (p_int >> 8));  // 發送低位
    msg.data[5] = (0xff & p_int);
    CRC16Generate(&msg);
    transmitData(&msg);
    receiveData(&msg);

    // 發送 I 參數 (32 位)
    memcpy(&i_int, &i_float, sizeof(i_float));
    clearData(&msg);
    msg.length = 8;
    msg.data[0] = controller_address;
    msg.data[1] = 0x06;
    msg.data[2] = 0x00;
    msg.data[3] = 0xc2;  // KI 高位
    msg.data[4] = (0xff & (i_int >> 24));
    msg.data[5] = (0xff & (i_int >> 16));
    CRC16Generate(&msg);
    transmitData(&msg);
    receiveData(&msg);

    clearData(&msg);
    msg.length = 8;
    msg.data[0] = controller_address;
    msg.data[1] = 0x06;
    msg.data[2] = 0x00;
    msg.data[3] = 0xc3;  // KI 低位
    msg.data[4] = (0xff & (i_int >> 8));
    msg.data[5] = (0xff & i_int);
    CRC16Generate(&msg);
    transmitData(&msg);
    receiveData(&msg);

    // 發送 D 參數 (32 位)
    memcpy(&d_int, &d_float, sizeof(d_float));
    clearData(&msg);
    msg.length = 8;
    msg.data[0] = controller_address;
    msg.data[1] = 0x06;
    msg.data[2] = 0x00;
    msg.data[3] = 0xc4;  // KD 高位
    msg.data[4] = (0xff & (d_int >> 24));
    msg.data[5] = (0xff & (d_int >> 16));
    CRC16Generate(&msg);
    transmitData(&msg);
    receiveData(&msg);

    clearData(&msg);
    msg.length = 8;
    msg.data[0] = controller_address;
    msg.data[1] = 0x06;
    msg.data[2] = 0x00;
    msg.data[3] = 0xc5;  // KD 低位
    msg.data[4] = (0xff & (d_int >> 8));
    msg.data[5] = (0xff & d_int);
    CRC16Generate(&msg);
    transmitData(&msg);
    receiveData(&msg);
}
