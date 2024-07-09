#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <cstdint>

extern "C"
{
  #include <motor_function.h>
}

// #define DEBUG

// Msg {Data, Length, Status}
void initMsg(carInfo *car_info);
void clearMsg(carInfo *car_info);
void processMsg(carInfo *car_info);
void sendMsg(carInfo *car_info);
void receiveMsg(carInfo *car_info);
void clearData(serialData *targetMsg);
void velCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
void readRegister_right(serialData *targetMsg);
void readRegister_left(serialData *targetMsg);
void CaclRpm(carInfo *carinfo);


carInfo car_info_;


int main(int argc, char **argv)
{
  // ros::Rate r(20);
  ros::init(argc, argv, "motor_comm");//can only have one node name "motor_comm"
  ros::NodeHandle rosNh_sub;
  ros::Publisher rightRpmPub = rosNh_sub.advertise<std_msgs::Float64>("/right_wheel/rpm",1);//But can have more than one topic. Ex:"/right_wheel/rpm"  and /left_wheel/rpm" 
  ros::Publisher leftRpmPub = rosNh_sub.advertise<std_msgs::Float64>("/left_wheel/rpm",1);
  ros::Subscriber velCmdSub = rosNh_sub.subscribe("/dlv/cmd_vel", 1, velCmdCallback);
  serialInit();
  initMsg(&car_info_);
  
  std_msgs::Float64 right_rpm_msg, left_rpm_msg;
  while(ros::ok())
  {
    receiveMsg(&car_info_);
    CaclRpm(&car_info_);
    right_rpm_msg.data = car_info_.right_rpm,left_rpm_msg.data = car_info_.left_rpm;//
    if(right_rpm_msg.data > 32767){
      right_rpm_msg.data-=65536;
    }
    if(left_rpm_msg.data > 32767){
      left_rpm_msg.data-=65536;
    }
    left_rpm_msg.data*=-1;
    rightRpmPub.publish(right_rpm_msg);
    leftRpmPub.publish(left_rpm_msg);
    // std::cout << "right wheel rpm = " << right_rpm_msg.data << " " << "left wheel rpm = " << left_rpm_msg.data << "\n";
    ros::spinOnce();
  }


}

 int times = 0;
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

  double wheel_radius = 0.075;
  double axis_length = 0.62;

  double right_wheel_vel = -50 * (linear_x + angular_z * axis_length) / wheel_radius;
  double left_wheel_vel = 50 * (linear_x  - angular_z * axis_length) / wheel_radius;

  #ifdef DEBUG
  std::cout<<"---------------------"<<'\n';
  std::cout<<"right_wheel_vel: "<<right_wheel_vel<<'\n';
  std::cout<<"left_wheel_vel: "<<left_wheel_vel<<'\n';
  std::cout<<"---------------------"<<'\n';
  #endif

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

  car_info->right_wheel.data[4] = (0xff & (int(right_wheel_vel) >> 8));
  car_info->left_wheel.data[4] = (0xff & (int(left_wheel_vel) >> 8));

  car_info->right_wheel.data[5] = (0xff & int(right_wheel_vel));
  car_info->left_wheel.data[5] = (0xff & int(left_wheel_vel));

  #ifdef DEBUG
  std::cout << "car_info->right_wheel.data[4] = " << (0xff & (int(right_wheel_vel) >> 8)) << "car_info->left_wheel.data[4] = " << (0xff & int(left_wheel_vel)) << "\n";
  #endif
  
  CRC16Generate(&car_info->right_wheel);
  CRC16Generate(&car_info->left_wheel);
  return;
}

void sendMsg(carInfo *car_info)
{
  transmitData(&car_info->left_wheel);
  receiveData(&car_info->left_wheel);

  transmitData(&car_info->right_wheel);
  receiveData(&car_info->right_wheel);

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

  car_info->right_wheel.data[4] = 0;
  car_info->left_wheel.data[4] = 0;

  car_info->right_wheel.data[5] = 0;
  car_info->left_wheel.data[5] = 0;
}

void receiveMsg(carInfo *car_info) {
  clearMsg(car_info);
  

  readRegister_right(&car_info->right_wheel);
  readRegister_left(&car_info->left_wheel);

  CRC16Generate(&car_info->right_wheel);
  CRC16Generate(&car_info->left_wheel);

  transmitData(&car_info->right_wheel);
  receiveData(&car_info->right_wheel);

  transmitData(&car_info->left_wheel);
  receiveData(&car_info->left_wheel);

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


void readRegister_right(serialData *targetMsg){
  // For more information about the AQMD6010BLs motor controller, find the use manual at page 119

  uint16_t controller_address = 2;
  uint16_t function_code = 0x03;
  uint16_t start_address = 0x22;
  uint16_t registers_amount = 0x01;

  targetMsg->length = 8;

  // ADR
  targetMsg->data[0] = controller_address;

  // Read function code
  targetMsg->data[1] = function_code;

  // Start register high bit
  targetMsg->data[2] = 0x00;

  // Start register low bit
  targetMsg->data[3] = start_address;

  // Register amount high bit
  targetMsg->data[4] = 0x00;

  // Register amount low bit
  targetMsg->data[5] = registers_amount;

  return;
}



void readRegister_left(serialData *targetMsg){
  // For more information about the AQMD6010BLs motor controller, find the use manual at page 119

  uint16_t controller_address = 1;
  uint16_t function_code = 0x03;
  uint16_t start_address = 0x22;
  uint16_t registers_amount = 0x01;
  
  targetMsg->length = 8;

  // ADR
  targetMsg->data[0] = controller_address;

  // Read function code
  targetMsg->data[1] = function_code;

  // Start register high bit
  targetMsg->data[2] = 0x00;

  // Start register low bit
  targetMsg->data[3] = start_address;

  // Register amount high bit
  targetMsg->data[4] = 0x00;

  // Register amount low bit
  targetMsg->data[5] = registers_amount;

  return;
}

void CaclRpm(carInfo *carinfo)
{
  if(carinfo->right_wheel.length >=7)//judge the rpm is needed to times 10 or not
  {
    int rpm_local = 0;
    rpm_local |= carinfo->right_wheel.data[3];
    rpm_local = (rpm_local << 8);
    rpm_local += carinfo->right_wheel.data[4];
    carinfo->right_rpm = rpm_local;
  }

  if(carinfo->left_wheel.length >= 7)//judge the rpm is needed to times 10 or not
  {
    int rpm_local = 0;
    rpm_local |= carinfo->left_wheel.data[3];
    rpm_local = (rpm_local << 8);
    rpm_local += carinfo->left_wheel.data[4];
    carinfo->left_rpm = rpm_local;
  }
}




