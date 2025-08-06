#include <ros/ros.h>
#include <step_motor/SetDistance.h>  // 改成你自己套件名稱
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "step_distance_client");
  ros::NodeHandle nh;

  // 建立兩個 service client
  ros::ServiceClient client1 = nh.serviceClient<step_motor::SetDistance>("cmd_distance_srv1");
  ros::ServiceClient client2 = nh.serviceClient<step_motor::SetDistance>("cmd_distance_srv2");

  while (ros::ok()) {
    int motor_id;
    float distance_cm;

    std::cout << "\n選擇馬達 (1 或 2)：";
    std::cin >> motor_id;

    if (motor_id != 1 && motor_id != 2) {
      std::cout << "無效的馬達編號，請輸入 1 或 2。\n";
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      continue;
    }

    std::cout << "輸入移動距離（cm，可正負）：";
    std::cin >> distance_cm;

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      std::cout << "請輸入有效的數字。\n";
      continue;
    }

    step_motor::SetDistance srv;
    srv.request.distance = distance_cm;

    // 呼叫對應馬達的 service
    if (motor_id == 1) {
      if (client1.call(srv)) {
        ROS_INFO("馬達1 回傳: %s", srv.response.result.c_str());
      } else {
        ROS_ERROR("無法呼叫 cmd_distance_srv1");
      }
    } else if (motor_id == 2) {
      if (client2.call(srv)) {
        ROS_INFO("馬達2 回傳: %s", srv.response.result.c_str());
      } else {
        ROS_ERROR("無法呼叫 cmd_distance_srv2");
      }
    }
  }

  return 0;
}