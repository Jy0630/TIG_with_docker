#include <ros/ros.h>
#include <step_motor/SetDistance.h>  // 你的 Service 頭檔，改成你自己的套件名稱
#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "step_distance_client");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<step_motor::SetDistance>("cmd_distance_srv");

  while (ros::ok()) {
    float distance_cm;
    std::cout << "請輸入移動距離（cm）：";
    std::cin >> distance_cm;

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(1000, '\n');
      std::cout << "請輸入有效數字。\n";
      continue;
    }

    step_motor::SetDistance srv;
    srv.request.distance = distance_cm;

    if (client.call(srv)) {
      ROS_INFO("服務回傳: %s", srv.response.result.c_str());
    } else {
      ROS_ERROR("無法呼叫 cmd_distance_srv 服務");
    }
  }

  return 0;
}
