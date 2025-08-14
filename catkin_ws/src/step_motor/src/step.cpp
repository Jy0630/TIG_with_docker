#include <ros/ros.h>
#include <step_motor/SetDistance.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "step_distance_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<step_motor::SetDistance>("cmd_distance_srv");

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
        srv.request.motor_id = motor_id;
        srv.request.distance = distance_cm;

        if (client.call(srv)) {
            ROS_INFO("馬達%d 回傳: %s", motor_id, srv.response.result.c_str());
        } else {
            ROS_ERROR("無法呼叫 cmd_distance_srv");
        }
    }
    return 0;
}
