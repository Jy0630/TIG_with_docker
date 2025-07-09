#include <ros/ros.h>
#include "DC_motor/SetHeight.h"  // 這是你的 service 檔案
#include <iostream>           // 為了輸入輸出用

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_height_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<hello::SetHeight>("set_height");

    float height;
    float relative;  // ⭐修改處：新增變數儲存相對高度

    std::cout << "請輸入絕對高度（cm）: ";
    std::cin >> height;
    std::cout << "請輸入相對偏移高度（cm，可正可負）: ";  // ⭐修改處：新增提示
    std::cin >> relative;

    hello::SetHeight srv;
    srv.request.height = height;
    srv.request.relative = relative;  // ⭐修改處：新增傳送相對高度

    if (client.call(srv))
    {
        if (srv.response.success)
            ROS_INFO("成功設定高度為 %.2f cm（相對偏移 %.2f cm）", srv.request.height, srv.request.relative);  // ⭐修改處：新增資訊
        else
            ROS_WARN("設定高度失敗！");//若 service 雖呼叫成功，但處理失敗，則印出警告。
    }
    else
    {
        ROS_ERROR("無法呼叫 set_height 服務");//如果無法呼叫 service（例如 Arduino 沒上線或連線失敗），印出錯誤。
    }

    return 0;
}
