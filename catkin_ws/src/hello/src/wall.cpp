#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <string>
#include <sstream>

class HeightPublisher
{
public:
    HeightPublisher()
    {
        pub = nh.advertise<std_msgs::Float32>("/target_height", 10);
    }

    void run()
    {
        ros::Rate rate(10);  // 10 Hz

        while (ros::ok())
        {
            std::string input;
            std::cout << "Enter target height (cm): ";
            std::getline(std::cin, input);

            std_msgs::Float32 msg;
            std::stringstream ss(input);
            float height;
            if (ss >> height)
            {
                msg.data = height;
                pub.publish(msg);
                ROS_INFO("Published target height: %.2f cm", height);
            }
            else
            {
                ROS_WARN("Invalid input. Please enter a number.");
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "height_publisher_node");
    HeightPublisher heightPublisher;
    heightPublisher.run();
    return 0;
}
