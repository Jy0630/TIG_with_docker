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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

struct MotionState {
    double vx;  // x 方向速度
    double vy;  // y 方向速度
    double omega; // 角速度
};

uint16_t calculateCRC(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < data.size(); pos++) {
        crc ^= data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int openSerialPort(const std::string& portName) {
    int serialPort = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialPort == -1) {
        std::cerr << "Error in opening serial port.\n";
        return -1;
    }

    termios tty;
    if (tcgetattr(serialPort, &tty) != 0) {
        std::cerr << "Error in getting serial port attributes.\n";
        close(serialPort);
        return -1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag |= PARENB;  // Enable parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit (most common)
    tty.c_cflag &= ~CSIZE;  // Clear all bits that set the data size
    tty.c_cflag |= CS8;     // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(ICRNL | INLCR); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        std::cerr << "Error in setting serial port attributes.\n";
        close(serialPort);
        return -1;
    }

    return serialPort;
}

bool writeData(int serialPort, const std::vector<uint8_t>& data) {
    ssize_t bytesWritten = write(serialPort, data.data(), data.size());
    if (bytesWritten < 0) {
        std::cerr << "Error writing data to serial port.\n";
        return false;
    }
    return true;
}

std::vector<uint8_t> readData(int serialPort, size_t size) {
    std::vector<uint8_t> buffer(size);
    ssize_t bytesRead = read(serialPort, buffer.data(), buffer.size());
    if (bytesRead < 0) {
        std::cerr << "Error reading data from serial port.\n";
        return {};
    }
    buffer.resize(bytesRead);
    return buffer;
}

std::vector<uint8_t> buildModbusRequest(uint8_t slaveAddress, uint8_t functionCode, uint16_t startAddress, uint16_t data) {
    std::vector<uint8_t> request = {slaveAddress, functionCode, (uint8_t)(startAddress >> 8), (uint8_t)startAddress, (uint8_t)(data >> 8), (uint8_t)data};
    uint16_t crc = calculateCRC(request);
    request.push_back(crc & 0xFF);
    request.push_back((crc >> 8) & 0xFF);
    return request;
}

void closeSerialPort(int serialPort) {
    close(serialPort);
}

MotionState calculateMotion(double omega_L, double omega_R, double R, double L, double theta, double dt) {//L是兩輪間距
    // 计算左右轮的线速度
    double v_L = omega_L * R;
    double v_R = omega_R * R;

    // 计算车辆的线速度和角速度
    double v = (v_L + v_R) / 2;
    double omega = (v_R - v_L) / L;

    // 计算 x 和 y 方向的速度
    double vx = v * cos(theta);
    double vy = v * sin(theta);

    // 更新航向角
    double new_theta = theta + omega * dt;

    return {vx, vy, omega};
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    std::string portName = "/dev/ttyS0"; // Adjust the port name accordingly
    int serialPort = openSerialPort(portName);

    if (serialPort == -1) {
        return 1;
    }

    uint8_t slaveAddress_left = 1;//left
    uint8_t slaveAddress_right = 2;//right
    uint8_t functionCode = 3;  // Write register
    uint16_t startAddress = 0x34;  // Holding register address
    uint16_t data = 0x0001;  // Data to write

    double R = 0.1;  // 轮子半径
    double L = 0.5;  // 轮子间的轴距

    ros::Rate r(1.0);
    while(n.ok()){
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        // Compute time step
        double dt = (current_time - last_time).toSec();

        // Build and send Modbus request
        std::vector<uint8_t> request_left = buildModbusRequest(slaveAddress_left, functionCode, startAddress, data);
        std::vector<uint8_t> request_right = buildModbusRequest(slaveAddress_right, functionCode, startAddress, data);

        if (!writeData(serialPort, request_left)) {
            closeSerialPort(serialPort);
            return 1;
        } 
         std::vector<uint8_t> response_left = readData(serialPort, 7);  // Based on expected Modbus RTU response length
        if (!writeData(serialPort, request_right)) {
            closeSerialPort(serialPort);
            return 1;
        }
        std::vector<uint8_t> response_right = readData(serialPort, 7);  // Based on expected Modbus RTU response length

        // Read response from serial port
       

        if ((response_left.size() > 0) && (response_right.size() > 0)) {
            // Parse the response to get wheel angular velocities
            // Assuming the response contains wheel angular velocities in a specific format
            // For example, let's assume response[3] and response[4] contain omega_L and omega_R respectively
            double omega_L = static_cast<double>((response_left[4] << 8)|response_left[5]);  // Example scaling
            double omega_R = static_cast<double>((response_right[4] << 8)|response_right[5]);

            // Calculate motion state
            MotionState motion = calculateMotion(omega_L, omega_R, R, L, th, dt);

            // Update position and orientation
            x += motion.vx * dt;
            y += motion.vy * dt;
            th += motion.omega * dt;

            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = motion.vx;
            odom.twist.twist.linear.y = motion.vy;
            odom.twist.twist.angular.z = motion.omega;

            //publish the message
            odom_pub.publish(odom);
        }

        last_time = current_time;
        r.sleep();
    }

    closeSerialPort(serialPort);
    return 0;
}

