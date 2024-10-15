#!/usr/bin/env python

import rospy


# from dlv_plate_ctrl.s_shape_motor import s_shape

from geometry_msgs.msg import Twist
from time import sleep
import serial
import time

# 设置串口通信（修改端口号和波特率与Arduino匹配）
ser = serial.Serial('/dev/arduino', 9600, timeout=1)

# 定义发送给Arduino的函数
def send_to_arduino(message):
    if isinstance(message, int):
        ser.write(str(message).encode())  # 将整数转换为字符串并编码为字节发送
        print(f"Sent to Arduino: {message}")



def move_to_final_motor():
    # Initialize the ROS node
    rospy.init_node('move_robot_node', anonymous=True)
    
    # Create a publisher for the 'dlv/cmd_vel' topic
    velocity_publisher = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
    
    # Set the loop rate (10 Hz)
    rate = rospy.Rate(10)
    
    # Create a Twist message
    vel_msg = Twist()

    def move_forward(duration, speed):
        vel_msg.linear.x = speed  # Move forward
        vel_msg.angular.z = 0.0  # No rotation
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_publisher.publish(vel_msg)
            rate.sleep()

    def stop_robot():
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)

    def move_to_final():
        move_forward(6.1, 0.25)  # Move forward for 6.1 seconds at 0.25 m/s
        stop_robot()
        sleep(1)   
        send_to_arduino(1)
        sleep(1) #front lift time
        move_forward(3.0, 0.1)
        stop_robot()
        sleep(1) 
        send_to_arduino(2)
        sleep(1) #front lift time
        move_forward(3.0, 0.1)
        stop_robot()
        sleep(1) 
        send_to_arduino(3)
        sleep(1) #front lift time
        move_forward(3.0, 0.1)
        stop_robot()
        sleep(1) 

    move_to_final()


def main():
    rospy.init_node('main_control')
    
    move_to_final_motor()




if __name__ == '__main__':
    main()