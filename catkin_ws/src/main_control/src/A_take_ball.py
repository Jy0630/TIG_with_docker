#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from time import sleep
import serial
import time

# 设置串口通信（修改端口号和波特率与Arduino匹配）
ser = serial.Serial('/dev/arduino', 9600, timeout=1)

# 定义发送给Arduino的函数
def send_to_arduino(message):
    if isinstance(message, str) and len(message) == 1:  # Ensure it's a single character
        ser.write(message.encode())  # Send the character
        print(f"Sent to Arduino: {message}")

def receive_from_arduino():
    while True:
        if ser.in_waiting > 0:  # Check if data is available to read
            response = ser.read(1)  # Read 1 byte from the serial buffer
            char_response = response.decode()  # Decode byte to character
            print(f"Received from Arduino: {char_response}")
            return char_response  # Return the received character

def wait_for_arduino_ready():
    print("Waiting for 'f' from Arduino to proceed...")
    while True:
        received_char = receive_from_arduino()
        if received_char == 'f':  # Wait until 'f' is received
            print("Received 'f' from Arduino, proceeding to the next step.")
            break

def move_to_final_motor():
    # Initialize the ROS node
    # rospy.init_node('move_robot_node', anonymous=True)
    
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
        send_to_arduino('a')
        wait_for_arduino_ready()
        move_forward(3.0, 0.1)
        stop_robot()
        sleep(1) 
        send_to_arduino(2)
        wait_for_arduino_ready()
        move_forward(3.0, 0.1)
        stop_robot()
        sleep(1) 
        send_to_arduino(3)
        wait_for_arduino_ready()
        move_forward(3.0, 0.1)
        stop_robot()
        sleep(1) 

    move_to_final()


def main():
    rospy.init_node('main_control')
    
    move_to_final_motor()




if __name__ == '__main__':
    main()