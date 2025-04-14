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

# def wait_for_arduino_ready():
#     print("Waiting for 'f' from Arduino to proceed...")
#     start_time = time.time()  # Record the start time
#     timeout = 10  # Set timeout to 10 seconds
    
#     while True:
#         received_char = receive_from_arduino()
#         if received_char == 'f':  # Wait until 'f' is received
#             print("Received 'f' from Arduino, proceeding to the next step.")
#             break
#         elif time.time() - start_time > timeout:  # Check if 10 seconds have passed
#             print("Timeout reached, proceeding without receiving 'f'.")
#             break

def A_take_ball_motor():
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

    def turn_left(duration, speed):
        vel_msg.linear.x = 0.0  # No forward movement
        vel_msg.angular.z = -speed  # Turn left
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_publisher.publish(vel_msg)
            rate.sleep()

    def turn_right(duration, speed):
        vel_msg.linear.x = 0.0  # No forward movement
        vel_msg.angular.z = speed  # Turn right
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_publisher.publish(vel_msg)
            rate.sleep()

    def stop_robot():
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)

    def A_take_ball():
        move_forward(6.1, 0.25)  # Move forward for 6.1 seconds at 0.25 m/s
        stop_robot()
        sleep(1)   

        turn_right(7.85, 0.2)    # Turn right for 3.09 seconds at 0.5 rad/s
        stop_robot()
        sleep(1)

        move_forward(2.5, 0.1)  # Move forward for 6.1 seconds at 0.25 m/s
        stop_robot()
        sleep(1)

        send_to_arduino('a')
        sleep(13)

        move_forward(6.0, 0.1)
        stop_robot()
        sleep(1) 

        send_to_arduino('a')
        sleep(13)

        move_forward(6.0, 0.1)
        stop_robot()
        sleep(1) 

        send_to_arduino('a')
        sleep(13)

        move_forward(6.0, 0.1)
        stop_robot()
        sleep(1) 

        send_to_arduino('a')
        sleep(13)

        move_forward(6.0, 0.1)
        stop_robot()
        sleep(1) 

        send_to_arduino('a')
        sleep(13)

        move_forward(4.0, 0.25)  # Move forward for 6.1 seconds at 0.25 m/s
        stop_robot()
        sleep(1) 

        turn_right(1, 0.1) 
        stop_robot()
        sleep(1)

        move_forward(8.5, 0.35)
        stop_robot()
        sleep(1)

    

    A_take_ball()


def main():
    rospy.init_node('main_control')
    
    A_take_ball_motor()




if __name__ == '__main__':
    main()