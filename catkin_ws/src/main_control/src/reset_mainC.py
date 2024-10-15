#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from time import sleep

def s_shape_motor():
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

    def s_shape():
        move_forward(7.4, 0.25)  # Move forward for 7.4 seconds at 0.25 m/s
        stop_robot()
        sleep(1)                 # 1-second delay

        turn_left(7.85, 0.2)     # Turn left for 3.09 seconds at 0.5 rad/s
        stop_robot()
        sleep(1)                 # 1-second delay

        move_forward(10.2, 0.25) # Move forward for 10.2 seconds at 0.25 m/s
        stop_robot()
        sleep(1)                 # 1-second delay

        turn_right(7.85, 0.2)    # Turn right for 3.09 seconds at 0.5 rad/s
        stop_robot()
        sleep(1)                 # 1-second delay

        move_forward(6.1, 0.25)  # Move forward for 6.1 seconds at 0.25 m/s
        stop_robot()
        sleep(1)                 # 1-second delay

        turn_right(7.85, 0.2)    # Turn right for 3.09 seconds at 0.5 rad/s
        stop_robot()
        sleep(1)                 # 1-second delay

        move_forward(10.2, 0.25) # Move forward for 10.2 seconds at 0.25 m/s
        stop_robot()
        sleep(1)                 # 1-second delay

        turn_left(7.85, 0.2)     # Turn left for 3.09 seconds at 0.5 rad/s
        stop_robot()
        sleep(1)                 # 1-second delay

        move_forward(10.4, 0.25) # Move forward for 10.4 seconds at 0.25 m/s
        stop_robot()

    # Call the s_shape function
    s_shape()

def main():
    rospy.init_node('main_control')
    s_shape_motor()

if __name__ == '__main__':
    main()
