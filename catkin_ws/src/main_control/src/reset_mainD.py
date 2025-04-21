#!/usr/bin/env python

import rospy


# from dlv_plate_ctrl.s_shape_motor import s_shape

from geometry_msgs.msg import Twist
from time import sleep

def A_to_C_motor():
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

    def A_to_C():
        move_forward(24.6, 0.25)  # Move forward for 7.4 seconds at 0.25 m/s
        stop_robot()
        sleep(1)                 # 1-second delay



    A_to_C()


def main():
    rospy.init_node('main_control')
    
    A_to_C_motor()


if __name__ == '__main__':
    main()