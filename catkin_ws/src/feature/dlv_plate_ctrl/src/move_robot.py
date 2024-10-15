#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

def move_robot():
    rospy.init_node('move_robot', anonymous=True)

    pub = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)
    
    # 从参数服务器获取线速度和角速度的值，默认值分别为0.1和0.0
    linear_x = rospy.get_param('~linear_x', 0.0)
    angular_z = rospy.get_param('~angular_z', 0.0)

    move_cmd = Twist()
    move_cmd.linear.x = linear_x
    move_cmd.angular.z = angular_z

    duration = rospy.get_param('~duration', 0.0)

    start_time = time.time()
    while time.time() - start_time < duration:
        pub.publish(move_cmd)
        rate.sleep()

    move_cmd.linear.x = 0.0 
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

# import rospy
# from geometry_msgs.msg import Twist
# from time import sleep

# def move_robot():
#     # Initialize the ROS node
#     rospy.init_node('move_robot_node', anonymous=True)
    
#     # Create a publisher for the 'dlv/cmd_vel' topic
#     velocity_publisher = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
    
#     # Set the loop rate (10 Hz)
#     rate = rospy.Rate(10)
    
#     # Create a Twist message
#     vel_msg = Twist()

#     def move_forward(duration, speed):
#         vel_msg.linear.x = speed  # Move forward
#         vel_msg.angular.z = 0.0  # No rotation
#         start_time = rospy.Time.now().to_sec()
#         while rospy.Time.now().to_sec() - start_time < duration:
#             velocity_publisher.publish(vel_msg)
#             rate.sleep()

#     def turn_left(duration, speed):
#         vel_msg.linear.x = 0.0  # No forward movement
#         vel_msg.angular.z = -speed  # Turn left
#         start_time = rospy.Time.now().to_sec()
#         while rospy.Time.now().to_sec() - start_time < duration:
#             velocity_publisher.publish(vel_msg)
#             rate.sleep()

#     def turn_right(duration, speed):
#         vel_msg.linear.x = 0.0  # No forward movement
#         vel_msg.angular.z = speed  # Turn right
#         start_time = rospy.Time.now().to_sec()
#         while rospy.Time.now().to_sec() - start_time < duration:
#             velocity_publisher.publish(vel_msg)
#             rate.sleep()

#     def stop_robot():
#         vel_msg.linear.x = 0.0
#         vel_msg.angular.z = 0.0
#         velocity_publisher.publish(vel_msg)

#     # Sequence of movements with a 1 second delay in between
#     move_forward(7.4, 0.25)  # Move forward for 6.1 seconds at 0.25 m/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     turn_left(3.09, 0.5)     # Turn left for 3.69 seconds at 0.5 rad/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     move_forward(10.2, 0.25) # Move forward for 10.4 seconds at 0.25 m/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     turn_right(3.09, 0.5)    # Turn right for 3.69 seconds at 0.5 rad/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     move_forward(6.1, 0.25)  # Move forward for 6.4 seconds at 0.25 m/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     turn_right(3.09, 0.5)    # Turn right for 3.69 seconds at 0.5 rad/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     move_forward(10.2, 0.25) # Move forward for 10.4 seconds at 0.25 m/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     turn_left(3.09, 0.5)     # Turn left for 3.69 seconds at 0.5 rad/s
#     stop_robot()
#     sleep(1)                 # 1-second delay

#     move_forward(10.4, 0.25) # Move forward for 10.4 seconds at 0.25 m/s
#     stop_robot()

# if __name__ == '__main__':
#     try:
#         move_robot()
#     except rospy.ROSInterruptException:
#         pass

