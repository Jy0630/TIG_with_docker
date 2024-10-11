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
