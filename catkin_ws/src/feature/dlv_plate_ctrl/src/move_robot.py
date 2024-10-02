#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time

def move_robot():
    rospy.init_node('move_robot', anonymous=True)

    pub = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
    

    rate = rospy.Rate(10)
    
    move_cmd = Twist()
    move_cmd.linear.x = 0.2 
    move_cmd.angular.z = 0.0

    start_time = time.time()
    while time.time() - start_time < 3:
        pub.publish(move_cmd)
        rate.sleep()

    move_cmd.linear.x = 0.0 
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
