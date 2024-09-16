#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Multiply the position and orientation values by -1
    msg.pose.pose.position.x *= -1
    msg.pose.pose.position.y *= -1
    msg.pose.pose.position.z *= 1
    
    msg.twist.twist.linear.x *= -1
    msg.twist.twist.linear.y *= -1
    msg.twist.twist.linear.z *= -1
    
    msg.twist.twist.angular.x *= 1
    msg.twist.twist.angular.y *= 1
    msg.twist.twist.angular.z *= 1
    
    # Publish the modified odometry
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('invert_odom')
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)
    rospy.spin()