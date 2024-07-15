#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def handle_odom_to_base(msg):
    br = tf.TransformBroadcaster()

    translation = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    rotation = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    br.sendTransform(translation, rotation, rospy.Time.now(), "base_footprint", "odom")

if __name__ == "__main__" :
    rospy.init_node("odom_to_base_tf_broadcaster")
    rospy.Subscriber('/scanmatch_odom', Odometry, handle_odom_to_base)
    rospy.spin()
