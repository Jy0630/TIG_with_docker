#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point

class MecanumController:
    def __init__(self):
        rospy.init_node('mecanum_control_node', anonymous=True)
        rospy.loginfo("'mecanum_control_node' started")
        self.detection_sub = rospy.Subscriber('/line_detect/detection_data', Point, self.control_callback)
        
        self.cmd_vel_pub = rospy.Publisher('/dlv/cmd_vel', Twist, queue_size=1)
        
        self.load_params()

    def load_params(self):

        # Pixel thresholds (單位: 像素)
        self.pixel_thresh_1 = rospy.get_param('~pixel_thresh_1', 20)
        self.pixel_thresh_2 = rospy.get_param('~pixel_thresh_2', 120)
        self.pixel_thresh_3 = rospy.get_param('~pixel_thresh_3', 220)
        
        # Angle thresholds (單位: 度)
        self.angle_thresh_1 = rospy.get_param('~angle_thresh_1', 30)
        self.angle_thresh_2 = rospy.get_param('~angle_thresh_2', 50)
        
        # Forward speed
        self.fwd_speed_normal = rospy.get_param('~fwd_speed_normal', 0.4)
        self.fwd_speed_correct_1 = rospy.get_param('~fwd_speed_correct_1', 0.3)
        self.fwd_speed_correct_2 = rospy.get_param('~fwd_speed_correct_2', 0.2)
        self.fwd_speed_correct_3 = rospy.get_param('~fwd_speed_correct_3', 0.1)
        self.fwd_speed_rotate_1 = rospy.get_param('~fwd_speed_rotate_1', 0.2)
        self.fwd_speed_rotate_2 = rospy.get_param('~fwd_speed_rotate_2', 0.1)
        
        # Lateral speed
        self.lat_speed_correct_1 = rospy.get_param('~lat_speed_correct_1', 0.1)
        self.lat_speed_correct_2 = rospy.get_param('~lat_speed_correct_2', 0.2)
        self.lat_speed_correct_3 = rospy.get_param('~lat_speed_correct_3', 0.2)
        
        # Rotation speed
        self.rot_speed_correct_1 = rospy.get_param('~rot_speed_correct_1', 0.3)
        self.rot_speed_correct_2 = rospy.get_param('~rot_speed_correct_2', 0.4)
        
        rospy.loginfo("Loaded parameters successfully:")

    def control_callback(self, data):
        pixel_deviation = data.x
        angle_deviation = data.y
        
        vel_msg = Twist()
        
        abs_pixel_dev = abs(pixel_deviation)
        abs_angle_dev = abs(angle_deviation)
        
        # Deal with lateral first, then rotate
        
        if abs_pixel_dev > self.pixel_thresh_3:
            vel_msg.linear.x = self.fwd_speed_correct_3
            # if pixel_deviation > 0, robot is to the right of the line, move left
            vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_3
            vel_msg.angular.z = 0
            
        elif abs_pixel_dev > self.pixel_thresh_2:

            vel_msg.linear.x = self.fwd_speed_correct_2
            vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_2
            vel_msg.angular.z = 0
            
        elif abs_pixel_dev > self.pixel_thresh_1:

            vel_msg.linear.x = self.fwd_speed_correct_1
            vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_1
            vel_msg.angular.z = 0
            
        else:

            vel_msg.linear.y = 0
            
            if abs_angle_dev > self.angle_thresh_2:

                vel_msg.linear.x = self.fwd_speed_rotate_2
                # Angle deviation is large, rotate to correct
                vel_msg.angular.z = np.sign(angle_deviation) * self.rot_speed_correct_2
                
            elif abs_angle_dev > self.angle_thresh_1:
                vel_msg.linear.x = self.fwd_speed_rotate_1
                vel_msg.angular.z = np.sign(angle_deviation) * self.rot_speed_correct_1
                
            else:
                vel_msg.linear.x = self.fwd_speed_normal
                vel_msg.angular.z = 0
                
        self.cmd_vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        mc = MecanumController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mecanum_control_node shutted down")
