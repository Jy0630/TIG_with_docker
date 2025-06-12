#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point

class MecanumController:
    def __init__(self):
        rospy.init_node('mecanum_control_node', anonymous=True)
        rospy.loginfo("運動控制節點 'mecanum_control_node' 已啟動")

        # Subscribe to line detection data
        self.detection_sub = rospy.Subscriber('/line_detect/detection_data', Point, self.control_callback)
        
        # publish velocity commands
        self.cmd_vel_pub = rospy.Publisher('/dlv/cmd_vel', Twist, queue_size=1)
        
        self.load_params()

    def load_params(self):
        """從ROS Parameter Server加載所有控制參數"""
        # 像素偏移閾值
        self.pixel_thresh_1 = rospy.get_param('~pixel_thresh_1', 20)
        self.pixel_thresh_2 = rospy.get_param('~pixel_thresh_2', 120)
        self.pixel_thresh_3 = rospy.get_param('~pixel_thresh_3', 220)
        
        # 角度偏移閾值 (單位: 度)
        self.angle_thresh_1 = rospy.get_param('~angle_thresh_1', 30)
        self.angle_thresh_2 = rospy.get_param('~angle_thresh_2', 50)
        
        # 前進速度
        self.fwd_speed_normal = rospy.get_param('~fwd_speed_normal', 0.4)
        self.fwd_speed_correct_1 = rospy.get_param('~fwd_speed_correct_1', 0.3)
        self.fwd_speed_correct_2 = rospy.get_param('~fwd_speed_correct_2', 0.2)
        self.fwd_speed_correct_3 = rospy.get_param('~fwd_speed_correct_3', 0.1)
        self.fwd_speed_rotate_1 = rospy.get_param('~fwd_speed_rotate_1', 0.2)
        self.fwd_speed_rotate_2 = rospy.get_param('~fwd_speed_rotate_2', 0.1)
        
        # 平移速度
        self.lat_speed_correct_1 = rospy.get_param('~lat_speed_correct_1', 0.1)
        self.lat_speed_correct_2 = rospy.get_param('~lat_speed_correct_2', 0.2)
        self.lat_speed_correct_3 = rospy.get_param('~lat_speed_correct_3', 0.2)
        
        # 旋轉速度
        self.rot_speed_correct_1 = rospy.get_param('~rot_speed_correct_1', 0.3)
        self.rot_speed_correct_2 = rospy.get_param('~rot_speed_correct_2', 0.4)
        
        rospy.loginfo("所有控制參數已成功加載")

    def control_callback(self, data):
        pixel_deviation = data.x
        angle_deviation = data.y
        
        vel_msg = Twist()
        
        abs_pixel_dev = abs(pixel_deviation)
        abs_angle_dev = abs(angle_deviation)
        
        # 核心控制邏輯：優先處理平移，在中線附近再處理角度
        
        if abs_pixel_dev > self.pixel_thresh_3:
            # 第三階段矯正 (偏移最大)
            vel_msg.linear.x = self.fwd_speed_correct_3
            # 偏左(dev<0)則向右(y<0)平移, 偏右(dev>0)則向左(y>0)平移
            vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_3
            vel_msg.angular.z = 0
            
        elif abs_pixel_dev > self.pixel_thresh_2:
            # 第二階段矯正
            vel_msg.linear.x = self.fwd_speed_correct_2
            vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_2
            vel_msg.angular.z = 0
            
        elif abs_pixel_dev > self.pixel_thresh_1:
            # 第一階段矯正
            vel_msg.linear.x = self.fwd_speed_correct_1
            vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_1
            vel_msg.angular.z = 0
            
        else:
            # 幾乎在線上，此時檢查並矯正角度
            vel_msg.linear.y = 0 # 無需平移
            
            if abs_angle_dev > self.angle_thresh_2:
                # 角度偏移過大
                vel_msg.linear.x = self.fwd_speed_rotate_2
                # 角度偏右(angle>0)需逆時針(z>0)旋轉, 角度偏左(angle<0)需順時針(z<0)旋轉
                vel_msg.angular.z = np.sign(angle_deviation) * self.rot_speed_correct_2
                
            elif abs_angle_dev > self.angle_thresh_1:
                # 角度偏移較小
                vel_msg.linear.x = self.fwd_speed_rotate_1
                vel_msg.angular.z = np.sign(angle_deviation) * self.rot_speed_correct_1
                
            else:
                # 路線筆直，正常前進
                vel_msg.linear.x = self.fwd_speed_normal
                vel_msg.angular.z = 0
                
        # 發布最終的速度指令
        self.cmd_vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        mc = MecanumController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mecanum_control_node shutted down")
