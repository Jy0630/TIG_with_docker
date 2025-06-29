#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
# 引入我們剛剛定義的 Service 類型
from line_follower.srv import SetLineFollower, SetLineFollowerResponse

class MecanumController:
    def __init__(self):
        rospy.init_node('line_following_node', anonymous=True)
        rospy.loginfo("'line_following_node' (Service Server) started")
        
        self.cmd_vel_pub = rospy.Publisher('/dlv/cmd_vel', Twist, queue_size=1)
        
        # **核心修改 1: 初始化時不訂閱**
        # self.detection_sub = rospy.Subscriber('/line_detect/detection_data', Point, self.control_callback)
        self.detection_sub = None  # 將訂閱者物件儲存起來，以便後續取消
        self.is_active = False     # 新增一個狀態旗標，方便管理

        self.load_params()
        
        # **核心修改 2: 建立 Service Server**
        # 當收到名為 'set_line_follower' 的服務請求時，呼叫 self.handle_set_line_follower 函式
        self.service = rospy.Service('set_line_follower', SetLineFollower, self.handle_set_line_follower)
        rospy.loginfo("Service 'set_line_follower' is ready.")

    # **核心修改 3: 撰寫 Service 的處理函式**
    def handle_set_line_follower(self, req):
        """
        這個函式會在收到 Service 請求時被呼叫。
        """
        # 如果請求是要啟動 (enable=True)
        if req.enable:
            # 只有在目前是停止狀態時才需要啟動
            if not self.is_active:
                rospy.loginfo("Received request to ENABLE line follower.")
                # 建立訂閱者，開始接收循線數據並呼叫 control_callback
                self.detection_sub = rospy.Subscriber('/line_detect/detection_data', Point, self.control_callback)
                self.is_active = True
                return SetLineFollowerResponse(success=True, message="Line follower has been enabled.")
            else:
                rospy.logwarn("Line follower is already enabled. No action taken.")
                return SetLineFollowerResponse(success=True, message="Already enabled.")

        # 如果請求是要停止 (enable=False)
        else:
            # 只有在目前是啟動狀態時才需要停止
            if self.is_active:
                rospy.loginfo("Received request to DISABLE line follower.")
                # 非常重要：取消訂閱！這樣 control_callback 就不會再被觸發
                if self.detection_sub:
                    self.detection_sub.unregister()
                    self.detection_sub = None
                
                # 非常重要：發送一個零速指令，確保機器人完全停止
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
                self.is_active = False
                return SetLineFollowerResponse(success=True, message="Line follower has been disabled and robot stopped.")
            else:
                rospy.logwarn("Line follower is already disabled. No action taken.")
                return SetLineFollowerResponse(success=True, message="Already disabled.")

    def load_params(self):
        # ... 這裡的參數載入邏輯完全不用變 ...
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
        
        rospy.loginfo("Parameters loaded successfully.")

    def control_callback(self, data):
        # 這個函式本身完全不用變，因為它的執行現在受到訂閱/取消訂閱的控制
        pixel_deviation = data.x
        angle_deviation = data.y
        vel_msg = Twist()
        abs_pixel_dev = abs(pixel_deviation)
        abs_angle_dev = abs(angle_deviation)
        
        # ... (你原有的完整控制邏輯) ...
        if abs_pixel_dev > self.pixel_thresh_3:
            vel_msg.linear.x = self.fwd_speed_correct_3
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
        rospy.spin()  # 保持節點運行，以便它可以持續接收服務請求
    except rospy.ROSInterruptException:
        rospy.loginfo("line_following_node shutted down")
