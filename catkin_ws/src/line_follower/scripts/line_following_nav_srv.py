#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from line_follower.srv import SetLineFollower, SetLineFollowerResponse
# NEW: 匯入我們新建立的 Service 類型
from line_follower.srv import SetCameraState

class MecanumController:
    def __init__(self):
        rospy.init_node('line_following_node', anonymous=True)
        rospy.loginfo("'line_following_node' (Service Server) started")
        
        self.cmd_vel_pub = rospy.Publisher('/dlv/cmd_vel', Twist, queue_size=1)
        self.detection_sub = None 
        self.is_active = False 

        self.load_params()
        
        # NEW: 設定 Service Client 以便控制攝影機
        rospy.loginfo("Waiting for '/line_detect/set_camera_state' service...")
        try:
            rospy.wait_for_service('/line_detect/set_camera_state', timeout=5.0)
            self.set_camera_state_client = rospy.ServiceProxy('/line_detect/set_camera_state', SetCameraState)
            rospy.loginfo("Service client for camera control is connected.")
        except rospy.ROSException:
            rospy.logerr("Service '/line_detect/set_camera_state' not available. Shutting down.")
            rospy.signal_shutdown("Camera control service not found.")
            return

        self.service = rospy.Service('set_line_follower', SetLineFollower, self.handle_set_line_follower)
        rospy.loginfo("Service 'set_line_follower' is ready.")

    def handle_set_line_follower(self, req):
        if req.enable:
            if not self.is_active:
                rospy.loginfo("Received request to ENABLE line follower.")
                # NEW: 啟用攝影機
                try:
                    resp = self.set_camera_state_client(True)
                    if not resp.success:
                        rospy.logerr("Failed to enable camera: %s", resp.message)
                        return SetLineFollowerResponse(success=False, message="Failed to enable camera.")
                except rospy.ServiceException as e:
                    rospy.logerr("Service call to enable camera failed: %s", e)
                    return SetLineFollowerResponse(success=False, message="Service call failed.")
                
                self.detection_sub = rospy.Subscriber('/line_detect/detection_data', Point, self.control_callback)
                self.is_active = True
                self.is_rotating = False # 重置旋轉狀態
                return SetLineFollowerResponse(success=True, message="Line follower and camera have been enabled.")
            else:
                rospy.logwarn("Line follower is already enabled.")
                return SetLineFollowerResponse(success=True, message="Already enabled.")
        else:
            if self.is_active:
                rospy.loginfo("Received request to DISABLE line follower.")
                if self.detection_sub:
                    self.detection_sub.unregister()
                    self.detection_sub = None
                
                stop_msg = Twist()
                self.cmd_vel_pub.publish(stop_msg)
                
                self.is_active = False
                
                # NEW: 關閉攝影機
                try:
                    self.set_camera_state_client(False)
                    rospy.loginfo("Camera shutdown command sent.")
                except rospy.ServiceException as e:
                    rospy.logwarn("Service call to disable camera failed: %s", e)
                
                return SetLineFollowerResponse(success=True, message="Line follower has been disabled and robot stopped. Camera shut down.")
            else:
                rospy.logwarn("Line follower is already disabled.")
                return SetLineFollowerResponse(success=True, message="Already disabled.")

    def load_params(self):
        self.pixel_thresh_normal = rospy.get_param('~pixel_thresh_normal', 50)
        self.pixel_thresh_large = rospy.get_param('~pixel_thresh_large', 120)
        self.angle_thresh_rotate = rospy.get_param('~angle_thresh_rotate', 180)
        self.angle_thresh_ok = rospy.get_param('~angle_thresh_ok', 180)
        self.fwd_speed_normal = rospy.get_param('~fwd_speed_normal', 0.38)
        self.fwd_speed_correct = rospy.get_param('~fwd_speed_correct', 0.3)
        self.lat_speed_correct_small = rospy.get_param('~lat_speed_correct_small', 0.035)
        self.lat_speed_correct_large = rospy.get_param('~lat_speed_correct_large', 0.035)
        self.rot_speed_correct = rospy.get_param('~rot_speed_correct', 0.15)
        rospy.loginfo("Simplified parameters loaded successfully.")

    def control_callback(self, data):
        pixel_deviation = data.x
        angle_deviation = data.y
        vel_msg = Twist()
        abs_pixel_dev = abs(pixel_deviation)
        abs_angle_dev = abs(angle_deviation)

        if abs_angle_dev > self.angle_thresh_rotate or (self.is_rotating and abs_angle_dev > self.angle_thresh_ok):
            if not self.is_rotating:
                 rospy.logwarn("Large angle deviation detected ({:.1f} deg). Rotating in place.".format(angle_deviation))
                 self.is_rotating = True
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.angular.z = -np.sign(angle_deviation) * self.rot_speed_correct
        else:
            self.is_rotating = False
            if abs_pixel_dev <= self.pixel_thresh_normal:
                vel_msg.linear.x = self.fwd_speed_normal
                vel_msg.linear.y = 0.0
                vel_msg.angular.z = 0.0
            elif abs_pixel_dev <= self.pixel_thresh_large:
                vel_msg.linear.x = self.fwd_speed_correct
                vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_small
                vel_msg.angular.z = 0.0
            else:
                vel_msg.linear.x = self.fwd_speed_correct
                vel_msg.linear.y = -np.sign(pixel_deviation) * self.lat_speed_correct_large
                vel_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        mc = MecanumController()
        # 初始化旋轉狀態旗標
        mc.is_rotating = False
        rospy.spin() 
    except rospy.ROSInterruptException:
        rospy.loginfo("line_following_node shutted down")
