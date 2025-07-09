#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from line_follower.srv import SetLineFollower
from wall_localization.srv import SetWallNavigation

from object_detect.srv import DetectOrangeGoal
import numpy as np


from  object_detect.srv import DetectObjects
from object_detect.srv import  DetectCoffee

class MainController:

    def __init__(self):
        rospy.init_node('main_control_node')
        rospy.loginfo("Main Controller Node Started.")

        # 等待所有服務啟動
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('set_line_follower')
        self.line_follower_client = rospy.ServiceProxy('set_line_follower', SetLineFollower)
        
        rospy.wait_for_service('navigate_by_wall')
        self.wall_nav_client = rospy.ServiceProxy('navigate_by_wall', SetWallNavigation)
        
        rospy.wait_for_service('detect_orange_goal')
        self.orange_detect_client = rospy.ServiceProxy('detect_orange_goal', DetectOrangeGoal)
        
        rospy.loginfo("All services are ready.")

        self.last_intersection_type = None
        self.intersection_sub = rospy.Subscriber('/line_detect/intersection_type', String, self.intersection_callback)
        rospy.loginfo("Subscribed to '/line_detect/intersection_type'.")

        #object detection
        rospy.wait_for_service('detect_objects_srv')
        self.detect_client = rospy.ServiceProxy('detect_objects_srv', DetectObjects)

        #coffeesupply
        rospy.wait_for_service('detect_coffee_srv')
        self.detect_client = rospy.ServiceProxy('detect_coffee_srv',DetectCoffee)

        self.run_competition_flow()

    def intersection_callback(self, msg):
        self.last_intersection_type = msg.data

    def toggle_line_follower(self, enable):
        """Start or stop the line follower service."""
        try:
            response = self.line_follower_client(enable)
            rospy.loginfo(f"Line follower toggled to {enable}: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to 'set_line_follower' failed: {e}")
            return False

    def follow_line_until_t_junction(self, timeout_sec=60.0):
        """Line following task until a T-junction is detected."""
        rospy.loginfo("Executing task: Follow line until T-junction...")
        self.last_intersection_type = ""
        if not self.toggle_line_follower(True):
            return False
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.last_intersection_type == "T_JUNCTION":
                rospy.loginfo("T-junction detected! Stopping.")
                return self.toggle_line_follower(False)
            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logerr(f"Timeout ({timeout_sec}s) reached. Stopping.")
                return self.toggle_line_follower(False)
            rate.sleep()
        return self.toggle_line_follower(False)

    def navigate_by_wall(self, front=-1.0, rear=-1.0, left=-1.0, right=-1.0, angle=-1.0, align_wall=""):
        """Control the robot to navigate by wall."""
        rospy.loginfo(f"Executing task: Wall navigation with params: front={front}, right={right}, angle={angle}...")
        try:
            response = self.wall_nav_client(
                target_front_distance=front, target_rear_distance=rear,
                target_left_distance=left, target_right_distance=right,
                target_angle=angle, align_to_wall=align_wall
            )
            rospy.loginfo(f"Navigation result: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to 'navigate_by_wall' failed: {e}")
            return False


    def detect_and_navigate_to_orange(self):
        """Detect orange and navigate to the detected goal."""
        rospy.loginfo("Executing task: Detect and Navigate to Orange...")
        
        # 1. Orange detect service
        try:
            rospy.loginfo("Calling orange detection service...")
            detect_response = self.orange_detect_client()
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to 'detect_orange_goal' failed: {e}")
            return False

        if not detect_response.success:
            rospy.logwarn("Orange detection failed. Skipping navigation.")
            return False

        rospy.loginfo("Orange goal found! Translating coordinates to navigation commands.")
        
        # 2. Translate the detected coordinates into navigation commands
        target_front = detect_response.target_y
        target_right = detect_response.target_x
        target_angle_deg = np.degrees(detect_response.target_final_yaw)
        
        # 3. Navigate to the target point
        rospy.loginfo(f"Step 1: Navigating to point (front: {target_front:.2f}m, right: {target_right:.2f}m).")
        if not self.navigate_by_wall(front=target_front, right=target_right, angle=0.0, align_wall="right"):
            rospy.logerr("Failed to navigate to the target point.")
            return False

        # 4. Turning to the final angle
        rospy.loginfo(f"Step 2: Rotating to final angle ({target_angle_deg:.2f} degrees).")
        if not self.navigate_by_wall(angle=target_angle_deg):
            rospy.logerr("Failed to rotate to the final angle.")
            return False
            
        rospy.loginfo("Successfully navigated to the orange goal!")
        return True

        
    #咖啡偵測coffee detect用來判斷菜單內容
    def detect_objects(self):
        try:
                resp = self.detect_client()
                detections = json.loads(resp.detection_result_json)
                return detections
        except rospy.ServiceException as e:
                rospy.logerr(f"Object detection service call failed: {e}")
                return []
        
    #coffeesupply 根據物件偵測結果（位置與名稱）判斷咖啡要放在哪一張桌子上，然後發佈 ROS topic 指令給其他控制單元執行
    def detect_coffee(self):
        try:
                resp = self.coffee_client()  # 呼叫 detect_coffee_srv (無參數)
                if resp.success:
                    return {
                        'target_name': resp.target_name,
                        'target_xyz': list(resp.target_xyz)
                    }
                else:
                    rospy.loginfo("Coffee detection succeeded, but no valid target matched.")
                    return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Coffee detection service call failed: {e}")
            return None

    def run_competition_flow(self):
        current_state = "CROSS_BRIDGE"

        while not rospy.is_shutdown():
            rospy.loginfo(f"====== Current State: {current_state} ======")
##########################################################################################
            if current_state == "CROSS_BRIDGE":
                if self.follow_line_until_t_junction():
                    current_state = "NAV_AFTER_BRIDGE"
                else:
                    current_state = "ERROR_RECOVERY"
##########################################################################################
            elif current_state == "NAV_AFTER_BRIDGE":
                if self.navigate_by_wall(front=2.5):
                    current_state = "DETECT_AND_NAV_TO_ORANGE"
                else:
                    current_state = "ERROR_RECOVERY"
##########################################################################################
            elif current_state == "DETECT_AND_NAV_TO_ORANGE":
                if self.detect_and_navigate_to_orange():
                    current_state = "COMPETITION_FINISH"
                else:
                    current_state = "ERROR_RECOVERY"
##########################################################################################
            elif current_state == "COMPETITION_FINISH":
                rospy.loginfo("All tasks completed successfully!")
                break

            elif current_state == "ERROR_RECOVERY":
                rospy.logerr("A task failed. Entering error recovery mode.")
                break
            
            rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        MainController()
    except rospy.ROSInterruptException:
        pass
