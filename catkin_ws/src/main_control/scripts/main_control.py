#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from line_follower.srv import SetLineFollower
from wall_localization.srv import SetWallNavigation
from geometry_msgs.msg import Twist

from object_detect.srv import DetectOrangeGoal
import numpy as np
import time

from object_detect.srv import DetectObjects
from object_detect.srv import  DetectCoffee

from std_msgs.msg import String, Float32, Bool
from step_motor.srv import SetDistance

class MainController:

    def __init__(self):
        rospy.init_node('main_control_node')
        rospy.loginfo("Main Controller Node Started.")

        # 等待所有服務啟動
        # rospy.loginfo("Waiting for services...")
        # rospy.wait_for_service('set_line_follower')
        # self.line_follower_client = rospy.ServiceProxy('set_line_follower', SetLineFollower)
        # self.cmd_vel_pub = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=10)
        # rospy.loginfo("Publisher to '/cmd_vel' created.")

        # rospy.wait_for_service('navigate_by_wall')
        self.wall_nav_client = rospy.ServiceProxy('navigate_by_wall', SetWallNavigation)
        
        # rospy.wait_for_service('detect_orange_goal')
        # self.orange_detect_client = rospy.ServiceProxy('detect_orange_goal', DetectOrangeGoal)
        self.dc_motor_ready = False
        self.ready_sub = rospy.Subscriber('/dc_ready', Bool, self.ready_callback)
        rospy.loginfo("Subscribing to /dc_ready topic for handshake.")

        self.height_pub = rospy.Publisher('/slider_setpoint', Float32, queue_size=10, latch=True)
        rospy.loginfo("Created Publisher to /slider_setpoint for DC motor control.")

        # 訂閱者：用於接收來自 Arduino 的 /slider_current_height 回報
        self.current_height = None # 用於儲存最新的高度回報值
        self.height_sub = rospy.Subscriber('/slider_current_height', Float32, self.height_callback)
        rospy.loginfo("Created Subscriber to /slider_current_height for feedback.")
        
        rospy.loginfo("All services are ready.")

        # self.last_intersection_type = None
        # self.intersection_sub = rospy.Subscriber('/line_detect/intersection_type', String, self.intersection_callback)
        # rospy.loginfo("Subscribed to '/line_detect/intersection_type'.")

        #object detection
        # rospy.wait_for_service('detect_objects_srv')
        # self.detect_client = rospy.ServiceProxy('detect_objects_srv', DetectObjects)

        #coffeesupply
        # rospy.wait_for_service('detect_coffee_srv')
        # self.detect_client = rospy.ServiceProxy('detect_coffee_srv',DetectCoffee)

        self.run_competition_flow()

    # def intersection_callback(self, msg):
    #     self.last_intersection_type = msg.data

    # def toggle_line_follower(self, enable):
    #     """Start or stop the line follower service."""
    #     try:
    #         response = self.line_follower_client(enable)
    #         rospy.loginfo(f"Line follower toggled to {enable}: {response.message}")
    #         return response.success
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call to 'set_line_follower' failed: {e}")
    #         return False

    # def follow_line_until_t_junction(self, timeout_sec=35.0):
    #     """Line following task until a T-junction is detected."""
    #     rospy.loginfo("Executing task: Follow line until T-junction...")
    #     self.last_intersection_type = ""
    #     if not self.toggle_line_follower(True):
    #         return False
    #     start_time = rospy.Time.now()
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         if self.last_intersection_type == "T_JUNCTION" or self.last_intersection_type == "LEFT_FORK" or self.last_intersection_type == "RIGHT_FORK":
    #             rospy.loginfo("Intersection detected! Stopping.")
    #             return self.toggle_line_follower(False)
    #         if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
    #             rospy.logerr(f"Timeout ({timeout_sec}s) reached. Stopping.")
    #             return self.toggle_line_follower(False)
    #         rate.sleep()
    #     return self.toggle_line_follower(False)

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

# 在 MainController 類別中，與您現有的 navigate_by_wall 函式並列

    def navigate_by_odometry(self, forward=0.0, left=0.0, angle=0.0):
        """
        基於里程計的導航函式。
        所有距離單位為米(m)，角度單位為度(deg)。
        """
        rospy.loginfo(f"Executing ODOMETRY navigation: move forward={forward}m, left={left}m, turn angle={angle}deg")
        try:
        # 關鍵：將 use_odometry 設為 True
            response = self.wall_nav_client(
                target_front_distance=forward if forward > 0 else -1.0,
                target_rear_distance=-forward if forward < 0 else -1.0,
                target_left_distance=left if left > 0 else -1.0,
                target_right_distance=-left if left < 0 else -1.0,
                target_angle=angle,
                align_to_wall="", # 里程計模式下，此參數無效
                use_odometry=True  # <--- 模式切換的總開關！
            )
        
            if response.success:
                rospy.loginfo(f"Odometry navigation successful: {response.message}")
                return True
            else:
                rospy.logerr(f"Odometry navigation failed: {response.message}")
                return False

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call for odometry navigation failed: {e}")
            return False
        
    # def move_for_duration(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=1.0):
    #         """
    #         以指定速度移動特定時間。
    #         :param linear_x: x 軸線速度 (m/s)
    #         :param linear_y: y 軸線速度 (m/s)
    #         :param angular_z: z 軸角速度 (rad/s)
    #         :param duration: 移動持續時間 (秒)
    #         """
    #         rospy.loginfo(f"Executing move_for_duration: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}, duration={duration}s")
            
    #         # 建立 Twist 訊息
    #         vel_msg = Twist()
    #         vel_msg.linear.x = linear_x
    #         vel_msg.linear.y = linear_y
    #         vel_msg.angular.z = angular_z
            
    #         # 設定發布頻率
    #         rate = rospy.Rate(10) # 10 Hz
            
    #         # 記錄開始時間
    #         start_time = rospy.Time.now()
            
    #         # 在指定時間內持續發布速度指令
    #         while (rospy.Time.now() - start_time).to_sec() < duration:
    #             if rospy.is_shutdown():
    #                 break
    #             self.cmd_vel_pub.publish(vel_msg)
    #             rate.sleep()
                
    #         # 時間到後，發布停止指令
    #         stop_msg = Twist()
    #         self.cmd_vel_pub.publish(stop_msg)
    #         rospy.loginfo("Movement duration ended. Stopping robot.")
    #         return True # 表示執行成功


    # def detect_and_navigate_to_orange(self, search_timeout_sec=120.0):
    #     """
    #     Detects an orange goal, translates its absolute world coordinates
    #     into relative wall distances, and navigates to goal.
    #     """

    #     origin_dist_to_right_wall = rospy.get_param("~origin_to_right_wall_dist", 0.5)  # x1 (meters)
    #     origin_dist_to_front_wall = rospy.get_param("~origin_to_front_wall_dist", 3.0)  # y1 (meters)

    #     rospy.loginfo("[GOAL轉換參數] Origin→右牆距離: %.2fm, Origin→前牆距離: %.2fm" %
    #               (origin_dist_to_right_wall, origin_dist_to_front_wall))

    #     start_time = rospy.Time.now()
    #     detect_response = None

    #     while not rospy.is_shutdown():
    #         if (rospy.Time.now() - start_time).to_sec() > search_timeout_sec:
    #             rospy.logerr(f"Search timed out after {search_timeout_sec}s. Could not find orange goal.")
    #             return False

    #         rospy.loginfo_throttle(5, "Continuously searching for orange pair...")
    #         try:
    #             response = self.orange_detect_client()
    #             if response.success:
    #                 rospy.loginfo("Orange goal found! Proceeding to translation and navigation.")
    #                 detect_response = response
    #                 break
    #             else:
    #                 rospy.sleep(1.0)
    #         except rospy.ServiceException as e:
    #             rospy.logerr(f"Service call to 'detect_orange_goal' failed: {e}. Retrying in 2 seconds...")
    #             rospy.sleep(2.0)

    #     if detect_response is None:
    #         return False

    #     world_x = detect_response.target_x
    #     world_y = detect_response.target_y

    #     nav_target_right = abs(world_x) + origin_dist_to_right_wall
    #     nav_target_front = origin_dist_to_front_wall - world_y

    #     rospy.loginfo("Coordinate Translation:")
    #     rospy.loginfo(f"  - Detected World Goal (X, Y): ({world_x:.3f}, {world_y:.3f})")
    #     rospy.loginfo(f"  - ==> Nav Goal (target_right, target_front, target_angle): ({nav_target_right:.3f}, {nav_target_front:.3f}, {np.degrees(detect_response.target_final_yaw):.3f})")

    #     if not self.navigate_by_wall(front=nav_target_front, right=nav_target_right ,target_angle=0.0, align_wall="right"):
    #         rospy.logerr("Failed to navigate to the target point.")
    #         return False

    #     target_angle_deg = np.degrees(detect_response.target_final_yaw)
    #     if not self.navigate_by_wall(angle=target_angle_deg):
    #         rospy.logerr("Failed to rotate to the final angle.")
    #         return False
            
        #     if response.success:
        #         rospy.loginfo(f"Odometry navigation successful: {response.message}")
        #         return True
        #     else:
        #         rospy.logerr(f"Odometry navigation failed: {response.message}")
        #         return False

        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Service call for odometry navigation failed: {e}")
        #     return False


    # def detect_and_navigate_to_orange(self, search_timeout_sec=120.0):
    #     """
    #     Detects an orange goal, translates its absolute world coordinates
    #     into relative wall distances, and navigates to goal.
    #     """

    #     origin_dist_to_right_wall = rospy.get_param("~origin_to_right_wall_dist", 0.5)  # x1 (meters)
    #     origin_dist_to_front_wall = rospy.get_param("~origin_to_front_wall_dist", 3.0)  # y1 (meters)

    #     rospy.loginfo("[GOAL轉換參數] Origin→右牆距離: %.2fm, Origin→前牆距離: %.2fm" %
    #               (origin_dist_to_right_wall, origin_dist_to_front_wall))

    #     start_time = rospy.Time.now()
    #     detect_response = None

    #     while not rospy.is_shutdown():
    #         if (rospy.Time.now() - start_time).to_sec() > search_timeout_sec:
    #             rospy.logerr(f"Search timed out after {search_timeout_sec}s. Could not find orange goal.")
    #             return False

    #         rospy.loginfo_throttle(5, "Continuously searching for orange pair...")
    #         try:
    #             response = self.orange_detect_client()
    #             if response.success:
    #                 rospy.loginfo("Orange goal found! Proceeding to translation and navigation.")
    #                 detect_response = response
    #                 break
    #             else:
    #                 rospy.sleep(1.0)
    #         except rospy.ServiceException as e:
    #             rospy.logerr(f"Service call to 'detect_orange_goal' failed: {e}. Retrying in 2 seconds...")
    #             rospy.sleep(2.0)

    #     if detect_response is None:
    #         return False

    #     world_x = detect_response.target_x
    #     world_y = detect_response.target_y

    #     nav_target_right = abs(world_x) + origin_dist_to_right_wall
    #     nav_target_front = origin_dist_to_front_wall - world_y

    #     rospy.loginfo("Coordinate Translation:")
    #     rospy.loginfo(f"  - Detected World Goal (X, Y): ({world_x:.3f}, {world_y:.3f})")
    #     rospy.loginfo(f"  - ==> Nav Goal (target_right, target_front, target_angle): ({nav_target_right:.3f}, {nav_target_front:.3f}, {np.degrees(detect_response.target_final_yaw):.3f})")

    #     if not self.navigate_by_wall(front=nav_target_front, right=nav_target_right ,target_angle=0.0, align_wall="right"):
    #         rospy.logerr("Failed to navigate to the target point.")
    #         return False

    #     target_angle_deg = np.degrees(detect_response.target_final_yaw)
    #     if not self.navigate_by_wall(angle=target_angle_deg):
    #         rospy.logerr("Failed to rotate to the final angle.")
    #         return False
            
    #     rospy.loginfo("Successfully navigated to the orange goal!")
    #     return True

    #咖啡偵測coffee detect用來判斷菜單內容
    # def detect_objects(self):
    #     try:
    #         resp = self.detect_client()
    #         detections = json.loads(resp.detection_result_json)
    #         return detections
    #     except rospy.ServiceException as e:
    #             rospy.logerr(f"Object detection service call failed: {e}")
    #             return []
        
    #coffeesupply 根據物件偵測結果（位置與名稱）判斷咖啡要放在哪一張桌子上，然後發佈 ROS topic 指令給其他控制單元執行
    # def detect_coffee(self):
    #     try:
    #         resp = self.coffee_client()  # 呼叫 detect_coffee_srv (無參數)
    #         if resp.success:
    #             return {
    #                 'target_name': resp.target_name,
    #                 'target_xyz': list(resp.target_xyz)
    #             }
    #         else:
    #             rospy.loginfo("Coffee detection succeeded, but no valid target matched.")
    #             return None
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Coffee detection service call failed: {e}")
    #         return None

    #步進距離
    def call_set_distance(self, motor_id, distance):
        service_name = 'cmd_distance_srv'
        rospy.wait_for_service(service_name)
        try:
            service_proxy = rospy.ServiceProxy(service_name, SetDistance)
            resp = service_proxy(motor_id, distance)
            return resp.result == f"motor{motor_id}_end"
        except rospy.ServiceException as e:
            rospy.logerr(f"呼叫 {service_name} 失敗: {e}")
        return False
    
    def height_callback(self, msg):
        """
        當收到 /slider_current_height 的新訊息時，此函式會被自動呼叫。
        它的功能是更新儲存的當前高度值。
        """
        self.current_height = msg.data

    def ready_callback(self, msg):
        """當收到 /dc_ready 的訊息時，更新就緒狀態旗標。"""
        if msg.data:
            self.dc_motor_ready = True
            rospy.loginfo("Received 'dc_ready' signal from Arduino. DC motor is ready.")
            # 我們可以取消訂閱，因為這是一個一次性的信號
            self.ready_sub.unregister()

    # ==============================================================
    # ==             致動器控制與等待函式 (Actuator Control)          ==
    # ==============================================================

    # --- NEW: 等待 Arduino 就緒的專用函式 ---
    def wait_for_dc_motor_ready(self, timeout_sec=10.0):
        """等待直到收到來自 Arduino 的 /dc_ready 信號。"""
        rospy.loginfo("Waiting for DC motor node to publish ready signal...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not self.dc_motor_ready and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logerr(f"Timeout! Did not receive /dc_ready signal in {timeout_sec} seconds.")
                return False
            rate.sleep()
        
        return self.dc_motor_ready


    #dc
    def move_slider_to_height(self, target_height_cm, tolerance_cm=1, timeout_sec=15.0):
        """發布目標高度，並等待 Arduino 回報已到達目標位置。"""
        rospy.loginfo(f"Commanding slider to move to {target_height_cm:.2f} cm...")
        
        # 這個檢查仍然有價值，作為雙重保險，確保回報通道也正常
        if self.current_height is None:
            rospy.logwarn("Height feedback is not available yet. Waiting for first feedback message...")
            rospy.sleep(1.0) 
            if self.current_height is None:
                rospy.logerr("Still no height feedback. Aborting move.")
                return False
        
        # 發布目標高度指令
        height_msg = Float32()
        height_msg.data = float(target_height_cm)
        self.height_pub.publish(height_msg)

        # 進入等待迴圈，直到到達目標或逾時
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if abs(self.current_height - target_height_cm) < tolerance_cm:
                rospy.loginfo(f"Slider reached target. Current height: {self.current_height:.2f} cm.")
                rospy.sleep(0.5)
                return True

            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                rospy.logerr(f"Timeout! Failed to reach {target_height_cm:.2f} cm. Last known height: {self.current_height:.2f} cm.")
                return False

            rate.sleep()
        
        return False

        
    def run_competition_flow(self):
        current_state = "hide"
        while not rospy.is_shutdown():
            rospy.loginfo(f"====== Current State: {current_state} ======")

            if current_state == "hide":
                if self.call_set_distance(1, 10):
                    current_state = "first_up"
                else:
                    current_state = "ERROR_RECOVERY"

            if current_state == "first_up":
                if self.move_slider_to_height(50):
                    current_state = "first_front"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "first_front":
                if self.navigate_by_wall(front=1.3, angle=0.0, align_wall="right"):
                    current_state = "1"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "1":
                if self.navigate_by_wall(right=0.56, angle=0.0, align_wall="right"):
                    current_state = "2"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "2":
                if self.navigate_by_wall(front=1.22, angle=0.0, align_wall="right"):
                    current_state = "3"
                else:
                    current_state = "ERROR_RECOVERY"
            
            elif current_state == "3":
                if self.call_set_distance(1, 29):
                    current_state = "first_down"
                else:
                    current_state = "ERROR_RECOVERY"        

            elif current_state == "first_down":
                if self.move_slider_to_height(31.75):
                    current_state = "first_withdraw"
                else:
                    current_state = "ERROR_RECOVERY"
#################################################################
            elif current_state == "first_withdraw":
                if self.call_set_distance(1, -32.8):
                    current_state = "4"
                else:
                    current_state = "ERROR_RECOVERY"
#################################################################
            elif current_state == "4":
                if self.navigate_by_wall(front=1.35, angle=0.0, align_wall="right"):
                    current_state = "4.3"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "4.3":
                if self.move_slider_to_height(15):
                    current_state = "4.5"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "4.5":
                if self.move_slider_to_height(3.256):
                    current_state = "4.6"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "4.6":
                if self.call_set_distance(1, -0.4):
                    current_state = "5"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "5":
                if self.navigate_by_wall(angle=90):
                    current_state = "5.5"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "5.5":
                if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                    current_state = "6"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "6":
                if self.navigate_by_wall(rear=2.178, angle=0.0, align_wall="right"):
                    current_state = "6.5"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "6.5":
                if self.navigate_by_wall(angle=90.0):
                    current_state = "6.7"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "6.7":
                if self.navigate_by_wall(angle=0.0, align_wall="rear"):
                    current_state = "7"
                else:
                    current_state = "ERROR_RECOVERY"

            # elif current_state == "4":
            #     if self.call_set_height(4):
            #         current_state = "put_coffee_down1"
            #     else:
            #         current_state = "ERROR_RECOVERY"

            # elif current_state == "7":
            #     if self.navigate_by_wall(left=2.319, angle=0.0, align_wall="left"):
            #         current_state = "7.5"
            #     else:
            #         current_state = "ERROR_RECOVERY"

            elif current_state == "7":
                if self.navigate_by_wall(rear=1.471, angle=0.0, align_wall="left"):
                    current_state = "put_coffee_down1"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "put_coffee_down1":
                if self.move_slider_to_height(1):
                    current_state = "first_put_coffee"
                else:
                    current_state = "ERROR_RECOVERY"
# ##################################################################
            elif current_state == "first_put_coffee":
                if self.call_set_distance(1, 12):
                    current_state = "put_coffee_down2"
                else:
                    current_state = "ERROR_RECOVERY"
# ####################################################################
            elif current_state == "put_coffee_down2":
                if self.move_slider_to_height(30):
                    current_state = "second_put_coffee"
                else:
                    current_state = "ERROR_RECOVERY"
# ###################################################################
            elif current_state == "second_put_coffee":
                if self.call_set_distance(1, -12):
                    current_state = "8.8"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "8.8":
                if self.navigate_by_wall(rear=1.35, angle=0.0, align_wall="rear"):
                    current_state = "put_coffee_down3"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "put_coffee_down3":
                if self.move_slider_to_height(0):
                    current_state = "put_coffee_down4"
                else:
                    current_state = "ERROR_RECOVERY"
###########################################################
#             elif current_state == "put_coffe_down3":
#                 if self.call_set_height():
#                     current_state = "third_put_coffee"
#                 else:
#                     current_state = "ERROR_RECOVERY"
# ##############################################################
#             elif current_state == "third_put_coffee":
#                 if self.call_set_distance(1,):
#                     current_state = "put_coffe_down4"
#                 else:
#                     current_state = "ERROR_RECOVERY"
# ################################################################
#             elif current_state == "put_coffe_down4":
#                 if self.call_set_height():
#                     current_state = "1"
#                 else:
#                     current_state = "ERROR_RECOVERY"
#########################################################
            elif current_state == "put_coffee_down4":
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