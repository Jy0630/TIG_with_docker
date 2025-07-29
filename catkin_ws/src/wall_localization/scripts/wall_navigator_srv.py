#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import tf.transformations
import numpy as np
import math
from wall_localization.srv import SetWallNavigation, SetWallNavigationResponse

def get_segmented_speed(error, thresholds, speeds):
    abs_error = abs(error)
    sign = np.sign(error)
    if abs_error > thresholds[0]: return sign * speeds[0]
    elif abs_error > thresholds[1]: return sign * speeds[1]
    elif abs_error > thresholds[2]: return sign * speeds[2]
    else: return 0.0

def normalize_angle(angle_deg):
    # Keep in [-180, 180]
    v = (angle_deg + 180) % 360 - 180
    return v

class WallNavigator:
    def __init__(self):
        rospy.init_node('wall_navigator')

        rospy.Subscriber('/wall_distances', Float64MultiArray, self.localization_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/dlv/cmd_vel', Twist, queue_size=10)
        self.nav_service = rospy.Service('navigate_by_wall', SetWallNavigation, self.handle_navigation_request)

        self.dist_thresholds = [0.5, 0.2, 0.05]
        self.dist_speeds = [0.3, 0.2, 0.1]
        self.final_angle_thresholds = [30.0, 10.0, 5.0]; self.final_angle_speeds = [0.3, 0.2, 0.1]  
        self.moving_align_thresholds = [30.0, 20.0, 10.0]; self.moving_align_speeds = [0.2, 0.15, 0.08]
        
        self.state = "IDLE"; self.current_goal = None; self.is_active = False
        self.rotation_sub_state = None; self.rotation_target_angles = {}
        self.alignment_wall = None
        self.odom_target_yaw = 0.0; self.current_yaw = 0.0; self.odom_received = False

        rospy.loginfo("Wall Navigator Node launched. Waiting for goals on service '/navigate_by_wall'...")

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        q_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(q_list)
        self.odom_received = True

    def handle_navigation_request(self, req):
        if (req.target_front_distance != -1.0 and req.target_rear_distance != -1.0) or \
           (req.target_left_distance != -1.0 and req.target_right_distance != -1.0):
            rospy.logerr("Mission Denied: Conflicting linear goals.")
            return SetWallNavigationResponse(success=False, message="Conflicting linear goals.")

        rospy.loginfo("New mission received via service. State transition to -> MOVING_TO_POINT")

        self.current_goal = req
        self.is_active = True
        self.state = "MOVING_TO_POINT"

        rate = rospy.Rate(20)
        while self.is_active and not rospy.is_shutdown():
            rate.sleep()

        if self.state == "IDLE":
            rospy.loginfo("Mission Accomplished! Sending success response.")
            return SetWallNavigationResponse(success=True, message="Navigation goal reached successfully.")
        else:
            rospy.logerr("Mission Failed or Interrupted. Sending failure response.")
            self.cmd_pub.publish(Twist())
            return SetWallNavigationResponse(success=False, message="Navigation failed or was interrupted.")

    # 最高指導原則旋轉邏輯
    def predict_and_prepare_rotation(self, sensed_data, angle_offset_deg):
        # Step 1: 找到一個可用參考牆
        candidates = []
        for wall in ['front','right','rear','left']:
            ang = sensed_data.get(f"{wall}_angle")
            if ang is not None and not np.isnan(ang):
                candidates.append((wall, ang))
        if not candidates:
            rospy.logwarn("No reference wall. Fallback to odometry-only rotation.")
            self.rotation_sub_state = "ROTATING_COMPLEX_COARSE"
            self.odom_target_yaw = self.current_yaw + np.radians(angle_offset_deg)
            self.state = "ROTATING_TO_ANGLE"
            return

        ref_wall, ref_angle = candidates[0]
        initial_ang = ref_angle
        rot = angle_offset_deg
        new_angle = normalize_angle(initial_ang + rot)

        # 依照remark: "若新角度在±44以內→同一牆，否則換牆"
        if -44.0 <= new_angle <= 44.0:
            # 不會換牆，可直接追角度
            self.rotation_target_angles = {ref_wall: new_angle}
            self.rotation_sub_state = "ROTATING_SIMPLE"
            rospy.loginfo(f"predict_and_prepare_rotation: same wall ({ref_wall}), target={new_angle:.2f}")
        else:
            # 超過44度 需換牆面 進入用odom粗調再用牆細調
            # 計算該轉到哪一面牆，並將目標角度帶進去
            walls = ['front','right','rear','left']
            idx = walls.index(ref_wall)
            shift = int(round(rot/90.0))
            new_idx = (idx + shift)%4  # 4面牆循環移動
            new_wall = walls[new_idx]

            # 依指導原則，target直接投射給新牆
            self.rotation_target_angles = {new_wall: normalize_angle(initial_ang)}
            self.rotation_sub_state = "ROTATING_COMPLEX_COARSE"
            self.odom_target_yaw = self.current_yaw + np.radians(angle_offset_deg)
            rospy.loginfo(f"predict_and_prepare_rotation: rotate odom first, then fine-tune at {new_wall}, target={normalize_angle(initial_ang):.2f}")

        self.state = "ROTATING_TO_ANGLE"

    def localization_callback(self, msg):
        if not self.is_active: 
            return

        sensed_data = {
            'front_dist': msg.data[0], 'rear_dist': msg.data[1], 'left_dist': msg.data[2], 'right_dist': msg.data[3],
            'front_angle': msg.data[4], 'rear_angle': msg.data[5], 'left_angle': msg.data[6], 'right_angle': msg.data[7]
        }
        cmd = Twist()

        if self.state == "MOVING_TO_POINT":
            has_linear_goal = any(getattr(self.current_goal, f'target_{d}_distance') != -1.0 for d in ['front', 'rear', 'left', 'right'])
            active_controls, achieved_controls = 0, 0
            if has_linear_goal:
                for direction in ['front', 'rear', 'left', 'right']:
                    target_dist_val = getattr(self.current_goal, f'target_{direction}_distance')
                    if target_dist_val != -1.0:
                        active_controls += 1
                        dist = sensed_data.get(f'{direction}_dist')
                        if not np.isnan(dist):
                            error = target_dist_val - dist
                            speed = get_segmented_speed(error, self.dist_thresholds, self.dist_speeds)
                            if direction == 'front': cmd.linear.x = -speed
                            elif direction == 'rear': cmd.linear.x = speed
                            elif direction == 'left': cmd.linear.y = -speed
                            else: cmd.linear.y = speed
                            if speed == 0.0: achieved_controls += 1

            is_alignment_goal = self.current_goal.target_angle == 0.0 and self.current_goal.align_to_wall in ['front', 'rear', 'left', 'right']
            if is_alignment_goal:
                alignment_wall = self.current_goal.align_to_wall
                wall_angle = sensed_data.get(f'{alignment_wall}_angle')
                if not np.isnan(wall_angle):
                    error = 0.0 - wall_angle
                    speed = get_segmented_speed(error, self.moving_align_thresholds, self.moving_align_speeds)
                    cmd.angular.z = speed
                    rospy.loginfo_throttle(1, f"Moving & Aligning (Loose). Err: {error:.2f}, Ang.Z: {cmd.angular.z:.2f}")

            # Transition判斷允許無linear目標直接準備旋轉
            if not has_linear_goal or (achieved_controls == active_controls and active_controls > 0):
                if self.current_goal.target_angle != -1.0:
                    if is_alignment_goal:
                        self.alignment_wall = self.current_goal.align_to_wall
                        rospy.loginfo(f"Transition -> Final Alignment for '{self.alignment_wall}'.")
                        self.state = "ROTATING_TO_ANGLE"
                        self.rotation_sub_state = "ALIGNING_TO_WALL"
                    else:
                        rospy.loginfo("Transition -> Prepare for In-Place Rotation.")
                        self.predict_and_prepare_rotation(sensed_data, self.current_goal.target_angle)
                else:
                    self.state = "GOAL_REACHED"

        elif self.state == "ROTATING_TO_ANGLE":
            if self.rotation_sub_state == "ALIGNING_TO_WALL":
                wall_angle = sensed_data.get(f'{self.alignment_wall}_angle')
                if not np.isnan(wall_angle):
                    error = 0.0 - wall_angle
                    speed = get_segmented_speed(error, self.final_angle_thresholds, self.final_angle_speeds)
                    cmd.angular.z = speed
                    rospy.loginfo_throttle(1, f"Final Aligning (Strict). Err: {error:.2f}, Ang.Z: {cmd.angular.z:.2f}")
                    if abs(error) < self.final_angle_thresholds[-1]:
                        self.state = "GOAL_REACHED"
                else:
                    rospy.logwarn(f"Cannot align: {self.alignment_wall} wall not detected. Mission complete.")
                    self.state = "GOAL_REACHED"
            elif self.rotation_sub_state == "ROTATING_SIMPLE":
                for wall_type, target_angle in self.rotation_target_angles.items():
                    wall_angle = sensed_data.get(f'{wall_type}_angle')
                    if wall_angle is not None and not np.isnan(wall_angle):
                        error = normalize_angle(target_angle - wall_angle)
                        speed = get_segmented_speed(error, self.final_angle_thresholds, self.final_angle_speeds)
                        cmd.angular.z = speed
                        rospy.loginfo_throttle(1, f"Fine-tuning {wall_type} | Target:{target_angle:.2f}, Current:{wall_angle:.2f}, Err:{error:.2f}")
                        if abs(error) < self.final_angle_thresholds[-1]:
                            self.state = "GOAL_REACHED"
                        break
            elif self.rotation_sub_state == "ROTATING_COMPLEX_COARSE":
                error_rad = normalize_angle(np.degrees(self.odom_target_yaw - self.current_yaw))
                speed = get_segmented_speed(error_rad, self.final_angle_thresholds, self.final_angle_speeds)
                cmd.angular.z = speed
                if abs(error_rad) < self.final_angle_thresholds[1]:
                    rospy.loginfo("Coarse rotation finished, switching to fine-tune.")
                    # 進入fine tune階段
                    self.rotation_sub_state = "ROTATING_SIMPLE"

        elif self.state == "GOAL_REACHED":
            rospy.loginfo("Goal Reached! Resetting state to IDLE.")
            self.is_active = False
            self.state = "IDLE"
            self.current_goal = None
            cmd = Twist()

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        navigator = WallNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Shutting down Wall Navigator.")
