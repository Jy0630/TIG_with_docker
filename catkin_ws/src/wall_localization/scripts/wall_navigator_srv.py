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


def normalize_angle(angle_rad):
    return (angle_rad + math.pi) % (2 * math.pi) - math.pi


class WallNavigator:
    def __init__(self):
        rospy.init_node('wall_navigator')


        # --- ROS Communication ---
        # rospy.Subscriber('/wall_nav/goal', WallNavGoal, self.goal_callback)
        rospy.Subscriber('/wall_distances', Float64MultiArray, self.localization_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/dlv/cmd_vel', Twist, queue_size=10)

        # 建立 Service Server
        self.nav_service = rospy.Service('navigate_by_wall', SetWallNavigation, self.handle_navigation_request)


        # --- Controller Parameters ---
        self.dist_thresholds = [0.5, 0.2, 0.05]; self.dist_speeds = [0.3, 0.2, 0.1]
        
        self.final_angle_thresholds = [30.0, 10.0, 5.0] 
        self.final_angle_speeds = [0.4, 0.2, 0.1]  

        # Loose Thresholds
        self.moving_align_thresholds = [30.0, 20.0, 10.0] 
        self.moving_align_speeds = [0.2, 0.15, 0.05]  
        
        # --- State Management ---
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
        """
        當 main_control 呼叫服務時，此函式會被觸發。
        """
        if (req.target_front_distance != -1.0 and req.target_rear_distance != -1.0) or \
           (req.target_left_distance != -1.0 and req.target_right_distance != -1.0):
            rospy.logerr("Mission Denied: Conflicting linear goals.")
            return SetWallNavigationResponse(success=False, message="Conflicting linear goals.")
        
        rospy.loginfo("New mission received via service. State transition to -> MOVING_TO_POINT")
        
        self.current_goal = req
        self.is_active = True
        self.state = "MOVING_TO_POINT"

        rate = rospy.Rate(10)
        while self.is_active and not rospy.is_shutdown():
            rate.sleep()

        if self.state == "IDLE":
            rospy.loginfo("Mission Accomplished! Sending success response.")
            return SetWallNavigationResponse(success=True, message="Navigation goal reached successfully.")
        else:
            rospy.logerr("Mission Failed or Interrupted. Sending failure response.")
            self.cmd_pub.publish(Twist())
            return SetWallNavigationResponse(success=False, message="Navigation failed or was interrupted.")


    def predict_and_prepare_rotation(self, sensed_data, angle_offset_deg):

        rospy.loginfo(f"State: PREPARE_ROTATION. Preparing to rotate by {angle_offset_deg} degrees.")
        ANGLE_TOLERANCE = 44.0
        wall_name_has_changed = False
        self.rotation_target_angles = {}
        current_angles = {'front': sensed_data['front_angle'], 'rear': sensed_data['rear_angle'],
                          'left': sensed_data['left_angle'], 'right': sensed_data['right_angle']}
        for wall_type, current_angle in current_angles.items():
            if np.isnan(current_angle): continue
            if wall_type == 'front': base_angle = 90
            elif wall_type == 'rear': base_angle = -90
            elif wall_type == 'left': base_angle = 180
            else: base_angle = 0
            world_angle = base_angle - current_angle
            new_relative_angle = (world_angle - angle_offset_deg + 180) % 360 - 180
            new_wall_type = None
            if abs(abs(new_relative_angle) - 90) < ANGLE_TOLERANCE:
                new_wall_type = 'front' if np.cos(np.radians(new_relative_angle)) < 0 else 'rear'
            elif abs(new_relative_angle) < ANGLE_TOLERANCE or abs(new_relative_angle) > (180 - ANGLE_TOLERANCE):
                new_wall_type = 'right' if abs(new_relative_angle) < 90 else 'left'
            if new_wall_type is None: continue
            if new_wall_type != wall_type: wall_name_has_changed = True
            if new_wall_type == 'front': new_target_angle = 90 - new_relative_angle
            elif new_wall_type == 'rear': new_target_angle = -90 - new_relative_angle
            elif new_wall_type == 'left': new_target_angle = 180 - new_relative_angle
            else: new_target_angle = 0 - new_relative_angle
            self.rotation_target_angles[new_wall_type] = new_target_angle
        rospy.loginfo(f"Calculated rotation target angles: {self.rotation_target_angles}")
        if wall_name_has_changed:
            rospy.loginfo("Prediction: Wall classification will change. Using complex rotation strategy.")
            self.rotation_sub_state = "ROTATING_COMPLEX_COARSE"
            self.odom_target_yaw = normalize_angle(self.current_yaw + np.radians(angle_offset_deg))
        else:
            rospy.loginfo("Prediction: Wall classification will not change. Using simple rotation strategy.")
            self.rotation_sub_state = "ROTATING_SIMPLE"
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
            active_controls, achieved_controls = 0, 0
            is_alignment_goal = self.current_goal.target_angle == 0.0 and self.current_goal.align_to_wall in ['front', 'rear', 'left', 'right']

            # --- Linear Control ---
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
            
            # --- Simultaneous Angular Control ---
            if is_alignment_goal:
                alignment_wall = self.current_goal.align_to_wall
                wall_angle = sensed_data.get(f'{alignment_wall}_angle')
                if not np.isnan(wall_angle):
                    error = 0.0 - wall_angle
                    # Use loose thresholds for moving and aligning
                    cmd.angular.z = get_segmented_speed(error, self.moving_align_thresholds, self.moving_align_speeds)
                    rospy.loginfo_throttle(1, f"Moving & Aligning (Loose). Err: {error:.2f}, Ang.Z: {cmd.angular.z:.2f}")

            # --- State Transition Check ---
            if achieved_controls == active_controls and active_controls > 0:
                if self.current_goal.target_angle != -1.0:
                    if is_alignment_goal:
                        self.alignment_wall = self.current_goal.align_to_wall
                        rospy.loginfo(f"Linear movement finished. State transition to -> ALIGNING_TO_WALL ({self.alignment_wall}) to finalize orientation.")
                        self.state = "ROTATING_TO_ANGLE"
                        self.rotation_sub_state = "ALIGNING_TO_WALL"
                    else:
                        rospy.loginfo("Linear movement finished. State transition to -> PREPARE_ROTATION")
                        self.predict_and_prepare_rotation(sensed_data, self.current_goal.target_angle)
                else:
                    self.state = "GOAL_REACHED"
        
        elif self.state == "ROTATING_TO_ANGLE":
            # --- Rotation Sub-State Machine ---
            if self.rotation_sub_state == "ALIGNING_TO_WALL":
                wall_angle = sensed_data.get(f'{self.alignment_wall}_angle')
                if not np.isnan(wall_angle):
                    error = 0.0 - wall_angle
                    # Use strict thresholds for final alignment
                    cmd.angular.z = get_segmented_speed(error, self.final_angle_thresholds, self.final_angle_speeds)
                    rospy.loginfo_throttle(1, f"Final Aligning (Strict). Err: {error:.2f}, Ang.Z: {cmd.angular.z:.2f}")
                    if cmd.angular.z == 0.0:
                        self.state = "GOAL_REACHED"
                else:
                    rospy.logwarn(f"Cannot align: {self.alignment_wall} wall not detected. Mission complete.")
                    self.state = "GOAL_REACHED"

            elif self.rotation_sub_state in ["ROTATING_SIMPLE", "ROTATING_COMPLEX_FINE"]:
                fine_tune_achieved = False
                found_ref_wall = False
                for wall_type in sorted(self.rotation_target_angles.keys(), key=lambda k: ['front', 'rear', 'right', 'left'].index(k)):
                    if not np.isnan(sensed_data[f'{wall_type}_angle']):
                        found_ref_wall = True
                        target_angle = self.rotation_target_angles[wall_type]
                        error = target_angle - sensed_data[f'{wall_type}_angle']
                        # Use strict thresholds for turning in place
                        cmd.angular.z = get_segmented_speed(error, self.final_angle_thresholds, self.final_angle_speeds)
                        rospy.loginfo_throttle(1, f"Fine-tuning (Strict). Target: {target_angle:.2f}, Current: {sensed_data[f'{wall_type}_angle']:.2f}, Err: {error:.2f}")
                        if cmd.angular.z == 0.0: fine_tune_achieved = True
                        break
                
                if found_ref_wall and fine_tune_achieved:
                    self.state = "GOAL_REACHED"
                elif not found_ref_wall:
                    rospy.logwarn_throttle(2, "Cannot fine-tune rotation: No valid reference wall found.")
                    self.state = "GOAL_REACHED"

            elif self.rotation_sub_state == "ROTATING_COMPLEX_COARSE":
                error_rad = normalize_angle(self.odom_target_yaw - self.current_yaw)
                # Use strict thresholds for initial rotation
                cmd.angular.z = get_segmented_speed(np.degrees(error_rad), self.final_angle_thresholds, self.final_angle_speeds)
                if abs(np.degrees(error_rad)) < self.final_angle_thresholds[1]:
                    rospy.loginfo("Coarse rotation by odometry finished. State transition to -> ROTATING_COMPLEX_FINE")
                    self.rotation_sub_state = "ROTATING_COMPLEX_FINE"
        
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
