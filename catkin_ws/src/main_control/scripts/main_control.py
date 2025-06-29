#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from line_follower.srv import SetLineFollower
from wall_localization.srv import SetWallNavigation

class MainController:

    def __init__(self):
        rospy.init_node('main_control_node')
        rospy.loginfo("Main Controller Node Started.")

        # --- 建立服務客戶端 (Service Clients) ---
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('set_line_follower')
        self.line_follower_client = rospy.ServiceProxy('set_line_follower', SetLineFollower)
        
        rospy.wait_for_service('navigate_by_wall')
        self.wall_nav_client = rospy.ServiceProxy('navigate_by_wall', SetWallNavigation)
        rospy.loginfo("All services are ready.")

        self.last_intersection_type = None
        self.intersection_sub = rospy.Subscriber('/line_detect/intersection_type', String, self.intersection_callback)
        rospy.loginfo("Subscribed to '/line_detect/intersection_type'.")


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
        rospy.loginfo(f"Executing task: Wall navigation...")
        try:
            # Service is synchronous, so we can wait for it to be ready
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


    def run_competition_flow(self):
        current_state = "CROSS_BRIDGE"

        while not rospy.is_shutdown():
            rospy.loginfo(f"====== Current State: {current_state} ======")

            if current_state == "CROSS_BRIDGE":
                if self.follow_line_until_t_junction():
                    current_state = "NAV_AFTER_BRIDGE"
                else:
                    current_state = "ERROR_RECOVERY"

            elif current_state == "NAV_AFTER_BRIDGE":

                if self.navigate_by_wall(front=0.5):
                    current_state = "TURN_TO_COFFEE_SHOP"
                else:
                    current_state = "ERROR_RECOVERY"



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
