#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dynamic_reconfigure.client import Client

def send_goal(x, y, z, w):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def update_dwa_params(max_vel_x, min_vel_x, max_vel_theta, min_vel_theta, 
                      path_distance_bias, goal_distance_bias, xy_goal_tolerance, yaw_goal_tolerance):
    dwa_client = Client("/move_base/DWAPlannerROS")
    params = {
        "max_vel_x": max_vel_x,
        "min_vel_x": min_vel_x,
        "max_vel_theta": max_vel_theta,
        "min_vel_theta": min_vel_theta,
        "path_distance_bias": path_distance_bias,
        "goal_distance_bias": goal_distance_bias,
        "xy_goal_tolerance" : xy_goal_tolerance,
        "yaw_goal_tolerance": yaw_goal_tolerance
    }
    dwa_client.update_configuration(params)

if __name__ == '__main__':

    rospy.init_node('multi_goal_with_dynamic_dwa')

    goals_with_params = [
        # (x, y, z, w), (max_vel_x, min_vel_x, max_vel_theta, min_vel_theta, path_distance_bias, goal_distance_bias)
        ((-4.0, -5.2, -1.0, 0.0), (0.5, -0.3, 0.8, -0.8, 32, 20, 0.2, 0.2))//-16.813, -17.988,
        ((-4.0, -2.5, -0.7, 0.72), (0.5, -0.3, 0.8, -0.8, 32, 20, 0.2, 0.2))//-16.813, -15.338,
        ((-2.53, -2.5, -1.0, 0.0), (0.5, -0.3, 0.8, -0.8, 32, 20, 0.2, 0.2))//-15.343, -15.338,
        ((-2.53, -5.2, 0.72, 0.69), (0.5, -0.3, 0.8, -0.8, 32, 20, 0.2, 0.2))//-15.343, -17.988,
        ((-0.35, -5.2, -1.0, 0.0), (0.5, -0.3, 0.8, -0.8, 32, 20, 0.2, 0.2))//-13.163, -17.988,
    ]

    for goal, params in goals_with_params:
        # Update DWA parameters before sending the goal
        update_dwa_params(params[0], params[1], params[2], params[3], params[4], params[5])

        # Send the navigation goal
        result = send_goal(goal[0], goal[1], goal[2], goal[3])
        if result:
            rospy.loginfo("Goal reached!")
        else:
            rospy.loginfo("Failed to reach goal!")

    rospy.spin()
