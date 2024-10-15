#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import PoseStamped

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

def publish_goal(x, y, z, w):
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Allow some time for the publisher to establish a connection

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = z
    goal.pose.orientation.w = w

    goal_pub.publish(goal)
    rospy.loginfo("Published goal to /move_base_simple/goal")

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
        ((-5.3, -5.3, 0.726, 0.688), (0.5, -0.3, 0.8, -0.8, 32, 20, 0.2, 0.2))
    ]

    for goal, params in goals_with_params:
        # Update DWA parameters before sending the goal
        update_dwa_params(params[0], params[1], params[2], params[3], params[4], params[5], 
                          params[6], params[7])
        rospy.loginfo("Dwa params updated")
        # Send the navigation goal
        # result = send_goal(goal[0], goal[1], goal[2], goal[3])
        result = publish_goal(goal[0], goal[1], goal[2], goal[3])

        if result:
            rospy.loginfo("Goal reached!")
        else:
            rospy.loginfo("Failed to reach goal!")

    rospy.spin()
