#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import numpy as np 

# Global variable to store the latest wall distance and angle data
# [Front distance, Rear distance, Left distance, Right distance,
#                           Front angle, Rear angle, Left angle, Right angle]
latest_data = [np.nan] * 8 

def wall_info_cb(msg):
    """
    ROS訂閱者的回調函數。
    更新全域變數 latest_data。
    """
    global latest_data
    if len(msg.data) == 8:
        latest_data = list(msg.data)
    else:
        rospy.logwarn_throttle(5, f"Wrong format: {len(msg.data)}，must be 8。")

def init_plot():
    """
    Initialize the plot with subplots for robot and wall information.
    """
    fig = plt.figure(figsize=(9, 9))
    fig.canvas.manager.set_window_title('Wall localizer visualization')

    gs = gridspec.GridSpec(3, 3, figure=fig)
    
    # Middle subplot for robot representation
    ax_robot = fig.add_subplot(gs[1, 1])
    ax_robot.set_title("Robot", fontsize=10, y=-0.2)
    ax_robot.add_patch(plt.Rectangle((0.3, 0.2), 0.4, 0.6, facecolor='lightblue', edgecolor='black'))
    ax_robot.arrow(0.5, 0.8, 0, 0.15, head_width=0.08, head_length=0.08, fc='red', ec='red') # 前方箭頭
    ax_robot.text(0.5, 0.9, 'Front (+X)', ha='center', va='bottom', fontsize=9)
    ax_robot.set_xlim(0, 1)
    ax_robot.set_ylim(0, 1)
    ax_robot.axis('off')

    # Create subplots for each wall
    ax_front = fig.add_subplot(gs[0, 1]) 
    ax_left  = fig.add_subplot(gs[1, 0])
    ax_rear  = fig.add_subplot(gs[2, 1])
    ax_right = fig.add_subplot(gs[1, 2])

    # Shut down the corners of the grid
    for r_idx, c_idx in [(0, 0), (0, 2), (2, 0), (2, 2)]:
        fig.add_subplot(gs[r_idx, c_idx]).axis('off')

    axes_list = [ax_front, ax_left, ax_rear, ax_right]
    titles = ['Front wall', 'Left wall', 'Rear wall', 'Right wall']
    text_objects = []

    for ax, title in zip(axes_list, titles):
        ax.set_title(title, fontsize=14, pad=10)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.axis('off')
        txt = ax.text(0.5, 0.5, 'Waiting for result...', ha='center', va='center',
                      fontsize=14, fontfamily='monospace', color='gray')
        text_objects.append(txt)

    plt.tight_layout(pad=3.0)
    return fig, text_objects

def update_plot(frame, text_elements):
    """
    Used by FuncAnimation to update the text elements with the latest data.
    """
    index_map = [
        (0, 4),  # front -> data[0], data[4]
        (2, 6),  # left -> data[2], data[6]
        (1, 5),  # rear -> data[1], data[5]
        (3, 7)   # right -> data[3], data[7]
    ]

    for i, txt_element in enumerate(text_elements):
        dist_idx, angle_idx = index_map[i]
        
        distance = latest_data[dist_idx]
        angle = latest_data[angle_idx]

        if np.isnan(distance) or np.isnan(angle):
            txt_element.set_text('No data')
            txt_element.set_color('orange')
        else:
            txt_element.set_text(f'Distance: {distance:.3f} m\nAngle: {angle:.2f} °')
            txt_element.set_color('black')
            
    return text_elements

if __name__ == '__main__':
    try:
        rospy.init_node('wall_info_visualizer_node', anonymous=True)

        rospy.Subscriber('/wall_distances', Float64MultiArray, wall_info_cb, queue_size=1)
        rospy.loginfo("Visualization node initialized, subscribing to /wall_distances topic...")
        rospy.loginfo("Waiting for wall distance data...")

        fig, text_artists = init_plot()

        # 10 Hz refresing rate
        ani = animation.FuncAnimation(
            fig, update_plot, fargs=(text_artists,), interval=100, cache_frame_data=False
        )

        plt.show() # show the plot window 

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down visualization node due to ROS interrupt.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
    finally:
        rospy.loginfo("Visualization node terminated.")
