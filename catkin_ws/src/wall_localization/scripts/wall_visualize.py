#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import math

# Global variable to store the latest data from the ROS topic
latest_data = [float('nan')]*8 # [front_dist, rear_dist, left_dist, right_dist, front_angle, rear_angle, left_angle, right_angle]

def wall_info_cb(msg):
    """
    Callback function for the ROS subscriber.
    Updates the global latest_data variable.
    """
    global latest_data
    if len(msg.data) == 8:
        latest_data = list(msg.data) # Ensure it's a list, though msg.data usually is
    else:
        rospy.logwarn_throttle(5, f"Received data with incorrect length: {len(msg.data)}. Expected 8.")
        # Optionally, reset to nan to indicate bad data
        # latest_data = [float('nan')]*8


def init_plot():
    """
    Initializes the matplotlib plot with a 3x3 grid layout.
    The central cell and corner cells are unused.
    Sets up 4 subplots for Front, Left, Rear, and Right wall information.
    Returns the figure and a list of text objects for updating.
    """
    fig = plt.figure(figsize=(8,8))
    fig.canvas.manager.set_window_title('Wall Information Visualizer') # Set a window title

    gs = gridspec.GridSpec(3, 3, figure=fig)

    # Create subplots for displaying wall information
    ax_front = fig.add_subplot(gs[0,1]) # Top-middle
    ax_left  = fig.add_subplot(gs[1,0]) # Middle-left
    ax_rear  = fig.add_subplot(gs[2,1]) # Bottom-middle
    ax_right = fig.add_subplot(gs[1,2]) # Middle-right

    # Turn off axes for unused subplots in the grid
    unused_cells_indices = [(0,0), (0,2), (1,1), (2,0), (2,2)]
    for r_idx, c_idx in unused_cells_indices:
        ax_unused = fig.add_subplot(gs[r_idx, c_idx])
        ax_unused.axis('off')

    axes_list = [ax_front, ax_left, ax_rear, ax_right]
    titles = ['Front Wall','Left Wall','Rear Wall','Right Wall']

    for ax, title in zip(axes_list, titles):
        ax.set_title(title, fontsize=16)
        ax.set_xlim(0,1) # For text positioning
        ax.set_ylim(0,1) # For text positioning
        ax.axis('off') # Turn off axis lines and ticks for these display plots

    # Create text objects in each subplot for displaying data
    text_objects = []
    for ax in axes_list:
        txt = ax.text(0.5, 0.5, '', ha='center', va='center',
                      fontsize=14, fontfamily='monospace')
        text_objects.append(txt)

    plt.tight_layout() # Adjust plot to ensure everything fits without overlapping
    return fig, text_objects

def update_plot(frame, text_elements):
    """
    Update function called by FuncAnimation.
    Updates the text in each subplot with the latest distance and angle data.
    The 'frame' argument is provided by FuncAnimation but not used here.
    """
    # latest_data indices:
    # 0: front_distance, 1: rear_distance, 2: left_distance, 3: right_distance
    # 4: front_angle,    5: rear_angle,    6: left_angle,    7: right_angle
    for i, txt_element in enumerate(text_elements):
        distance = latest_data[i]
        angle = latest_data[i+4] # Corresponding angle

        if math.isnan(distance) or math.isnan(angle):
            txt_element.set_text('No data')
        else:
            txt_element.set_text(f'Distance (m): {distance:.2f}\nAngle (°): {angle:.1f}')
    return text_elements # FuncAnimation expects the artists to be returned

if __name__=='__main__':
    try:
        rospy.init_node('wall_info_visualizer_grid', anonymous=True)

        # Subscribe to the topic publishing wall information
        # Ensure the topic name '/wall_info' matches your publisher node
        rospy.Subscriber('/wall_info', Float32MultiArray,
                         wall_info_cb, queue_size=1)

        rospy.loginfo("Wall Information Visualizer node started. Waiting for data on /wall_info...")

        # Initialize the plot
        fig, text_artists = init_plot()

        # Create the animation
        # interval is in milliseconds (100ms = 10Hz)
        # cache_frame_data=False is added to address the UserWarning
        ani = animation.FuncAnimation(
            fig, update_plot, fargs=(text_artists,), interval=100, cache_frame_data=False
        )

        plt.show() # Display the plot. This is a blocking call.

        # rospy.spin() is generally not needed after plt.show() if the script's
        # main purpose is the plot, as plt.show() blocks until the window is closed.
        # However, ROS callbacks will still run in background threads.
        # If the script needs to continue running as a ROS node after the plot is closed,
        # then rospy.spin() would be relevant here.
        # For this script, once plt.show() returns (plot closed), the script can often exit.
        # If you want the node to explicitly keep running until Ctrl+C after closing the plot:
        # rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt. Shutting down wall visualizer.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
    finally:
        rospy.loginfo("Wall Information Visualizer node shutting down.")
