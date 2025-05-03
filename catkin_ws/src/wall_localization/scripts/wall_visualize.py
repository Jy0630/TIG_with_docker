#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation
import math

latest_data = [float('nan')]*8

def wall_info_cb(msg):
    global latest_data
    if len(msg.data) == 8:
        latest_data = msg.data

def init_plot():
    fig = plt.figure(figsize=(8,8))

    gs = gridspec.GridSpec(3, 3, figure=fig)

    # set up 4 subplots
    ax_front = fig.add_subplot(gs[0,1])
    ax_left  = fig.add_subplot(gs[1,0])
    ax_rear  = fig.add_subplot(gs[2,1])
    ax_right = fig.add_subplot(gs[1,2])

    # useless subplots go away
    for r in [0,2]:
        for c in [0,2]:
            fig.add_subplot(gs[r,c]).axis('off')
    fig.add_subplot(gs[1,1]).axis('off')


    for ax, title in zip(
        [ax_front, ax_left, ax_rear, ax_right],
        ['Front_wall','Left_wall','Rear_wall','Right_wall']
    ):
        ax.set_title(title, fontsize=16)
        ax.set_xlim(0,1); ax.set_ylim(0,1)
        ax.axis('off')

    # set up text
    txts = []
    for ax in [ax_front, ax_left, ax_rear, ax_right]:
        txt = ax.text(0.5,0.5,'',ha='center',va='center',
                      fontsize=14, fontfamily='monospace')
        txts.append(txt)
    plt.tight_layout()
    return fig, txts

def update(frame, txts):
    for i, txt in enumerate(txts):
        d = latest_data[i]
        a = latest_data[i+4]
        if math.isnan(d) or math.isnan(a):
            txt.set_text('No data')
        else:
            txt.set_text(f'Distance(m): {d:.2f}\nAngle(°): {a:.1f}')
    return txts

if __name__=='__main__':
    rospy.init_node('wall_info_viz_grid')
    rospy.Subscriber('/wall_info', Float32MultiArray,
                     wall_info_cb, queue_size=1)

    # 100 ms update once (10Hz)
    fig, txts = init_plot()
    ani = animation.FuncAnimation(
        fig, update, fargs=(txts,), interval=100)
    plt.show()
    rospy.spin()
