#!/usr/bin/env python

import rospy
import serial
import subprocess
import os
import sys
import threading
import time

# 用于存储当前运行的launch进程
launch_process = None

def run_launch_file(button_char):
    global launch_process
    if launch_process:
        # 如果有进程在运行，先终止它
        launch_process.terminate()
        launch_process.wait()  # 等待进程结束

    if button_char == '1':
        # launch_file = "$(find main_control)/launch/A_to_D_nav.launch"
        launch_cmd = ["roslaunch", "main_control", "A_to_D_nav.launch"]
    elif button_char == '2':
        launch_cmd = ["roslaunch", "main_control", "A_take_ball_1.launch"]
        # launch_file = "$(find main_control)/launch/A_take_ball_1.launch"
    elif button_char == '3':
        launch_cmd = ["roslaunch", "main_control", "A_take_ball_2.launch"]
        # launch_file = "$(find main_control)/launch/A_take_ball_2.launch"
    elif button_char == '4':
        launch_cmd = ["roslaunch", "main_control", "reset_mainC.launch"]
        # launch_file = "$(find main_control)/launch/reset_mainC.launch"
    elif button_char == '5':
        launch_cmd = ["roslaunch", "main_control", "to_B.launch"]
    else:
        return

    # 使用subprocess运行ROS launch文件
    launch_process = subprocess.Popen(launch_cmd)
    # launch_process = subprocess.Popen(["roslaunch", launch_file])
    rospy.loginfo(f"Running launch file for button {button_char}")

def restart_script():
    global launch_process
    if launch_process:
        launch_process.terminate()
        launch_process.wait()
        time.sleep(0.5)

    # 重启当前脚本
    os.execv(sys.executable, ['python'] + sys.argv)

def reset_process():
    global launch_process
    if launch_process:
        # Terminate the current launch process
        rospy.loginfo("Resetting process by terminating the current launch process.")
        launch_process.terminate()
        launch_process.wait()

        # Optionally, restart the last launch file that was running
        # Uncomment below if you want to restart the last launch process
        # launch_process = subprocess.Popen(["roslaunch", last_launch_file])
    
    rospy.loginfo("Process has been reset.")

def listen_to_arduino(ser):
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                button_char = ser.read().decode('utf-8')  # 读取并解码为字符串
    
                if button_char in ['1', '2', '3', '4', '5']:  # 检查是否是有效按钮
                    run_launch_file(button_char)

                # if button_char in ['1', '2', '3','4','5']:  # 检查是否是有效按钮
                #     run_launch_file(button_char)
                else:
                    rospy.logwarn("Received invalid data from Arduino")

            except Exception as e:
                rospy.logwarn(f"Error receiving data from Arduino: {e}")

def main():
    global launch_process
    rospy.init_node('arduino_listener')
    
    ser = serial.Serial('/dev/arduino', 9600)

    # 启动线程监听Arduino
    arduino_thread = threading.Thread(target=listen_to_arduino, args=(ser,))
    arduino_thread.start()

    # 主线程可以做其他事情，或保持活动状态
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
