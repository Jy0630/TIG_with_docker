#!/usr/bin/env python

import rospy
import serial
import subprocess
import os
import sys
import threading

# 用于存储当前运行的launch进程
launch_process = None

def run_launch_file(button_number):
    global launch_process
    if launch_process:
        # 如果有进程在运行，先终止它
        launch_process.terminate()
        launch_process.wait()  # 等待进程结束

    if button_number == 1:
        launch_file = "/path/to/your/launch1.launch"
    elif button_number == 2:
        launch_file = "/path/to/your/launch2.launch"
    elif button_number == 3:
        launch_file = "/path/to/your/launch3.launch"
    else:
        return

    # 使用subprocess运行ROS launch文件
    launch_process = subprocess.Popen(["roslaunch", launch_file])
    rospy.loginfo(f"Running launch file for button {button_number}")

def restart_script():
    global launch_process
    if launch_process:
        launch_process.terminate()
        launch_process.wait()

    # 重启当前脚本
    os.execv(sys.executable, ['python'] + sys.argv)

def listen_to_arduino(ser):
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            try:
                button_number = int(ser.read().decode('utf-8'))
                if button_number == 4:
                    restart_script()  # 按下按钮4时重启脚本
                else:
                    run_launch_file(button_number)
            except ValueError:
                rospy.logwarn("Received invalid data from Arduino")

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
