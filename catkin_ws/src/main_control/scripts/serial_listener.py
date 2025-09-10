#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
from std_msgs.msg import UInt8

class SerialListenerNode:
    """
    監聽指定的序列埠，讀取 Arduino 發送的按鈕ID，
    並將其作為 ROS 的 UInt8 訊息發布到 /button_press 主題。
    """
    def __init__(self):
        rospy.init_node('serial_listener_node', anonymous=True)
        
        port = rospy.get_param('~port', '/dev/arduino_nano')
        baudrate = rospy.get_param('~baudrate', 57600)
        
        rospy.loginfo("Starting serial listener on port %s at %d baud", port, baudrate)

        self.publisher = rospy.Publisher('/button_press', UInt8, queue_size=10)
        
        try:
            # initialize serial connection
            self.serial_port = serial.Serial(port=port, baudrate=baudrate, timeout=1.0)
            rospy.loginfo("Serial port opened successfully.")
        except serial.SerialException as e:
            rospy.logerr("Error opening serial port: %s", e)
            rospy.signal_shutdown("Failed to open serial port.")
            return

    def run(self):
        """
        主迴圈，持續讀取序列埠並發布訊息。
        """
        while not rospy.is_shutdown():
            # check if serial port is still open
            if not self.serial_port.is_open:
                rospy.logwarn("Serial port is not open. Trying to reconnect...")
                try:
                    self.serial_port.open()
                    rospy.loginfo("Serial port reconnected.")
                except serial.SerialException as e:
                    rospy.logerr("Failed to reconnect: %s", e)
                    rospy.sleep(2.0) # wait and do it again
                    continue

            # read line from serial port
            if self.serial_port.in_waiting > 0:
                try:
                    line = self.serial_port.readline()
                    # 將讀取的 bytes 解碼為 utf-8 字串，並去除前後空白
                    button_id_str = line.decode('utf-8').strip()
                    
                    if button_id_str.isdigit():
                        button_id = int(button_id_str)
                        rospy.loginfo("Received button ID from serial: %d", button_id)
                        
                        # 建立並發布 ROS 訊息
                        msg = UInt8()
                        msg.data = button_id
                        self.publisher.publish(msg)
                    else:
                        if button_id_str: # 避免印出空的警告
                           rospy.logwarn("Received non-digit data from serial: '%s'", button_id_str)

                except UnicodeDecodeError:
                    rospy.logwarn("UnicodeDecodeError: Received malformed data.")
                except Exception as e:
                    rospy.logerr("An error occurred: %s", e)

        if self.serial_port.is_open:
            self.serial_port.close()
            rospy.loginfo("Serial port closed.")

if __name__ == '__main__':
    try:
        listener = SerialListenerNode()
        listener.run()
    except rospy.ROSInterruptException:
        pass
