#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from laser_line_extraction.msg import LineSegmentList
from std_msgs.msg import Float64MultiArray

class WallLocalizer:
    def __init__(self):
        """
        Initialize the ROS node and set up subscribers and publishers
        """
        rospy.init_node('wall_localizer', anonymous=True)

        self.line_subscriber = rospy.Subscriber(
            '/line_segments', 
            LineSegmentList, 
            self.line_callback
        )
        
        self.output_publisher = rospy.Publisher(
            '/wall_distances',
            Float64MultiArray,
            queue_size=10
        )
        
        rospy.loginfo("Result published to /wall_distances topic")

    def line_callback(self, msg):
        """
        Receive line segments and calculate distances and angles to walls
        """
        walls = {
            'front': [], 'rear': [], 'left': [], 'right': []
        }

        ANGLE_TOLERANCE_DEG = 44.0 

        for line in msg.line_segments:
            start_point = np.array(line.start)
            end_point = np.array(line.end)
            
            line_vector = end_point - start_point
            length = np.linalg.norm(line_vector)
            
            if length == 0: continue
            
            mid_point = (start_point + end_point) / 2.0
            
            angle_rad = np.arctan2(line_vector[1], line_vector[0])
            angle_deg = np.degrees(angle_rad)

            # determine wall direction based on angle
            if abs(abs(angle_deg) - 90) < ANGLE_TOLERANCE_DEG:
                if mid_point[0] > 0:
                    walls['rear'].append((length, line_vector, start_point))
                else:
                    walls['front'].append((length, line_vector, start_point))

            elif abs(angle_deg) < ANGLE_TOLERANCE_DEG or abs(angle_deg) > (180 - ANGLE_TOLERANCE_DEG):
                if mid_point[1] > 0:
                    walls['right'].append((length, line_vector, start_point))
                else:
                    walls['left'].append((length, line_vector, start_point))

        output_data = {
            'front_distance': None, 'rear_distance': None, 'left_distance': None, 'right_distance': None,
            'front_angle': None, 'rear_angle': None, 'left_angle': None, 'right_angle': None
        }

        for direction, candidates in walls.items():
            if not candidates: continue
            
            longest_line = max(candidates, key=lambda item: item[0])
            _length, line_vec, start_p = longest_line
            end_p = start_p + line_vec
            distance = np.abs(start_p[0]*end_p[1] - end_p[0]*start_p[1]) / _length
            
            # Ensure line vector is oriented correctly
            if direction in ['front', 'rear'] and line_vec[1] < 0:
                line_vec = -line_vec
            if direction in ['left', 'right'] and line_vec[0] < 0:
                line_vec = -line_vec
            
            line_vec_normalized = line_vec / np.linalg.norm(line_vec)
            nx, ny = line_vec_normalized[0], line_vec_normalized[1]

            # Angle calculation
            # 使用 atan2(cross, dot) 計算帶符號的角度
            if direction in ['front', 'rear']:
                # Calculate signed angle with respect to Y-axis (0, 1)
                # dot = nx*0 + ny*1 = ny
                # cross = nx*1 - ny*0 = nx
                angle_rad = np.arctan2(nx, ny)
                angle_deg = np.degrees(angle_rad)
                output_data[f'{direction}_angle'] = angle_deg
            elif direction in ['left', 'right']:
                # Calculate signed angle with respect to X-axis (1, 0)
                # dot = nx*1 + ny*0 = nx
                # cross = nx*0 - ny*1 = -ny
                angle_rad = np.arctan2(-ny, nx)
                angle_deg = np.degrees(angle_rad)
                output_data[f'{direction}_angle'] = angle_deg
            
            output_data[f'{direction}_distance'] = distance

        final_output = [
            output_data['front_distance'], output_data['rear_distance'],
            output_data['left_distance'], output_data['right_distance'],
            output_data['front_angle'], output_data['rear_angle'],
            output_data['left_angle'], output_data['right_angle']
        ]

        output_msg = Float64MultiArray()
        publish_data = [val if val is not None else float('nan') for val in final_output]
        output_msg.data = publish_data
        self.output_publisher.publish(output_msg)

        # rospy.loginfo_throttle(1.0, f"已發布數據: {[f'{v:.3f}' if not np.isnan(v) else 'NaN' for v in publish_data]}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        localizer = WallLocalizer()
        localizer.run()
    except rospy.ROSInterruptException:
        pass
