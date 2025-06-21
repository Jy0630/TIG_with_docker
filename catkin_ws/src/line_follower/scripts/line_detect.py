#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class LineDetectorHybridThreshold:
    def __init__(self):
        rospy.init_node('line_detect', anonymous=True)
        rospy.loginfo("line_detect node started")

        self.bridge = CvBridge()
        camera_id = rospy.get_param("~camera_id", "/dev/camera")
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            rospy.logerr("cannot access camera %s" % camera_id)
            rospy.signal_shutdown("cannot access camera %s" % camera_id)
            return
        rospy.loginfo("launched camera %s" % camera_id)

        self.detection_pub = rospy.Publisher('/line_detect/detection_data', Point, queue_size=1)
        self.intersection_pub = rospy.Publisher('/line_detect/intersection_type', String, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/line_detect/image_processed', Image, queue_size=1)

        rospy.Timer(rospy.Duration(1.0/30.0), self.process_frame)

    def _preprocess_image(self, roi):
        """
        Conbine global Otsu thresholding and adaptive thresholding to enhance line detection.
        """
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        _, global_thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        equalized = clahe.apply(gray)
        # To adjust params, try different values for blockSize and C, (blockSize must be odd)
        adaptive_thresh = cv2.adaptiveThreshold(equalized, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                                cv2.THRESH_BINARY_INV, 15, 4) 

        combined_thresh = cv2.bitwise_or(global_thresh, adaptive_thresh)


        kernel = np.ones((5, 5), np.uint8)
        closed_thresh = cv2.morphologyEx(combined_thresh, cv2.MORPH_CLOSE, kernel)
        
        return closed_thresh

    def process_frame(self, event):
        ret, cv_image = self.cap.read()
        if not ret:
            rospy.logwarn("cannot read frame from camera")
            return

        display_image = cv_image.copy()
        height, width, _ = display_image.shape
        
        bottom_roi = display_image[height // 2:, :]
        top_roi = display_image[:height // 2, :]

        pixel_deviation, angle_deviation, main_line_center_x = self.detect_line(bottom_roi)
        intersection_type = self.detect_intersection(top_roi, main_line_center_x)

        detection_data = Point(x=pixel_deviation, y=angle_deviation, z=0)
        self.detection_pub.publish(detection_data)
        self.intersection_pub.publish(String(data=intersection_type))

        if main_line_center_x is not None:
            cv2.line(display_image, (main_line_center_x, 0), (main_line_center_x, height), (0, 0, 255), 1)

        info_text_line = "PixelDev: {:.1f}, AngleDev: {:.1f}".format(pixel_deviation, angle_deviation)
        info_text_fork = "Intersection: {}".format(intersection_type)
        cv2.putText(display_image, info_text_line, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(display_image, info_text_fork, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.line(display_image, (0, height // 2), (width, height // 2), (255, 255, 0), 2)

        cv2.imshow("Processed Frame", display_image)
        cv2.waitKey(1)

        try:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)

    def detect_line(self, roi):
        thresh = self._preprocess_image(roi)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0, 0.0, None
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        pixel_deviation, angle_deviation, line_center_x = 0.0, 0.0, None
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            pixel_deviation = cx - (roi.shape[1] // 2)
            line_center_x = cx
            rect = cv2.minAreaRect(largest_contour)
            rect_width, rect_height = rect[1]
            angle = rect[2]
            if rect_width < rect_height:
                angle_deviation = angle + 90
            else:
                angle_deviation = angle
            cv2.drawContours(roi, [largest_contour], -1, (0, 255, 0), 2)
        return pixel_deviation, angle_deviation, line_center_x

    def detect_intersection(self, roi, main_line_center_x):
        if main_line_center_x is None:
            return "STRAIGHT"
        thresh = self._preprocess_image(roi)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return "STRAIGHT"
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) < 800:
             return "STRAIGHT"
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.drawContours(roi, [largest_contour], -1, (255, 0, 0), 2)
        center_line_ref = main_line_center_x
        margin = 30
        is_left = (x + w -200) < (center_line_ref - margin)
        is_right = x +200> (center_line_ref + margin)
        if not is_left and not is_right:
            return "T_JUNCTION"
        elif is_left and not is_right:
            return "LEFT_FORK"
        elif not is_left and is_right:
            return "RIGHT_FORK"
        else:
            return "STRAIGHT"

    def on_shutdown(self):
        rospy.loginfo("shutting 'line_detector_node'...")
        self.cap.release()
        rospy.loginfo("camera released")
        cv2.destroyAllWindows()
        rospy.loginfo("OpenCV windows closed")

if __name__ == '__main__':
    try:
        ld_node = LineDetectorHybridThreshold()
        rospy.on_shutdown(ld_node.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
