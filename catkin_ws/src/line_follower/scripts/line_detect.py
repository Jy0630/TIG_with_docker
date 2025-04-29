#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        # Publisher
        self.cmd_pub = rospy.Publisher("raw_cmd_vel", Twist, queue_size=1)

        # ———————— 參數設定 ————————
        # 影像參考線
        self.horiz_line_y  = rospy.get_param("~horiz_line_y", 240)

        # 位置偏移閾值（像素）
        self.pos_thresh1   = rospy.get_param("~pos_thresh1", 20)
        self.pos_thresh2   = rospy.get_param("~pos_thresh2", 120)
        self.pos_thresh3   = rospy.get_param("~pos_thresh3", 220)

        # 前進速度對應四階段
        self.forward_nominal     = rospy.get_param("~forward_nominal",0.4)
        self.forward_stage1      = rospy.get_param("~forward_stage1",0.3)
        self.forward_stage2      = rospy.get_param("~forward_stage2",0.2)
        self.forward_stage3      = rospy.get_param("~forward_stage3",0.1)

        # 平移速度對應三階段
        self.trans_stage1        = rospy.get_param("~trans_stage1",0.1)
        self.trans_stage2        = rospy.get_param("~trans_stage2",0.2)
        self.trans_stage3        = rospy.get_param("~trans_stage3",0.3)

        # 角度偏差閾值（度）
        self.angle_thresh1       = rospy.get_param("~angle_thresh1",30)
        self.angle_thresh2       = rospy.get_param("~angle_thresh2",50)

        # 旋轉速度對應兩階段
        self.rot_stage1          = rospy.get_param("~rot_stage1",0.3)
        self.rot_stage2          = rospy.get_param("~rot_stage2",0.4)
        # ——————————————————————

        # 相機裝置
        camera_id = rospy.get_param("~camera_id", "/dev/video5")
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            rospy.logerr("無法開啟 camera %s" % camera_id)
            rospy.signal_shutdown("無法開啟 camera")

    def process_frame(self, img):
        # 1. 二值化
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            rospy.loginfo("找不到黑線")
            self.cmd_pub.publish(Twist())
            return

        # 2. 最大輪廓
        c = max(cnts, key=cv2.contourArea)
        cv2.line(img, (0, self.horiz_line_y), (img.shape[1], self.horiz_line_y), (0,255,0), 2)

        # 3. 多段擬合交點與角度
        eps   = 0.02 * cv2.arcLength(c, True)
        approx= cv2.approxPolyDP(c, eps, True).reshape(-1,2)
        inter_x, seg_angle = None, None
        best_d = 1e9
        cx_img = img.shape[1]//2

        for i in range(len(approx)-1):
            p1, p2 = approx[i], approx[i+1]
            y1, y2 = p1[1], p2[1]
            if (y1-self.horiz_line_y)*(y2-self.horiz_line_y) <= 0:
                x_int = p1[0] if y1==y2 else int(p1[0] + (self.horiz_line_y-y1)*(p2[0]-p1[0])/(y2-y1))
                d = abs(x_int - cx_img)
                if d < best_d:
                    best_d = d
                    inter_x = x_int
                    dx, dy = p2[0]-p1[0], p2[1]-p1[1]
                    ang = np.degrees(np.arctan2(dy, dx))
                    seg_angle = ang + 180 if ang<0 else ang

        # fitLine fallback
        if inter_x is None:
            vx, vy, x0, y0 = cv2.fitLine(c, cv2.DIST_L2,0,0.01,0.01).flatten()
            if abs(vy)>1e-3:
                inter_x   = int(x0 + (self.horiz_line_y-y0)*vx/vy)
                seg_angle = np.degrees(np.arctan2(vy, vx))
                if seg_angle<0: seg_angle+=180
            else:
                inter_x, seg_angle = cx_img, 90

        cv2.circle(img, (inter_x, self.horiz_line_y), 5, (0,0,255), -1)

        # 4. 計算誤差
        pos_err   = inter_x - cx_img       # +: 右偏, -:左偏
        ang_err   = seg_angle - 90         # +: 線稍右傾, -:左傾

        twist = Twist()
        # —————————————— 位置矯正階段 ——————————————
        a = abs(pos_err)
        if   a <= self.pos_thresh1:
            # 先做角度矯正
            twist.linear.x = self.forward_nominal
            # 角度在第一階段
            if abs(ang_err) > self.angle_thresh1:
                if abs(ang_err) <= self.angle_thresh2:
                    twist.linear.x  = self.forward_stage1
                    twist.angular.z = -self.rot_stage1 * np.sign(ang_err)
                else:
                    twist.linear.x  = self.forward_stage2
                    twist.angular.z = -self.rot_stage2 * np.sign(ang_err)
        elif a <= self.pos_thresh2:
            twist.linear.x = self.forward_stage1
            twist.linear.y = -self.trans_stage1 * np.sign(pos_err)
        elif a <= self.pos_thresh3:
            twist.linear.x = self.forward_stage2
            twist.linear.y = -self.trans_stage2 * np.sign(pos_err)
        else:
            twist.linear.x = self.forward_stage3
            twist.linear.y = -self.trans_stage3 * np.sign(pos_err)
        # ——————————————————————————————————————————————————————————

        self.cmd_pub.publish(twist)
        cv2.imshow("Line Follower", img)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                self.process_frame(frame)
            else:
                rospy.logerr("讀影像失敗")
            rate.sleep()

if __name__ == '__main__':
    try:
        LineFollower().run()
    except rospy.ROSInterruptException:
        pass
