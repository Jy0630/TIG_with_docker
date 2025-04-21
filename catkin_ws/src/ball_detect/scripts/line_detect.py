#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)

        # 發佈機器人控制指令
        self.cmd_pub = rospy.Publisher("dlv/cmd_vel", Twist, queue_size=1)

        # 從 rosparam 讀取參數
        self.horiz_line_y = rospy.get_param("~horiz_line_y", 240)          # 參考線在影像中的 y 座標
        self.center_tolerance = rospy.get_param("~center_tolerance", 30)     # 中心容忍範圍（像素）
        self.angle_threshold = rospy.get_param("~angle_threshold", 10)       # 角度容忍誤差（度數）
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)           # 正常前進速度
        self.correcting_speed = rospy.get_param("~correcting_speed", 0.1)     # 校正基礎速度（較低速度）
        # 比例參數：依誤差調整校正力道
        self.pos_kp = rospy.get_param("~pos_kp", 0.005)                      # 位置誤差比例係數
        self.angle_kp = rospy.get_param("~angle_kp", 0.005)                  # 角度誤差比例係數

        # 攝像頭設備參數
        camera_id = rospy.get_param("~camera_id", "/dev/video5")

        # 開啟攝像頭
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            rospy.logerr("無法開啟camera %s" % camera_id)
            rospy.signal_shutdown("無法開啟camera")

        

    def process_frame(self, cv_image):
        """
        影像處理與控制邏輯：
          1. 轉灰階、二值化，並找出黑線輪廓。
          2. 利用 cv2.approxPolyDP 將輪廓近似為多段直線，以捕捉曲線特性。
          3. 從多段直線中找出穿過參考水平線的線段，並計算其在參考線上的交點與斜率。
          4. 根據交點位置與角度誤差，採用比例控制（pos_kp 與 angle_kp）發布平移與旋轉指令，同時保持向前移動。
        """



        # 轉灰階與二值化
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        # 找出所有外部輪廓
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.loginfo("找不到黑線輪廓")
            # 建立 Twist 訊息
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0 
            twist.angular.z = 0 
            self.cmd_pub.publish(twist)
            return

        # 選擇面積最大的輪廓作為目標
        c = max(contours, key=cv2.contourArea)

        # 畫出參考水平線
        cv2.line(cv_image, (0, self.horiz_line_y), (cv_image.shape[1], self.horiz_line_y), (0, 255, 0), 2)

        # 取得輪廓多邊形近似，epsilon 為輪廓周長的 2%
        epsilon = 0.02 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        approx = approx.reshape(-1, 2)
        
        # 畫出所有線段以供調試
        for i in range(len(approx) - 1):
            pt1 = tuple(approx[i])
            pt2 = tuple(approx[i+1])
            cv2.line(cv_image, pt1, pt2, (255, 0, 255), 2)
        # 也連接最後一點與第一點
        if len(approx) > 2:
            cv2.line(cv_image, tuple(approx[-1]), tuple(approx[0]), (255, 0, 255), 2)

        # 找出穿過參考水平線的線段
        seg_found = False
        inter_x = None
        seg_angle = None  # 此段直線的角度（度）
        min_dist = 1e9
        for i in range(len(approx) - 1):
            p1 = approx[i]
            p2 = approx[i+1]
            y1, y2 = p1[1], p2[1]
            # 若參考線在兩點之間（含相等）的話
            if (y1 - self.horiz_line_y) * (y2 - self.horiz_line_y) <= 0:
                # 線性插值求 x 座標
                if (p2[1] - p1[1]) != 0:
                    ratio = (self.horiz_line_y - p1[1]) / float(p2[1] - p1[1])
                    x_int = int(p1[0] + ratio * (p2[0] - p1[0]))
                else:
                    x_int = p1[0]
                # 若有多個，取與影像中心最接近者
                img_center = cv_image.shape[1] // 2
                if abs(x_int - img_center) < min_dist:
                    min_dist = abs(x_int - img_center)
                    inter_x = x_int
                    # 計算該段直線的斜率，單位轉換為角度
                    dx = p2[0] - p1[0]
                    dy = p2[1] - p1[1]
                    seg_angle = np.arctan2(dy, dx) * 180.0 / np.pi
                    # 調整角度到 [0,180)
                    if seg_angle < 0:
                        seg_angle += 180
                    seg_found = True

        # 如果未找到穿過參考線的線段，則退回用 cv2.fitLine 擬合整個輪廓
        # if not seg_found:
        #     [vx, vy, x0, y0] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
        #     # 求交點
        #     if vx != 0:
        #         seg_angle = np.arctan2(vy, vx) * 180.0 / np.pi
        #         if seg_angle < 0:
        #             seg_angle += 180
        #         # 計算交點：線性方程求 x 當 y=self.horiz_line_y
        #         inter_x = int(x0 + ((self.horiz_line_y - y0) * vx / vy))
        #     else:
        #         seg_angle = 90
        #         inter_x = cv_image.shape[1] // 2

        # 如果未找到穿過參考水平線的線段，則退回用 cv2.fitLine 擬合整個輪廓
        if not seg_found:

            [vx, vy, x0, y0] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            if abs(vy) > 1e-3:  # 檢查 vy 是否足夠大，以免除以零
                seg_angle = np.arctan2(vy, vx) * 180.0 / np.pi
                if seg_angle < 0:
                    seg_angle += 180
                inter_x = int(x0 + ((self.horiz_line_y - y0) * vx / vy))
            else:
                # 當 vy 非常小時，認為線條近似水平，故給予預設值
                seg_angle = 90
                inter_x = cv_image.shape[1] // 2

        # 畫出參考交點
        if inter_x is not None:
            cv2.circle(cv_image, (inter_x, self.horiz_line_y), 5, (0, 0, 255), -1)

        # 計算幾何中心，這裡用整個輪廓的重心
        M = cv2.moments(c)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(cv_image, (cx, cy), 5, (255, 0, 0), -1)
        else:
            cx = cv_image.shape[1] // 2

        # 使用交點位置與參考中心來計算平移誤差
        img_center = cv_image.shape[1] // 2
        pos_error = inter_x - img_center  # 正數表示右偏，負數表示左偏

        # 旋轉誤差：理想應為 90 度（垂直）
        angle_error = seg_angle - 90

        # 根據誤差乘上比例係數，得到調整量
        adjust_linear_y = self.pos_kp * pos_error
        adjust_angular_z = self.angle_kp * angle_error

        # 若誤差在容忍範圍內則不做調整
        if abs(pos_error) < self.center_tolerance:
            adjust_linear_y = 0
        if abs(angle_error) < self.angle_threshold:
            adjust_angular_z = 0

        # 建立 Twist 訊息
        twist = Twist()
        twist.linear.x = self.forward_speed
        twist.linear.y = -adjust_linear_y  # 若右偏，往左修正（注意方向依機器人定義）
        twist.angular.z = -adjust_angular_z  # 根據旋轉誤差校正方向，可依實際機器人調整符號

        self.cmd_pub.publish(twist)

        # 調試顯示影像
        cv2.imshow("Line Follower", cv_image)
        cv2.waitKey(3)

    def run(self):
        rate = rospy.Rate(30)  # 30 FPS
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                self.process_frame(frame)
            else:
                rospy.logerr("讀取影像失敗")
            rate.sleep()

if __name__ == '__main__':
    try:
        lf = LineFollower()
        lf.run()
    except rospy.ROSInterruptException:
        pass
