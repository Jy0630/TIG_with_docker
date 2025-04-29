#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class SimpleKalman1D:
    def __init__(self, Q, R, P, x0):

        self.Q = Q
        self.R = R
        self.P = P
        self.x = x0

    def update(self, measurement):
        # 预测
        self.P = self.P + self.Q
        # 计算卡尔曼增益
        K = self.P / (self.P + self.R)
        # 更新估计
        self.x = self.x + K * (measurement - self.x)
        # 更新协方差
        self.P = (1 - K) * self.P
        return self.x

class TwistKalmanFilterNode:
    def __init__(self):
        rospy.init_node('twist_kalman_filter')

        # read rosparam

        namespace = rospy.get_name()
        Q_list = rospy.get_param(namespace + '/Q',   [0.1]*6)
        R_list = rospy.get_param(namespace + '/R',   [1.0]*6)
        P0_list = rospy.get_param(namespace + '/P0', [1.0]*6)
        x0_list = rospy.get_param(namespace + '/x0', [0.0]*6)

        # create kalman filters for all 6 dimensions
        self.filters = []
        for i in range(6):
            kf = SimpleKalman1D(
                Q=Q_list[i],
                R=R_list[i],
                P=P0_list[i],
                x0=x0_list[i]
            )
            self.filters.append(kf)

        self.pub = rospy.Publisher('dlv/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('raw_cmd_vel', Twist, self.cb_twist, queue_size=1)

        rospy.loginfo("twist_kalman_filter node activated.")
        rospy.spin()

    def cb_twist(self, msg_raw):
        measurements = [
            msg_raw.linear.x,
            msg_raw.linear.y,
            msg_raw.linear.z,
            msg_raw.angular.x,
            msg_raw.angular.y,
            msg_raw.angular.z,
        ]

        filtered_vals = []
        for kf, m in zip(self.filters, measurements):
            filtered_vals.append(kf.update(m))


        twist_filt = Twist()
        twist_filt.linear.x  = filtered_vals[0]
        twist_filt.linear.y  = filtered_vals[1]
        twist_filt.linear.z  = filtered_vals[2]
        twist_filt.angular.x = filtered_vals[3]
        twist_filt.angular.y = filtered_vals[4]
        twist_filt.angular.z = filtered_vals[5]

        self.pub.publish(twist_filt)

if __name__ == '__main__':
    try:
        TwistKalmanFilterNode()
    except rospy.ROSInterruptException:
        pass
