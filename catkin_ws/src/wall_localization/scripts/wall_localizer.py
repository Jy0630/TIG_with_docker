#!/usr/bin/env python
import rospy
import math
from laser_line_extraction.msg import LineSegmentList
from std_msgs.msg import Float32MultiArray

def normalize_angle(a):
    a = abs((a + math.pi) % math.pi)
    return a if a <= math.pi/2 else math.pi - a

class WallLocator:
    def __init__(self):

        self.sub = rospy.Subscriber(
            '/line_segments', LineSegmentList, self.cb_lines, queue_size=1)
        
        self.pub = rospy.Publisher(
            '/wall_info', Float32MultiArray, queue_size=1)
        # save the longest ：(radius, angle, length)
        self.sides = dict.fromkeys(['front','rear','left','right'], None)

    def cb_lines(self, msg):
        # initialize
        for k in self.sides: self.sides[k] = None

        for seg in msg.line_segments:
            x0,y0 = seg.start; x1,y1 = seg.end
            dx,dy = x1-x0, y1-y0
            L = math.hypot(dx,dy)
            mx,my = (x0+x1)/2, (y0+y1)/2
            dir_ang = math.atan2(dy,dx)

            # front/rear/left/right decision
            if abs(abs(dir_ang)-math.pi/2) < math.pi/4:
                side = 'front' if mx>0 else 'rear'
            else:
                side = 'right' if my>0 else 'left'

            # longest stays
            prev = self.sides[side]
            if prev is None or L>prev[2]:
                self.sides[side] = (seg.radius, seg.angle, L)

        # publish
        arr = Float32MultiArray()
        data = []
        for side in ['front','rear','left','right']:
            rec = self.sides[side]
            if rec is None:
                data += [float('nan'), float('nan')]
            else:
                r, θ, _ = rec
                wall_dir = θ + math.pi/2
                if side in ('left','right'):
                    ang = normalize_angle(wall_dir)
                else:
                    ang = normalize_angle(wall_dir - math.pi/2)
                data += [r, math.degrees(ang)]
        arr.data = data
        self.pub.publish(arr)

if __name__ == '__main__':
    rospy.init_node('wall_locator', anonymous=False)
    WallLocator()
    rospy.spin()
