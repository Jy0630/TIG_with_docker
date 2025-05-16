import math
import numpy as np
import rospy
from laser_line_extraction.msg import LineSegmentList, LineSegment
from std_msgs.msg import Float32MultiArray

class LineSegmentProcessor:
    def __init__(self):
        self.line_segments = []
        # ROS setup
        rospy.init_node('line_segment_processor', anonymous=True)
        self.line_sub = rospy.Subscriber('/line_segments', LineSegmentList, self.line_callback)
        self.output_pub = rospy.Publisher('/wall_info', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def line_callback(self, msg):
        self.line_segments = msg.line_segments

    def normalize_vector(self, vector):
        """Normalizes a vector to unit length."""
        norm = np.linalg.norm(vector)
        if norm == 0:
            return vector
        return vector / norm

    def line_normal_vector(self, segment):
        """Calculates the normal vector of a line segment.  Correct for left-handed coord."""
        normal_x = math.cos(segment.angle)
        normal_y = -math.sin(segment.angle) #Corrected for left-handed
        return np.array([normal_x, normal_y])

    def classify_line(self, segment):
        """Classifies the line segment based on the dot product."""
        normal_vector = self.line_normal_vector(segment)
        normal_vector = self.normalize_vector(normal_vector)

        # Define the robot's axes.  Ensure correct directions!
        axes = {
            'front': np.array([1, 0]),    # x-axis
            'rear': np.array([-1, 0]),   # -x-axis
            'left': np.array([0, 1]),   # y-axis (left side of the robot)
            'right': np.array([0, -1])     # -y-axis (right side of the robot)  <---THIS IS CRITICAL
        }

        # Calculate the dot products and negate them
        dot_products = {
            direction: -np.dot(normal_vector, axis) for direction, axis in axes.items()
        }

        # Determine the direction with the maximum dot product
        best_direction = max(dot_products, key=dot_products.get)

        return best_direction

    def line_length(self, start, end):
        return np.linalg.norm(np.array(end) - np.array(start))

    def segment_angle(self, start, end):
      """Calculates the angle of the segment relative to the robot's forward direction (x-axis)."""
      dx = end[0] - start[0]
      dy = end[1] - start[1]
      angle_rad = math.atan2(dy, dx) # angle in radians
      angle_deg = math.degrees(angle_rad)

      # Adjust the angle to be relative to the robot's forward direction (x-axis)
      angle_deg = (angle_deg + 360) % 360  # Normalize to [0, 360)
      return angle_deg
    def process_line_segments(self):
        result = {'front': None, 'rear': None, 'left': None, 'right': None}
        for seg in self.line_segments:
            direction = self.classify_line(seg)
            length = self.line_length(seg.start, seg.end)
            if result[direction] is None or length > result[direction]['length']:
                result[direction] = {
                    'distance': seg.radius,
                    'angle': self.segment_angle(seg.start, seg.end),
                    'length': length
                }

        output = []
        for key in ['front', 'rear', 'left', 'right']:
            if result[key]:
                output.append(result[key]['distance'])
            else:
                output.append(float('nan'))

        for key in ['front', 'rear', 'left', 'right']:
            if result[key]:
                output.append(result[key]['angle'])
            else:
                output.append(float('nan'))
        return output

    def run(self):
        while not rospy.is_shutdown():
            output = self.process_line_segments()
            output_msg = Float32MultiArray()
            output_msg.data = output
            self.output_pub.publish(output_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        processor = LineSegmentProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
