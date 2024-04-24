#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image

import base64
from std_msgs.msg import String


class Camera:
  def __init__(self, camera_id):
    rospy.init_node('camera')
    self.camera_id = camera_id
    self.cap = cv2.VideoCapture(self.camera_id)

    if self.cap.isOpened():
      print('\nCamera connected.\n')
    else :
      print('\nCamera not connected.\n')

    self.web_pub = rospy.Publisher('/golfbot/camera_web', String, queue_size=1)
        
    rate = rospy.Rate(10)

  def talker(self):
    while not rospy.is_shutdown():
      ret, frame = self.cap.read()
      if not ret : 
        break
      
      # Encode the image
      _, buffer = cv2.imencode('.jpg', frame)
      image_as_str = base64.b64encode(buffer).decode('utf-8')

      # Publish the encoded image
      self.web_pub.publish(image_as_str)

    self.cap.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
  camera = Camera(0)
  try:
    camera.talker()
  except rospy.ROSInterruptException:
    pass