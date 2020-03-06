#!/usr/bin/env python2
import sys

#from __future__ import print_function

from gi.repository import Gst
import gi
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import rospy
import sys
import numpy as np
import roslib

gi.require_version('Gst', '1.0')

print(cv2.__version__)

class publishcamera:

  def __init__(self):
    self.image_pub = rospy.Publisher("/input/image_raw/compressed", CompressedImage,  queue_size=10)
    self.image_pub_raw = rospy.Publisher("/input/image_raw", Image,  queue_size=10)
    self.bridge = CvBridge()
    self.read_cam()

  def read_cam(self):
    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360,format=(string)NV12, framerate=(fraction)7/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
    #print(cap.isOpened())
    if cap.isOpened():
      #cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
      while not rospy.is_shutdown():
        ret_val, cv_image = cap.read()
        cv_image = cv2.flip(cv_image, -1)
	#print('Shape',cv_image.shape)
        # cv2.imshow('demo',cv_image)
        #print("cv_image", cv_image)

        try:
          #bridge = CvBridge().cv2_to_imgmsg(cv_image, "bgr8")
	  #self.image_pub.publish(bridge)

	  #### Create CompressedIamge ####
          msg = CompressedImage()
          msg.header.stamp = rospy.Time.now()
          msg.format = "jpeg"
          msg.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()

	  #### Create RawImage ####
          msg_raw = Image()
          msg_raw.header.stamp = rospy.Time.now()
          msg_raw.encoding = "bgr8"
          msg_raw.height = 360
          msg_raw.width = 640
          msg_raw.data = np.array(cv_image).tostring()
          msg_raw.step = 640*3

          # Publish new image
          self.image_pub.publish(msg)
          self.image_pub_raw.publish(msg_raw)

          

        except CvBridgeError as e:
          print(e)

        keypressed = cv2.waitKey(1)%256
	#print("key ",keypressed)
        
	if keypressed == 27:
          print("escape key pressed")
          cv2.destroyAllWindows()
          break

    else:
        print ("camera open failed, retrying")
        self.read_cam()
        # cv2.destroyAllWindows()

def main(args):  
  rospy.init_node('publishcamera', anonymous=True)
  ic = publishcamera()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':    
  print(cv2.getBuildInformation())
  Gst.debug_set_active(True)
  Gst.debug_set_default_threshold(3)
  main(sys.argv)



