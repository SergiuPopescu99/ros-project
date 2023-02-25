#!/usr/bin/env python
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def image_callback(image):
    bridge = CvBridge()
    try:
      cv_image = bridge.compressed_imgmsg_to_cv2(image)
    except CvBridgeError as e:
      print(e)

    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Green color
    low_red = np.array([36, 0, 0])
    high_red = np.array([86,255,255])
    green_mask = cv2.inRange(hsv_frame, low_red, high_red)
    hsv_red_output = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

    cv2.imshow("Image window hsv", hsv_red_output)
    cv2.imshow("mask", green_mask)
    if cv2.countNonZero(green_mask) > 0:
      print("found green")

    cv2.imshow("Image window1", cv_image)
    cv2.waitKey(1)


  
def main(args):
  rospy.init_node('image_converter', anonymous=True)
  #for turtlebot3 waffle
  #image_topic="/camera/rgb/image_raw/compressed"
  #for usb cam
  #image_topic="/usb_cam/image_raw"
  image_sub = rospy.Subscriber("/camera/image/compressed",  CompressedImage, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)