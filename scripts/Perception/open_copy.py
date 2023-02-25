#!/usr/bin/env python

#import numpy: the data structure that will handle an image
import numpy as np

#import openCV
import cv2
 

image_name = "tree"

print ('read an image from file')
img = cv2.imread("/home/sergiu/catkin_ws/src/ros_tutorial/scripts/Perception/images/tree.jpg")

print ('create a window holder for the image')
cv2.namedWindow("Image",cv2.WINDOW_NORMAL)

print ('display the image')
cv2.imshow("Image",img)

print ('press a key inside the image to make a copy')
cv2.waitKey(0)

print ('image copied to folder images/copy/')
cv2.imwrite("images/copy/"+image_name+"-copy.jpg",img)

#cv2.error: OpenCV(4.2.0) ../modules/highgui/src/window.cpp:376: error: (-215:Assertion failed)
#  size.width>0 && size.height>0 in function 'imshow