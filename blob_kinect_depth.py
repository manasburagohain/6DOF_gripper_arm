import sys
import cv2
from kinect import Kinect
import numpy as np
import freenect

#!/usr/bin/python3
import time
from functools import partial
from os import path

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor)

from kinect import Kinect

font = cv2.FONT_HERSHEY_SIMPLEX

#############################################################################

# STARTING THE PROGRAM HERE #

# Here depth camera is used for block detection

# Loading the frame from the Kinect Depth Camera
depth_frame = freenect.sync_get_depth()[0]

# Extracting required depth informmation from the appropriate bits
np.clip(depth_frame,0,2**10 - 1,depth_frame)
depth_frame >>= 2
cv2.namedWindow("normal_image",cv2.WINDOW_AUTOSIZE)
cv2.imshow('normal_image', depth_frame)		
depth_frame = depth_frame.astype(np.uint8)

stack_size=['one','two','three','four','five']
stack_thresh_lower_array=np.array([[175],[170],[165],[160],[150]])
stack_thresh_higher_array=np.array([[177],[175],[170],[165],[160]])

# Need to threshold the image to determine stack of blocks

# Once threshold image then find all the contours

# Fit bounding box to all these contours

# fit the center of all these areas

# Convert this location in kinect depth to kinect rgb

# find color at that specific location

# # Performing Operations on camera frames to process the output for all the colors

for i in range(len(stack_thresh_lower_array)):
	# Thresholding the image
	mask = cv2.inRange(depth_frame, stack_thresh_lower_array[i], stack_thresh_higher_array[i])

# 	# Performing Morphological Operations on image

	opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
	closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))

# 	# Detecting the contours
	im2, contours, hierarchy = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

# 	# Find the largest contour # May need to change this to check if area between predefined limits
	contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours if (cv2.contourArea(contour)<900)]

	print (contour_sizes)
	biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

	print(cv2.contourArea(biggest_contour))

# 	# Drawing the largest contour
	# cv2.drawContours(depth_frame, biggest_contour, -1, (255,255,0), 3)

# 	# Drawing a bounding rectangle for the detected box
	rect = cv2.minAreaRect(biggest_contour)
	box = cv2.boxPoints(rect)
	box = np.int0(box)
	center=rect[0]
	angle = rect[2]
	cv2.drawContours(depth_frame,[box],0,(0,0,255),2)

# 	# Marking the center of the box
	depth_frame[int(center[1])-2:int(center[1])+2,int(center[0])-2:int(center[0])+2]=[0]

	# Print Block Height
	# z = kinect.currentDepthFrame[int(center[1])][int(center[0])]
	# print("Block height is",z)

# 	# Determining the rotation matrice
# 	rows, cols = img.shape[:2]
# 	rot = cv.getRotationMatrix2D(center, angle-90, 1)
# 	print(rot)
# 	# img = cv.warpAffine(img, rot, (rows,cols))

# 	# Center of Mass of the detected contour
# 	# M = cv.moments(biggest_contour)
# 	# print("M is", M )

# 	# cx = int(M['m10']/M['m00'])
# 	# cy = int(M['m01']/M['m00'])

# 	# # Printing the COM coordinates of the detected object
# 	# print ("Cx is ", cx)
# 	# print ("Cy is ", cy)

# 	# # Marking the COMon the image
# 	# img[cy-2:cy+2,cx-2:cx+2]=[0,0,255]

cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("mask",cv2.WINDOW_AUTOSIZE)
cv2.imshow('window', depth_frame)
cv2.imshow('mask', closing)

	# cv2.setMouseCallback("window",mouse_callback)

# time.sleep(1) # Use this to continuously show all windows
while True:
	ch = 0xFF & cv2.waitKey(10)
	if ch == 0x1B:
		break
cv2.destroyAllWindows()