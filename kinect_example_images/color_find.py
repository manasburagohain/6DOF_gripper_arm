import sys
import cv2 as cv
import numpy as np

font = cv.FONT_HERSHEY_SIMPLEX

# def mouse_callback(event,x,y,flags,param):
# 	r = img[y][x][2]
# 	g = img[y][x][1]
# 	b = img[y][x][0]
# 	h = hsv[y][x][0]
# 	s = hsv[y][x][1]
# 	v = hsv[y][x][2]
# 	output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
# 	output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
# 	tmp = hsv.copy()
# 	cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (0,0,0))
# 	cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (0,0,0))
# 	cv2.imshow('window', tmp)
# 	if event == cv2.EVENT_LBUTTONDOWN:
# 		print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)


if len(sys.argv) == 2:
	print "Opening " + str(sys.argv[1])
	img = cv.imread(sys.argv[1])
	hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

	# lower_blue = np.array([110,50,50])
	# upper_blue = np.array([130,255,255])

	# mask = cv2.inRange(hsv, lower_blue, upper_blue)

	color_order=['yellow','orange','pink','black','blue','green','purple','red']

	color_lower_array=np.array([[10,0,210],[0,100,170],[165,90,210],[0,20,20],[20,110,20],[40,5,88],[140,70,80],[169,120,120]])
	color_higher_array=np.array([[30,255,255],[40,255,210],[173,215,255],[180,110,80],[110,130,80],[80,150,200],[160,150,210],[180,255,210]])

	i=1
 #    # Threshold the HSV image to get only blue colors

 	# len(color_lower_array)
	for i in range(0,i+1):
		# Segregating the colors
		mask = cv.inRange(hsv, color_lower_array[i], color_higher_array[i])
		print (color_order[i])

		# Performing Morphological Operations on Image
		opening = cv.morphologyEx(mask, cv.MORPH_OPEN, cv.getStructuringElement(cv.MORPH_RECT,(5,5)))
		closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT,(2,2)))
		
		# Detecting the contour
		contours, hierarchy = cv.findContours(closing, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

		# Find the largest contour
		contour_sizes = [(cv.contourArea(contour), contour) for contour in contours]
		biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
		
		# Drawing the largest contour
		# cv.drawContours(img, biggest_contour, -1, (255,255,0), 3)

		# Drawing a bounding rectangle for the detected box
		rect = cv.minAreaRect(biggest_contour)
		box = cv.boxPoints(rect)
		box = np.int0(box)
		center=rect[0]
		print("centers are",center)
		angle = rect[2]
		cv.drawContours(img,[box],0,(0,0,255),2)

		img[int(center[1])-2:int(center[1])+2,int(center[0])-2:int(center[0])+2]=[0,0,255]

		rows, cols = img.shape[:2]
		rot = cv.getRotationMatrix2D(center, angle-90, 1)
		print(rot)
		# img = cv.warpAffine(img, rot, (rows,cols))



		# Center of Mass of the detected contour
		# M = cv.moments(biggest_contour)
		# print("M is", M )

		# cx = int(M['m10']/M['m00'])
		# cy = int(M['m01']/M['m00'])

		# # Printing the COM coordinates of the detected object
		# print ("Cx is ", cx)
		# print ("Cy is ", cy)

		# # Marking the COMon the image
		# img[cy-2:cy+2,cx-2:cx+2]=[0,0,255]

	cv.namedWindow("window",1)
	cv.imshow('window', img)


	# cv2.setMouseCallback("window",mouse_callback)

	while True:
		ch = 0xFF & cv.waitKey(10)
		if ch == 27:
			break
	cv.destroyAllWindows()



else:
	print "Expected filename as argument"