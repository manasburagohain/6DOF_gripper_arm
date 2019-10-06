import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect

class Kinect():
    def __init__(self):
        self.currentVideoFrame = np.array([])
        self.currentDepthFrame = np.array([])
        if(freenect.sync_get_depth() == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True

        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.kinectCalibrated = False
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)

        """ Extra arrays for colormaping the depth image"""
        self.DepthHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthCM=np.array([])

        """ block info """
        self.block_contours = np.array([])
        self.block_coordinates=np.array([])

    def captureVideoFrame(self):
        """
        Capture frame from Kinect, format is 24bit RGB
        """
        if(self.kinectConnected):
            self.currentVideoFrame = freenect.sync_get_video()[0]
        else:
            self.loadVideoFrame()
        self.processVideoFrame()


    def processVideoFrame(self):
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,255,255),3)


    def captureDepthFrame(self):
        """
        Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.currentDepthFrame = self.registerDepthFrame(freenect.sync_get_depth()[0])
            else:
                self.currentDepthFrame = freenect.sync_get_depth()[0]
        else:
            self.loadDepthFrame()


    def loadVideoFrame(self):
        self.currentVideoFrame = cv2.cvtColor(
            cv2.imread("data/ex0_bgr.png",cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB
            )

    def loadDepthFrame(self):
        self.currentDepthFrame = cv2.imread("data/ex0_depth16.png",0)

    def convertFrame(self):
        """ Converts frame to format suitable for Qt  """
        try:
            img = QImage(self.currentVideoFrame,
                             self.currentVideoFrame.shape[1],
                             self.currentVideoFrame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertDepthFrame(self):
        """ Converts frame to a colormaped format suitable for Qt
            Note: this cycles the spectrum over the lowest 8 bits
        """
        try:

            """
            Convert Depth frame to rudimentary colormap
            """
            self.DepthHSV[...,0] = self.currentDepthFrame
            self.DepthHSV[...,1] = 0x9F
            self.DepthHSV[...,2] = 0xFF
            self.DepthCM = cv2.cvtColor(self.DepthHSV,cv2.COLOR_HSV2RGB)
            cv2.drawContours(self.DepthCM,self.block_contours,-1,(0,0,0),3)

            img = QImage(self.DepthCM,
                             self.DepthCM.shape[1],
                             self.DepthCM.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def getAffineTransform(self, coord1, coord2):
        """
        Given 2 sets of corresponding coordinates,
        find the affine matrix transform between them.

        TODO: Rewrite this function to take in an arbitrary number of coordinates and
        find the transform without using cv2 functions
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        self.depth2rgb_affine = cv2.getAffineTransform(pts1,pts2)
        # print("calcAffine")
        return self.depth2rgb_affine


    def registerDepthFrame(self, frame):
        """
        TODO:
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        # print("warpAffine")
        return cv2.warpAffine(frame,self.depth2rgb_affine,(640,480))

    def loadCameraCalibration(self):
        """
        TODO:
        Load camera intrinsic matrix from file.
        """
        pass

    def blockDetector(self):
        """
        TODO:
        Implement your block detector here.
        You will need to locate
        blocks in 3D space
        """

        # Taking the RGB camera as input
        img = self.currentVideoFrame

        # Converting the Camera frame from RGB to HSV
        # bgr_frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

        # Definining the HSV Values
        color_order=['yellow','orange','pink','black','blue','green','purple','red']
        color_lower_array=np.array([[250,123,123],[123,148,55],[130,177,50],[95,125,120],[140,145,143],[136,110,130],[123,144,106],[87,160,70]])
        color_higher_array=np.array([[255,132,132],[132,158,66],[143,188,60],[107,136,130],[150,156,152],[147,120,140],[132,155,117],[98,170,80]])

        # Extracting the H,S and V values at the center of the block in the RGB frame
        # Creating list to store the detected colors
        color=[]
        count=0

        # print(self.block_coordinates.size)


        for i in range(self.block_coordinates.size - 2):
            h = hsv[int(self.block_coordinates[i+count+1])][int(self.block_coordinates[i+count])][0]
            s = hsv[int(self.block_coordinates[i+count+1])][int(self.block_coordinates[i+count])][1]
            v = hsv[int(self.block_coordinates[i+count+1])][int(self.block_coordinates[i+count])][2]
            rgb_hsv_values=np.array([])
            rgb_hsv_values=np.array([h,s,v])
            print("HSV value of pixel is",rgb_hsv_values)
            count=count+1


            for j in range(len(color_order)):
                # print (rgb_hsv_values[0]>=color_lower_array[int(j)][0])
                # print (rgb_hsv_values[0]<=color_higher_array[int(j)][0])
                # print (rgb_hsv_values[1]>=color_lower_array[int(j)][1])
                # print (rgb_hsv_values[1]<=color_higher_array[int(j)][1])
                # print (rgb_hsv_values[2]>=color_lower_array[int(j)][2])
                # print (rgb_hsv_values[2]<=color_higher_array[int(j)][2])

                if (rgb_hsv_values[0]>=color_lower_array[j][0] and rgb_hsv_values[0]<=color_higher_array[j][0]) and (rgb_hsv_values[1]>=color_lower_array[j][1] and rgb_hsv_values[1]<=color_higher_array[j][1]) and (rgb_hsv_values[2]>=color_lower_array[j][2] and rgb_hsv_values[2]<=color_higher_array[j][2]):
                    color.append(color_order[j])
            # print (color)
            if(self.block_coordinates.size==count*2):
                break

            
            # cv2.setMouseCallback("window",mouse_callback)

        print (color)
        return

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """

        # Loading the frame from the Kinect Depth Camera
        depth_frame = self.currentDepthFrame
        self.block_contours = ([])

        # Extracting required depth informmation from the appropriate bits
        np.clip(depth_frame,0,2**10 - 1,depth_frame)
        depth_frame >>= 2
        depth_frame = depth_frame.astype(np.uint8)

        # Defining lower and higher limits for thresholding
        stack_size=['one','two','three','four','five']
        stack_thresh_lower_array=np.array([[175],[170],[165],[160],[150]])
        stack_thresh_higher_array=np.array([[177],[175],[170],[165],[160]])

        # Defining np array for storing block coordinates
        self.block_coordinates=([])
        block_coordinates=np.array([])
        int_block_contours = []
        int_block_coordinates = np.array([])
        # For each threshold in stack performing the below operations (This is used to detect block stacks up to 5)
        count = 0
        countin = 0
        for i in range(len(stack_thresh_lower_array)):
            # Thresholding the image
            mask = cv2.inRange(depth_frame, stack_thresh_lower_array[i], stack_thresh_higher_array[i])
            # Performing Morphological Operations on image
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
            closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))

            # Detecting the contours
            im2, contours, hierarchy = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            # Find the largest contour and ensuring area is within a defined limit
            contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours if (cv2.contourArea(contour)>400 and cv2.contourArea(contour)<900)]
            # biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
            block_contours = []
            block_coordinates=np.array([])
            # print(contours)
            for contour in contours:
                if (cv2.contourArea(contour)>400 and cv2.contourArea(contour)<900):
                    countin +=1
                    block_contours.append(contour)
            # print(len(block_contours))
            # Drawing the largest contour

            # cv2.drawContours(depth_frame, biggest_contour, -1, (255,255,0), 3)
            print("Threshold is for",stack_size[i])
            # Drawing a bounding rectangle for the detected box
            for contour in block_contours:
                block_coordinates=np.array([])
                count += 1
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                center=rect[0]
                angle = rect[2]
                cv2.drawContours(depth_frame,[box],0,(0,0,255),2)

            # Marking the center of the box
                depth_frame[int(center[1])-2:int(center[1])+2,int(center[0])-2:int(center[0])+2]=[0]

                # print("depth change \n")
            # Storing the center coordinates in np array



                block_coordinates=np.append(block_coordinates,[center[0],center[1]])
                print("Block coordinates are", block_coordinates)
                int_block_contours += block_contours
                # print("Contours are", int_block_contours)
                int_block_coordinates = np.append(int_block_coordinates,block_coordinates)
                print("Block coordinates are", int_block_coordinates)
                print("Length of accumulated block contours is", len(int_block_contours),'b\n')
                print("Length of accumulated block coords is",int_block_coordinates.size, 'c\n')
                print("Block ends")
            # Print Block Height
            # z = kinect.currentDepthFrame[int(center[1])][int(center[0])]
            # print("Block height is",z)

            # Determining the rotation matrice
            # rows, cols = img.shape[:2]
            # rot = cv.getRotationMatrix2D(center, angle-90, 1)
            # print(rot)
            # img = cv.warpAffine(img, rot, (rows,cols))

            # Method 2 for Detecting COM using Moments Method

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
        self.block_contours = int_block_contours
        self.block_coordinates = int_block_coordinates
        # print(block_coordinates)
        cv2.imwrite('test.jpg',depth_frame)
        # cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
        # cv2.namedWindow("mask",cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('window', depth_frame)
        # cv2.imshow('mask', closing)

        # time.sleep(5) # Pausing for 5 seconds
        # while True:
        #     ch = 0xFF & cv2.waitKey(10)
        #     if ch == 0x1B:
        #         break
        # cv2.destroyAllWindows()



        ##############################################################

        # RGB Camera portion of the image


        # pass
        return