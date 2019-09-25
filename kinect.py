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
        cv2.drawContours(self.currentVideoFrame,self.block_contours,-1,(255,0,255),3)


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
        print(cv2.getAffineTransform(pts1,pts2))
        return cv2.getAffineTransform(pts1,pts2)


    def registerDepthFrame(self, frame):
        """
        TODO:
        Using an Affine transformation, transform the depth frame to match the RGB frame
        """
        pass

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
        pass

    def detectBlocksInDepthImage(self):
        """
        TODO:
        Implement a blob detector to find blocks
        in the depth image
        """

        # Loading the frame from the Kinect Depth Camera
        depth_frame = freenect.sync_get_depth()[0]

        # Extracting required depth informmation from the appropriate bits
        np.clip(depth_frame,0,2**10 - 1,depth_frame)
        depth_frame >>= 2
        depth_frame = depth_frame.astype(np.uint8)

        # Defining lower and higher limits for thresholding
        stack_size=['one','two','three','four','five']
        stack_thresh_lower_array=np.array([[175],[170],[165],[160],[150]])
        stack_thresh_higher_array=np.array([[177],[175],[170],[165],[160]])

        # For each threshold in stack performing the below operations (This is used to detect block stacks up to 5)

        for i in range(len(stack_thresh_lower_array)):
            # Thresholding the image
            mask = cv2.inRange(depth_frame, stack_thresh_lower_array[i], stack_thresh_higher_array[i])

            # Performing Morphological Operations on image
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
            closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))

            # Detecting the contours
            im2, contours, hierarchy = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            # Find the largest contour and ensuring area is within a defined limit
            contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours if (cv2.contourArea(contour)>700 and cv2.contourArea(contour)<900)]
            biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

            # Drawing the largest contour 
            # cv2.drawContours(depth_frame, biggest_contour, -1, (255,255,0), 3)

            # Drawing a bounding rectangle for the detected box
            rect = cv2.minAreaRect(biggest_contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            center=rect[0]
            angle = rect[2]
            cv2.drawContours(depth_frame,[box],0,(0,0,255),2)

            # Marking the center of the box
            depth_frame[int(center[1])-2:int(center[1])+2,int(center[0])-2:int(center[0])+2]=[0]

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

            cv2.namedWindow("window",cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow("mask",cv2.WINDOW_AUTOSIZE)
            cv2.imshow('window', depth_frame)
            cv2.imshow('mask', closing)

            while True:
                ch = 0xFF & cv2.waitKey(10)
                if ch == 0x1B:
                    break
            cv2.destroyAllWindows()
        pass