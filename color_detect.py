# Importing the required librariess

import cv2
img = cv2.imread('/path/to/image.jpg',cv2.IMREAD_UNCHANGED)


img = cv2.imread('/path/to/image.jpg',cv2.IMREAD_UNCHANGED)

rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
hsv_frame =  cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2HSV)

