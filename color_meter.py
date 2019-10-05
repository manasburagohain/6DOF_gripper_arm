import sys
import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect

font = cv2.FONT_HERSHEY_SIMPLEX

def mouse_callback(event,x,y,flags,param):
    r = img[y][x][2]
    g = img[y][x][1]
    b = img[y][x][0]
    h = hsv[y][x][0]
    s = hsv[y][x][1]
    v = hsv[y][x][2]
    print('2')
    output_rgb = "R:%d, G:%d, B:%d " % (r, g, b)
    output_hsv = "H:%d, S:%d, V:%d" % (h, s, v)
    location="X:%d, Y:%d" % (x,y)

    tmp = img.copy()
    print('3')
    cv2.putText(tmp,output_rgb, (10,20), font, 0.5, (255,255,255))
    cv2.putText(tmp,output_hsv, (10,40), font, 0.5, (255,255,255))
    cv2.putText(tmp,location, (10,60), font, 0.5, (255,255,255))
    print('4')
    cv2.imshow('window', tmp)
    print('5')
    if event == cv2.EVENT_LBUTTONDOWN:
        print "bgr: (%d, %d, %d) \nhsv: (%d, %d, %d)" % (b,g,r,h,s,v)

img = freenect.sync_get_video()[0]
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
cv2.namedWindow("window",2)
# cv2.imwrite('blah.jpg',hsv)
# cv2.imshow("blah",img)
print(img[0][0][2])
cv2.setMouseCallback("window",mouse_callback)

while True:
    ch = 0xFF & cv2.waitKey(10)
    if ch == 27:
        break
cv2.destroyAllWindows()

# else:
#     print "Expected filename as argument"