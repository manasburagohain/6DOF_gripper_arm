from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
src = cv.imread("/home/student/armlab-f19/kinect_example_images/ex0_bgr.png")
dep = cv.imread("/home/student/armlab-f19/kinect_example_images/ex0_depth8.png")
srcTri = np.array( [[126, 442], [131, 93], [476, 98]] ).astype(np.float32)
dstTri = np.array( [[127, 438], [132, 60], [508, 64]] ).astype(np.float32)
warp_mat = cv.getAffineTransform(dstTri, srcTri)
print(warp_mat.shape[0],warp_mat.shape[1])
warp_dst = cv.warpAffine(dep, warp_mat, (src.shape[1], src.shape[0]))
# Rotating the image after Warp
center = (warp_dst.shape[1]//2, warp_dst.shape[0]//2)
angle = -50
scale = 0.6
rot_mat = cv.getRotationMatrix2D( center, angle, scale )
warp_rotate_dst = cv.warpAffine(warp_dst, rot_mat, (warp_dst.shape[1], warp_dst.shape[0]))
cv.imshow('Source image', src)
cv.imshow('Warp', warp_dst)
cv.imshow('rgb + warp', src+warp_dst)
cv.imshow('Depth + warp', dep+warp_dst)
cv.waitKey()