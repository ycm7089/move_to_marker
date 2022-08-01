#! /usr/bin/env python3

from termios import tcdrain
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import os
import numpy as np

class CameraImage():
    def __init__(self):

        self.rgb_image = Image()
        self.cv_br = CvBridge()
        
        self.k = np.array(
            [[1245.133331, 0.0, 1021.56411],
             [0.0, 1244.702243, 765.638118],
             [0.0, 0.0, 1.0]])

        self.d = np.array([-0.355604, 0.108672, -0.000292, 0.000296, 0.0])

        # Subscribers
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_cb)

    def camera_cb(self, msg):
        
        self.rgb_image = self.cv_br.imgmsg_to_cv2(msg, desired_encoding= "bgr8")
        self.findArucoMarkers(self.rgb_image)
        cv2.namedWindow('test')
        cv2.imshow('test', self.rgb_image)
        cv2.waitKey(1)

    def findArucoMarkers(self, img, draw=True):

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, f'DICT_ARUCO_ORIGINAL')

        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        
        bboxs, ids, rejected = cv2.aruco.detectMarkers(gray, arucoDict, parameters = arucoParam, 
                                                       cameraMatrix=self.k, distCoeff=self.d)

        # If markers are detected
        if len(bboxs) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                # 0.1 : Aruco-Marker size, k : camera K, d : camera D
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(bboxs[i], 0.1, self.k,
                                                                        self.d)
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(img, bboxs) 

                # Draw Axis
                cv2.aruco.drawAxis(img, self.k, self.d, rvec, tvec, 0.05)  
                print(tvec)

        return img

if __name__ == '__main__':
    try:
        rospy.init_node('camera_sub')
        cm_camera_sub = CameraImage()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
