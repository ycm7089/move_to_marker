#! /usr/bin/env python3

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

        # Subscribers
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_cb)

    def camera_cb(self, msg):
        
        self.rgb_image = self.cv_br.imgmsg_to_cv2(msg, desired_encoding= "bgr8")
        self.findArucoMarkers(self.rgb_image)
        cv2.namedWindow('test')
        cv2.imshow('test', self.rgb_image)
        cv2.waitKey(1)

    def findArucoMarkers(self, img, markerSize = 5, totalMarkers=250, draw=True):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        key = getattr(aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
        arucoDict = aruco.Dictionary_get(key)
        arucoParam = aruco.DetectorParameters_create()
        bboxs, ids, rejected = aruco.detectMarkers(gray, arucoDict, parameters = arucoParam)
        print(ids)

        if draw:
            aruco.drawDetectedMarkers(img, bboxs)

if __name__ == '__main__':
    try:
        rospy.init_node('camera_sub')
        cm_camera_sub = CameraImage()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
