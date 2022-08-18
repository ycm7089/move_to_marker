#! /usr/bin/env python3

from termios import tcdrain
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import os
import numpy as np
from aruco_mapping.msg import ArucoMarker
# from geometry_msgs import Pose

class CameraImage():
    def __init__(self):

        self.rgb_image = Image()
        self.cv_br = CvBridge()
        self.Aruco_Marker_Pose = ArucoMarker()
        
        self.k = np.matrix(
            [[1245.133331, 0.0, 1021.56411],
             [0.0, 1244.702243, 765.638118],
             [0.0, 0.0, 1.0]])

        self.d = np.matrix([[-0.355604, 0.108672, -0.000292, 0.000296, 0.0]])

        # Subscribers
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.camera_cb)
        self.marker_pose_pub = rospy.Publisher("Aruco_Pose", ArucoMarker, queue_size=1)

    def camera_cb(self, msg):
        
        self.rgb_image = self.cv_br.imgmsg_to_cv2(msg, desired_encoding= "bgr8")
        self.findArucoMarkers(self.rgb_image)
        # cv2.namedWindow('test')
        # cv2.imshow('test', self.rgb_image)
        # cv2.waitKey(1)

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
                # should check rvec
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(bboxs[i], 0.1, self.k, self.d)
                print(rvec)
                print("====="*20)
                print(tvec)
                print("")
                # remove [] in tvec, rvec 
                re_tvec = tvec[0]
                re_rvec = rvec[0]
                # print(re_tvec)
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(img, bboxs) 

                # Draw Axis
                cv2.aruco.drawAxis(img, self.k, self.d, re_rvec, re_tvec, 0.05)

                #
                Roll, Pitch ,Yaw = self.get_radian_from_euler(re_rvec,re_tvec)

                Q_x, Q_y, Q_z, Q_w = self.get_quaternion_from_euler(Roll, Pitch, Yaw)
                # print("Q_x %.3f Q_Y %.3f Q_z %.3f Q_w" %(Q_x[0], Q_y[0], Q_z[0],Q_w[0]))
                # print(Q_x, Q_y, Q_z, Q_w )
                # orientation value is very strange

                self.Aruco_Marker_Pose.num_of_visible_markers = len(bboxs)
                
                self.Aruco_Marker_Pose.global_camera_pose.position.x = re_tvec[0][0]
                self.Aruco_Marker_Pose.global_camera_pose.position.y = re_tvec[0][1]
                self.Aruco_Marker_Pose.global_camera_pose.position.z = re_tvec[0][2]

                self.Aruco_Marker_Pose.global_camera_pose.orientation.x = Q_x
                self.Aruco_Marker_Pose.global_camera_pose.orientation.y = Q_y
                self.Aruco_Marker_Pose.global_camera_pose.orientation.z = Q_z
                self.Aruco_Marker_Pose.global_camera_pose.orientation.w = Q_w

                self.marker_pose_pub.publish(self.Aruco_Marker_Pose)

                # if ids == 20 :
                #     print("Yes")

        return img

    def get_radian_from_euler(self, rvec, tvec) :
        # dst : 3x3 matrix, jacobian : 3x9 matrix 
        # https://www.programcreek.com/python/example/89450/cv2.Rodrigues
        dst, jacobian = cv2.Rodrigues(rvec)
        # print(dst)
        # print("====="*20)
        self.dst_1 = np.array(dst)
        self.tvec_1 = np.array(tvec)
        # print(self.dst_1.shape)
        # print(self.tvec_1.shape)

        # transpose(self.tvec_1, (1,0)) : self.tvec_1 의 0번째와 1번째의 값의 순서를 바꾸겠다
        # 1은 dimension을 의미한다. 2차원은 v,h로 가능하지만 그 이상은 어렵기에 숫자가 도입
        self.pose_mat = np.concatenate((self.dst_1, np.transpose(self.tvec_1,(1,0))),1)    

        # self.pose_mat = cv2.hconcat((self.dst_1,self.tvec_1)) 
        # print(self.pose_mat)
        _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(self.pose_mat)
        # http://amroamroamro.github.io/mexopencv/matlab/cv.decomposeProjectionMatrix.html
        # print("Roll %.3f Pitch %.3f Yaw %.3f" %(euler_angles[0], euler_angles[1], euler_angles[2]))
        
        # radian check the plus/minus
        Roll = euler_angles[0] * np.pi / 180.0
        Pitch = -euler_angles[1] * np.pi / 180.0
        Yaw = -euler_angles[2] * np.pi / 180.0

        return Roll, Pitch, Yaw

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        self.qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return self.qx, self.qy, self.qz, self.qw

if __name__ == '__main__':
    try:
        rospy.init_node('camera_sub')
        cm_camera_sub = CameraImage()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
