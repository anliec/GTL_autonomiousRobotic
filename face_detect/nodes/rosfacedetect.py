#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib

import sys
import os

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from face_detect.msg import faces
from sensor_msgs.msg import RegionOfInterest

roslib.load_manifest('face_detect')

min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True

if __name__ == '__main__':
    opencv_dir = '/usr/share/opencv/haarcascades/';

    face_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print "Could not find face cascade"
        sys.exit(-1)
    eye_cascade = cv2.CascadeClassifier(opencv_dir + 'haarcascade_eye.xml')
    if eye_cascade.empty():
        print "Could not find eye cascade"
        sys.exit(-1)
    br = CvBridge()
    rospy.init_node('facedetect')
    display = rospy.get_param("~display", True)

    pub = rospy.Publisher('facedetect', faces, queue_size=5)


    def detect_and_publish(imgmsg):
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        # allocate temporary images
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv_faces = face_cascade.detectMultiScale(gray, 1.3, 3)
        message = faces()
        message.faces_rois = []
        for (x, y, w, h) in cv_faces:
            roi = RegionOfInterest()
            roi.x_offset = x
            roi.y_offset = y
            roi.height = h
            roi.width = w
            message.faces_rois.append(roi)
        pub.publish(message)


    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_publish)
    rospy.spin()
