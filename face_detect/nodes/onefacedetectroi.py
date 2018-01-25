#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import roslib

import sys

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import RegionOfInterest
import pcl_ros

roslib.load_manifest('face_detect')

min_size = (10, 10)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
display = True

if __name__ == '__main__':
    opencv_dir = '/usr/share/opencv/haarcascades/'

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

    pub_roi = rospy.Publisher('one_face_roi', RegionOfInterest, queue_size=2)


    def detect_and_publish(imgmsg):
        if pub_roi.get_num_connections() > 0:
            img = br.imgmsg_to_cv2(imgmsg, "bgr8")
            # allocate temporary images
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            cv_faces = face_cascade.detectMultiScale(gray, 1.3, 3)
            if len(cv_faces) > 0:
                max_size = 0
                roi = RegionOfInterest()
                for i, (x, y, w, h) in enumerate(cv_faces):
                    size = h * w
                    # add ROI to roi_msg
                    if size > max_size:
                        max_size = size
                        roi.x_offset = 128 - x
                        roi.y_offset = 128 - y
                        roi.height = h
                        roi.width = w
                pub_roi.publish(roi)


    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_publish)
    rospy.spin()
