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
from sensor_msgs.msg import RegionOfInterest, Image, PointCloud2
from visualization_msgs.msg import Marker
import pcl_ros

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

    pub_roi = rospy.Publisher('facedetect', faces, queue_size=5)
    pub_im = rospy.Publisher('facedetect_cim', Image, queue_size=2)
    pub_mk = rospy.Publisher('facedetect_marker', Marker, queue_size=5)


    def detect_and_publish(imgmsg):
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        # allocate temporary images
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv_faces = face_cascade.detectMultiScale(gray, 1.3, 3)
        # initialise roi msg
        roi_msg = faces()
        roi_msg.faces_rois = []
        # initialise Marker
        marker = Marker()
        for i, (x, y, w, h) in enumerate(cv_faces):
            # add ROI to roi_msg
            if pub_roi.get_num_connections() > 0:
                roi = RegionOfInterest()
                roi.x_offset = x
                roi.y_offset = y
                roi.height = h
                roi.width = w
                roi_msg.faces_rois.append(roi)
            # generate image to publish
            if pub_im.get_num_connections() > 0:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                roi_gray = gray[y:y + h, x:x + w]
                roi_color = img[y:y + h, x:x + w]
                eyes = eye_cascade.detectMultiScale(roi_gray)
                for (ex, ey, ew, eh) in eyes:
                    cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2)
            # setup marker and publish it
            if pub_mk.get_num_connections() > 0:
                marker.header.frame_id = "Hokuyo"
                marker.header.stamp = rospy.Time.now()
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.ns = "face"
                marker.pose.position.x = 1.0
                marker.pose.position.y = -(x - 128) / 100.0
                marker.pose.position.z = -(y - 128) / 100.0
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1
                marker.color.r = 0.0
                
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.scale.x = 0.03
                marker.scale.y = w / 100.0
                marker.scale.z = h / 100.0
                marker.lifetime = rospy.Duration()
                # publish marker
                pub_mk.publish(marker)
        # publish ROI vector
        if pub_roi.get_num_connections() > 0:
            pub_roi.publish(roi_msg)
        # publish image with indicators
        if pub_im.get_num_connections() > 0:
            msg_image = br.cv2_to_imgmsg(img)
            pub_im.publish(msg_image)


    rospy.Subscriber("~image", sensor_msgs.msg.Image, detect_and_publish)
    rospy.spin()
