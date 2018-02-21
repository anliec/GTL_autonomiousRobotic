import roslib;

roslib.load_manifest('ar_mapping')
import rospy
import numpy as np
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

import rover_driver
from rover_driver.rover_kinematics import *


class Landmark:

    def h(self, X):
        xr = X[0, 0]
        yr = X[1, 0]
        tr = X[2, 0]
        xl = self.L[0, 0]
        yl = self.L[1, 0]
        return np.mat([[np.sqrt((xr - xl) ** 2 + (yr - yl) ** 2)], [np.math.atan2(yl - yr, xl - xr) - tr]])

    def __init__(self, Z, X, R):
        # Initialise a landmark based on measurement Z, 
        # current position X and uncertainty R
        self.L = np.vstack([0, 0])
        self.P = np.mat([[0.1, 0], [0, 0.1]])

        xr = X[0, 0]
        yr = X[1, 0]
        tr = X[2, 0]
        xl = self.L[0, 0]
        yl = self.L[1, 0]
        divider = np.math.sqrt((xr-xl)**2+(yr-yl)**2)
        divider2 = (xl-yr)**2+(yl-xr)**2
        # from wolfram alpha: jacobian (sqrt((a-x)^2+(b-y)^2), atan2(y-a,x-b))
        # https://www.wolframalpha.com/input/?i=jacobian+(sqrt((a-x)%5E2%2B(b-y)%5E2),+atan2(y-a,x-b))
        self.H = np.transpose(np.mat(
            [
                [-(xr-xl)/divider, -(yr-yl)/divider],
                [(xr-yl)/divider2, (xl-yr)/divider2]
            ]))
        theta = Z[1, 0] + X[2, 0]
        self.L = X[0:2] + float(Z[0, 0]) * np.mat([[cos(theta)], [sin(theta)]])

    def update(self, Z, X, R):
        """
        :param Z: position of the landmark in the Robot frame
        :param X: position of the robot in the world frame
        :param R: covariance matrix of the observation noise
        :return:
        """
        # Update the landmark based on measurement Z,
        # current position X and uncertainty R
        y = Z - self.h(X)
        S = R + self.H * self.P * self.H.T
        K = self.P * self.H.T * np.mat(np.linalg.inv(S))
        self.L = self.L + K * y
        self.P = (np.mat(np.identity(2)) - K * self.H) * self.P
        print ("P:"+str(self.P))
        return self.L


class MappingKF:
    def __init__(self):
        self.lock = threading.Lock()
        self.marker_list = {}
        self.marker_pub = rospy.Publisher("~landmarks", MarkerArray, queue_size=1)

    def update_ar(self, Z, X, Id, uncertainty):
        self.lock.acquire()
        # print "Update: Z=" + str(Z.T) + " X=" + str(X.T) + " Id=" + str(Id)
        R = np.mat(np.diag([uncertainty ** 2, uncertainty ** 2]))
        # Take care of the landmark Id observed as Z from X
        # self.marker_list is expected to be a dictionary of Landmark
        # such that current landmark can be retrieved as self.marker_list[Id] 
        # At initialisation, self.marker_list is empty
        # TODO
        if Id in self.marker_list:
            self.marker_list[Id].update(Z, X, R)
            print('Id:' + str(Id))
            print(self.marker_list[Id].L)
        else:
            self.marker_list[Id] = Landmark(Z, X, R)
        self.lock.release()

    def publish(self, target_frame, timestamp):
        ma = MarkerArray()
        for id in self.marker_list:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0, 0]
            marker.pose.position.y = Lkf.L[1, 0]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = max(3 * np.sqrt(Lkf.P[0, 0]), 0.05)
            marker.scale.y = max(3 * np.sqrt(Lkf.P[1, 1]), 0.05)
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime.secs = 3.0
            ma.markers.append(marker)
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = 1000 + id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            Lkf = self.marker_list[id]
            marker.pose.position.x = Lkf.L[0, 0]
            marker.pose.position.y = Lkf.L[1, 0]
            marker.pose.position.z = 1.0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.text = str(id)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.lifetime.secs = 3.0
            ma.markers.append(marker)
        self.marker_pub.publish(ma)
