import roslib;

roslib.load_manifest('ar_mapping')
import rospy
import numpy as np
from numpy.linalg import inv
from math import pi, sin, cos
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped
import tf
import threading

import rover_driver
from ar_loc.rover_kinematics import *


class MappingKF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))
        self.P = np.mat(np.diag(initial_uncertainty))
        self.idx = {}
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("~landmarks", MarkerArray, queue_size=1)

    @staticmethod
    def getRotation(theta):
        R = np.mat(np.zeros((2, 2)))
        R[0, 0] = cos(theta)
        R[0, 1] = -sin(theta)
        R[1, 0] = sin(theta)
        R[1, 1] = cos(theta)
        return R

    @staticmethod
    def h_loc(X, L):
        """
        Compute the position of L in the X frame
            :param X: position of the rover [[x],[y],[theta]] on world frame
            :param L: position of the landmark [[x],[y]] on world frame
            :return pos_lm_frame: cartesian coordinate of L in the X frame (rover frame) [[x],[y]]
            :return dist: cartesian coordinate of the vector from X to L (world frame) [[x],[y]]
        """
        dist = np.mat(L - X[0:2])
        assert dist[0, 0] == L[0, 0] - X[0, 0] and dist[1, 0] == L[1, 0] - X[1, 0]
        return MappingKF.getRotation(-X[2, 0]) * dist, dist

    def predict(self, motor_state, drive_cfg, encoder_precision):
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return self.X, self.P
        # print "-"*32
        # then compute odometry using least square
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        self.motor_state.copy(motor_state)

        # Implement Kalman prediction here
        theta = self.X[2, 0]
        Rtheta = np.mat([[cos(theta), -sin(theta), 0],
                         [sin(theta),  cos(theta), 0],
                         [0,           0,          1]])
        DeltaX = iW * S
        # Update the state using odometry (same code as for the localisation
        # homework), but we only need to deal with a subset of the state:
        # TODO: encoder_precision is the error on S
        Q = np.mat(
            [
                [encoder_precision**2, 0, 0],
                [0, encoder_precision**2, 0],
                [0, 0, encoder_precision**2]
            ])
        # TODO: compute the real F matrix...
        F = np.mat(
            [
                [1, 0, -sin(theta) * DeltaX[0, 0] - cos(theta) * DeltaX[1, 0]],
                [0, 1,  cos(theta) * DeltaX[0, 0] - sin(theta) * DeltaX[1, 0]],
                [0, 0,                                                      1]
            ])

        movement = np.matmul(Rtheta, DeltaX)
        movement[2, 0] = ((movement[2, 0] + pi) % (2 * pi)) - pi  # ensure movement angle in [-pi;pi]
        # ultimately :
        self.X[0:3, 0] += movement
        self.P[0:3, 0:3] = F.T * self.P[0:3, 0:3] * F + Q

        self.lock.release()
        assert type(F) == np.matrixlib.defmatrix.matrix and type(Q) == np.matrixlib.defmatrix.matrix
        assert type(self.P) == np.matrixlib.defmatrix.matrix
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        return self.X, self.P

    def update_ar(self, Z, id, uncertainty):
        # Landmark id has been observed as Z with a given uncertainty
        self.lock.acquire()
        print "Update: Z=" + str(Z.T) + " X=" + str(self.X.T) + " Id=" + str(id)
        # Update the full state self.X and self.P based on landmark id
        # be careful that this might be the first time that id is observed
        R = np.mat(np.diag([uncertainty**2] * 2))  # 2x2
        theta = self.X[2, 0]
        (lx, _) = self.X.shape
        if id in self.idx:
            lid = self.idx[id]
            y_cart, dist = self.h_loc(self.X[0:3, 0], self.X[lid:lid+3, 0])
            y = Z - y_cart
            H = np.mat(np.zeros((2, lx)))
            H[:, 0:3] = np.mat(
                [
                    [-cos(theta), -sin(theta),  dist[1, 0] * cos(theta) - dist[0, 0] * sin(theta)],
                    [sin(theta),  -cos(theta), -dist[0, 0] * cos(theta) - dist[1, 0] * sin(theta)]
                ]
            )
            H[0:2, lid:lid+3] = self.getRotation(-theta)
            S = H * self.P * H.T + R  # 2x2
            K = self.P * H.T * np.mat(np.linalg.inv(S))  # 2x3

            self.X += K * y  # 1x3
            self.P = (np.identity(lx) - K * H) * self.P  # 3x3

            # update landmark and rover position
        else:
            L = self.getRotation(theta) * (self.X[0:2] - Z)
            self.X = np.vstack((self.X, L))
            new_P = np.mat(np.identity(lx+2)) * uncertainty
            new_P[0:lx, 0:lx] = self.P
            self.P = new_P
            self.idx[id] = lx

        self.lock.release()

        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        return self.X, self.P

    def update_compass(self, Z, uncertainty):
        pass
        # assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        # self.lock.acquire()
        # # print "Update: S=" + str(Z) + " X=" + str(self.X.T)
        # # Implement kalman update using compass here
        # y_polar = np.mat([[Z - self.X[2, 0]]])  # 1x1
        # H = np.mat(  # 3x1
        #     [
        #         [0, 0, 1],
        #     ]
        # )
        # R = np.mat(np.diag([uncertainty] * 1))  # 1x1
        # S = H * self.P * H.T + R  # 1x1
        # K = self.P * H.T * np.mat(np.linalg.inv(S))  # 1x3
        #
        # y_polar[0, 0] = ((y_polar[0, 0] + pi) % (2 * pi)) - pi  # ensure movement angle in [-pi;pi]
        # self.X += K * y_polar  # 1x3
        # self.P = (np.identity(3) - K * H) * self.P  # 3x3
        # self.lock.release()
        # assert type(y_polar) == np.matrixlib.defmatrix.matrix and type(H) == np.matrixlib.defmatrix.matrix
        # assert type(R) == np.matrixlib.defmatrix.matrix
        # assert type(S) == np.matrixlib.defmatrix.matrix and type(K) == np.matrixlib.defmatrix.matrix
        # assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        # return self.X, self.P

    def publish(self, target_frame, timestamp):
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = self.X[0, 0]
        pose.pose.position.y = self.X[1, 0]
        pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2, 0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        self.pose_pub.publish(pose)
        ma = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 5000
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = -0.1
        marker.scale.x = 3 * np.sqrt(self.P[0, 0])
        marker.scale.y = 3 * np.sqrt(self.P[1, 1])
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        ma.markers.append(marker)
        for id in self.idx:
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = target_frame
            marker.ns = "landmark_kf"
            marker.id = id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            lx = self.idx[id]
            marker.pose.position.x = self.X[lx, 0]
            marker.pose.position.y = self.X[lx+1, 0]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 1
            marker.pose.orientation.w = 0
            marker.scale.x = max(3 * np.sqrt(self.P[lx, 0]), 0.05)
            marker.scale.y = max(3 * np.sqrt(self.P[lx+1, 1]), 0.05)
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
            marker.pose.position.x = self.X[lx, 0]
            marker.pose.position.y = self.X[lx+1, 0]
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

        # for id in self.idx.iterkeys():
        #     marker = Marker()
        #     marker.header.stamp = timestamp
        #     marker.header.frame_id = target_frame
        #     marker.ns = "landmark_kf"
        #     marker.id = id
        #     marker.type = Marker.CYLINDER
        #     marker.action = Marker.ADDfrontCamera
        #     l = self.idx[id]
        #     marker.pose.position.x = self.X[l, 0]
        #     marker.pose.position.y = self.X[l + 1, 0]
        #     marker.pose.position.z = -0.1
        #     marker.pose.orientation.x = 0
        #     marker.pose.orientation.y = 0
        #     marker.pose.orientation.z = 1
        #     marker.pose.orientation.w = 0
        #     marker.scale.x = 3 * np.sqrt(self.P[l, l])
        #     marker.scale.y = 3 * np.sqrt(self.P[l + 1, l + 1])
        #     marker.scale.z = 0.1
        #     marker.color.a = 1.0
        #     marker.color.r = 1.0
        #     marker.color.g = 1.0
        #     marker.color.b = 0.0
        #     marker.lifetime.secs = 3.0
        #     ma.markers.append(marker)
        #     marker = Marker()
        #     marker.header.stamp = timestamp
        #     marker.header.frame_id = target_frame
        #     marker.ns = "landmark_kf"
        #     marker.id = 1000 + id
        #     marker.type = Marker.TEXT_VIEW_FACING
        #     marker.action = Marker.ADD
        #     marker.pose.position.x = self.X[l + 0, 0]
        #     marker.pose.position.y = self.X[l + 1, 0]
        #     marker.pose.position.z = 1.0
        #     marker.pose.orientation.x = 0
        #     marker.pose.orientation.y = 0
        #     marker.pose.orientation.z = 1
        #     marker.pose.orientation.w = 0
        #     marker.text = str(id)
        #     marker.scale.x = 1.0
        #     marker.scale.y = 1.0
        #     marker.scale.z = 0.2
        #     marker.color.a = 1.0
        #     marker.color.r = 1.0
        #     marker.color.g = 1.0
        #     marker.color.b = 1.0
        #     marker.lifetime.secs = 3.0
        #     ma.markers.append(marker)
        self.marker_pub.publish(ma)
