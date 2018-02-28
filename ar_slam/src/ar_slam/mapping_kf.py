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


class Landmark:

    def __init__(self, Z, X, R, Px):
        # Initialise a landmark based on measurement Z,
        # current position X and uncertainty R
        self.L = np.vstack([0, 0])
        xr = X[0, 0]
        yr = X[1, 0]
        tr = X[2, 0]
        xl = self.L[0, 0]
        yl = self.L[1, 0]
        self.H = np.mat(
            [
                [-cos(tr), -sin(tr),  (yl-yr)*cos(tr)-(xl-xr)*sin(tr),  cos(tr), -sin(tr)],
                [sin(tr),  -cos(tr), -(xl-xr)*cos(tr)-(yl-yr)*sin(tr),  sin(tr), cos(tr)]
            ]
        )
        # self.dfdz = np.mat(
        #     [
        #         [cos(tr), sin(tr)],
        #         [-sin(tr), cos(tr)]
        #     ])
        dfdz = self.H[:, 3:5]  # in this case h and baseChange has the same Jacobian
        self.P = np.mat(np.zeros((5, 5)))
        self.P[0:3, 0:3] = Px
        print dfdz * R * dfdz.T
        self.P[3:5, 3:5] = dfdz * R * dfdz.T  #
        # from wolfram alpha:
        # https://www.wolframalpha.com/input/?i=jacobian+(+(x-a)*cos(c)-(y-b)*sin(c)+,+(y-b)*cos(c)%2B(x-a)*sin(c)+)

        baseChange = np.mat([
            [cos(tr), -sin(tr), xr],
            [sin(tr), cos(tr), yr]
        ])
        self.L = baseChange * np.vstack((Z, [1]))

    def update(self, Z, X, R, Px):
        """
        :param Z: position of the landmark in the Robot frame
        :param X: position of the robot in the world frame
        :param R: covariance matrix of the observation noise
        :param Px: covariance matrix of the robot localisation
        :return:
        """
        # Update the landmark based on measurement Z,
        # current position X and uncertainty R
        tr = X[2, 0]
        self.P[0:3, 0:3] = Px
        y = Z - MappingKF.h_loc(X, self.L)  # 2x1 + 2x1

        dfdz = self.H[:, 3:5]

        S = dfdz * R * dfdz + self.H * self.P * self.H.T  # 2x2 + 2x5 * 5x5 * 5x2
        K = self.P * self.H.T * np.mat(np.linalg.inv(S))  # 5x5 * 5x2 * 2x2
        new_X = np.mat(np.vstack((X, self.L))) + K * y  # 5x1 + 5x2 * 2x1
        self.L = new_X[3:5, 0]
        X = new_X[0:3, 0]
        self.P = (np.mat(np.identity(5)) - K * self.H) * self.P  # (5x5 - 5x2*2x5) * 5x5
        print ("X:" + str(X))
        print ("Z:" + str(Z))
        assert type(self.H) == np.matrixlib.defmatrix.matrix and type(S) == np.matrixlib.defmatrix.matrix
        assert type(K) == np.matrixlib.defmatrix.matrix
        assert type(R) == np.matrixlib.defmatrix.matrix
        return X, self.P[0:3, 0:3]


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
        self.P[0:3, 0:3] = F.T * self.P * F + Q

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
        if id in self.idx:
            # y_cart, dist = self.h_loc(self.X[0:3, 0], self.idx[id].L)
            # y_cart = Z - y_cart
            # theta = self.X[2, 0]
            # # https://www.wolframalpha.com/input/?i=jacobian(%5B%5Bcos(z)*(a-x)+-+sin(z)*(b-y)%5D,+%5Bsin(z)*(a-x)+%2B+cos(z)*(b-y)%5D%5D)
            # H = np.mat(
            #     [
            #         [-cos(theta), -sin(theta),  dist[1, 0] * cos(theta) - dist[0, 0] * sin(theta)],
            #         [sin(theta),  -cos(theta), -dist[0, 0] * cos(theta) - dist[1, 0] * sin(theta)]
            #     ]
            # )
            # S = H * self.P[0:3, 0:3] * H.T + R  # 2x2
            # K = self.P[0:3, 0:3] * H.T * np.mat(np.linalg.inv(S))  # 2x3
            #
            # self.X[0:3, 0] += K * y_cart  # 1x3
            # self.P[0:3, 0:3] = (np.identity(3) - K * H) * self.P[0:3, 0:3]  # 3x3

            # update landmark and rover position
            (self.X, self.P) = self.idx[id].update(Z, self.X, R, self.P)
        else:
            self.idx[id] = Landmark(Z, self.X, R, self.P)

        self.lock.release()

        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        return self.X, self.P

    def update_compass(self, Z, uncertainty):
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        self.lock.acquire()
        # print "Update: S=" + str(Z) + " X=" + str(self.X.T)
        # Implement kalman update using compass here
        y_polar = np.mat([[Z - self.X[2, 0]]])  # 1x1
        H = np.mat(  # 3x1
            [
                [0, 0, 1],
            ]
        )
        R = np.mat(np.diag([uncertainty] * 1))  # 1x1
        S = H * self.P * H.T + R  # 1x1
        K = self.P * H.T * np.mat(np.linalg.inv(S))  # 1x3

        y_polar[0, 0] = ((y_polar[0, 0] + pi) % (2 * pi)) - pi  # ensure movement angle in [-pi;pi]
        self.X += K * y_polar  # 1x3
        self.P = (np.identity(3) - K * H) * self.P  # 3x3
        self.lock.release()
        assert type(y_polar) == np.matrixlib.defmatrix.matrix and type(H) == np.matrixlib.defmatrix.matrix
        assert type(R) == np.matrixlib.defmatrix.matrix
        assert type(S) == np.matrixlib.defmatrix.matrix and type(K) == np.matrixlib.defmatrix.matrix
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        return self.X, self.P

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
            Lkf = self.idx[id]
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
            Lkf = self.idx[id]
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
