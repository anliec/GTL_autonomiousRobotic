import roslib;

roslib.load_manifest('ar_loc')
import rospy
import numpy as np
from numpy.linalg import pinv, inv
from math import pi, sin, cos, fmod
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf
import threading

from rover_kinematics import *


class RoverKF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        """
        :param initial_pose:
        :param initial_uncertainty: [1.0, 1.0, 1.0]
        """
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))  # 1x3
        self.P = np.mat(np.diag(initial_uncertainty))  # 3x3
        self.ellipse_pub = rospy.Publisher("~ellipse", Marker, queue_size=1)
        self.pose_with_cov_pub = rospy.Publisher("~pose_with_covariance", PoseWithCovarianceStamped, queue_size=1)
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix

    @staticmethod
    def getRotation(theta):
        R = np.mat(np.zeros((2, 2)))
        R[0, 0] = cos(theta)
        R[0, 1] = -sin(theta)
        R[1, 0] = sin(theta)
        R[1, 1] = cos(theta)
        return R

    @staticmethod
    def h(X, L):
        """
        Compute the position of L in the X frame
            :param X: position of the rover [[x],[y],[theta]] on world frame
            :param L: position of the landmark [[x],[y]] on world frame
            :return pos_lm_frame: cartesian coordinate of L in the X frame (rover frame) [[x],[y]]
            :return dist: cartesian coordinate of the vector from X to L (world frame) [[x],[y]]
        """
        dist = np.mat(L - X[0:2])
        assert dist[0, 0] == L[0, 0] - X[0, 0] and dist[1, 0] == L[1, 0] - X[1, 0]
        return RoverKF.getRotation(-X[2, 0]) * dist, dist

    def predict(self, motor_state, drive_cfg, encoder_precision):
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        self.lock.acquire()
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.first_run = False
            self.lock.release()
            return
        # Prepare odometry matrices (check rover_odo.py for usage)
        iW = self.prepare_inversion_matrix(drive_cfg)
        S = self.prepare_displacement_matrix(self.motor_state, motor_state, drive_cfg)
        self.motor_state.copy(motor_state)

        # Implement Kalman prediction here
        DeltaX = iW * S

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
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ])

        theta = self.X[2, 0]
        Rtheta = np.mat([[cos(theta), -sin(theta), 0],
                         [sin(theta), cos(theta), 0],
                         [0, 0, 1]])
        movement = np.matmul(Rtheta, DeltaX)
        # ultimately : 
        self.X += movement
        self.P = F * self.P * F.T + Q

        self.lock.release()
        assert type(F) == np.matrixlib.defmatrix.matrix and type(Q) == np.matrixlib.defmatrix.matrix
        assert type(self.P) == np.matrixlib.defmatrix.matrix
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix

    def update_ar(self, Z, L, uncertainty):
        """
        :param Z: observation in the rover frame [[x],[y]]
        :param L: position of the landmark on world frame [[x],[y]]
        :param uncertainty:
        :return:
        """
        self.lock.acquire()
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        # print "Update: L=" + str(L.T) + " X=" + str(self.X.T) + " uncertainty=" + str(uncertainty)
        # Implement kalman update using landmarks here

        y_cart, dist = self.h(self.X, L)
        y_cart = Z - y_cart
        theta = self.X[2, 0]
        # https://www.wolframalpha.com/input/?i=jacobian(%5B%5Bcos(z)*(a-x)+-+sin(z)*(b-y)%5D,+%5Bsin(z)*(a-x)+%2B+cos(z)*(b-y)%5D%5D)
        H = np.mat(
            [
                [-cos(theta), -sin(theta),  dist[1, 0] * cos(theta) - dist[0, 0] * sin(theta)],
                [sin(theta),  -cos(theta), -dist[0, 0] * cos(theta) - dist[1, 0] * sin(theta)]
            ]
        )
        R = np.mat(np.diag([uncertainty] * 2))  # 2x2
        S = H * self.P * H.T + R  # 2x2
        K = self.P * H.T * np.mat(np.linalg.inv(S))  # 2x3

        self.X += K * y_cart  # 1x3
        self.P = (np.identity(3) - K * H) * self.P  # 3x3

        self.lock.release()
        print(y_cart)
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix
        assert type(H) == np.matrixlib.defmatrix.matrix and type(S) == np.matrixlib.defmatrix.matrix
        assert type(K) == np.matrixlib.defmatrix.matrix and type(y_cart) == np.matrixlib.defmatrix.matrix
        assert type(R) == np.matrixlib.defmatrix.matrix

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

        self.X += K * y_polar  # 1x3
        self.P = (np.identity(3) - K * H) * self.P  # 3x3
        self.lock.release()
        assert type(y_polar) == np.matrixlib.defmatrix.matrix and type(H) == np.matrixlib.defmatrix.matrix
        assert type(R) == np.matrixlib.defmatrix.matrix
        assert type(S) == np.matrixlib.defmatrix.matrix and type(K) == np.matrixlib.defmatrix.matrix
        assert type(self.X) == np.matrixlib.defmatrix.matrix and type(self.P) == np.matrixlib.defmatrix.matrix

    def publish(self, pose_pub, target_frame, stamp):
        """
        This publishes the pose but also the pose with covariance and the error ellipse in rviz
        :param pose_pub:
        :param target_frame:
        :param stamp:
        :return:
        """
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = stamp
        pose.pose.pose.position.x = self.X[0, 0]
        pose.pose.pose.position.y = self.X[1, 0]
        pose.pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2, 0])
        pose.pose.pose.orientation.x = Q[0]
        pose.pose.pose.orientation.y = Q[1]
        pose.pose.pose.orientation.z = Q[2]
        pose.pose.pose.orientation.w = Q[3]
        psub = PoseStamped()
        psub.header = pose.header
        psub.pose = pose.pose.pose
        pose_pub.publish(psub)
        C = [0] * 36
        C[0] = self.P[0, 0]
        C[1] = self.P[0, 1]
        C[5] = self.P[0, 2]
        C[6] = self.P[1, 0]
        C[7] = self.P[1, 1]
        C[11] = self.P[1, 2]
        C[30] = self.P[2, 0]
        C[31] = self.P[2, 1]
        C[35] = self.P[2, 2]
        pose.pose.covariance = C
        self.pose_with_cov_pub.publish(pose)
        marker = Marker()
        marker.header = pose.header
        marker.ns = "kf_uncertainty"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose.pose
        marker.scale.x = 3 * np.sqrt(self.P[0, 0])
        marker.scale.y = 3 * np.sqrt(self.P[1, 1])
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.ellipse_pub.publish(marker)

    # broadcast the estimated transform
    def broadcast(self, br, target_frame, stamp):
        br.sendTransform((self.X[0, 0], self.X[1, 0], 0),
                         tf.transformations.quaternion_from_euler(0, 0, self.X[2, 0]),
                         stamp, "/%s/ground" % self.name, target_frame)
