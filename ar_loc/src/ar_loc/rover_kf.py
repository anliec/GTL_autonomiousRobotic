import roslib;

roslib.load_manifest('ar_loc')
import rospy
import numpy as np
from numpy.linalg import pinv, inv
from math import pi, sin, cos
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
        print "initial_uncertainty:", initial_uncertainty
        RoverKinematics.__init__(self)
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))  # 1x3
        self.P = np.mat(np.diag(initial_uncertainty))  # 3x3
        self.ellipse_pub = rospy.Publisher("~ellipse", Marker, queue_size=1)
        self.pose_with_cov_pub = rospy.Publisher("~pose_with_covariance", PoseWithCovarianceStamped, queue_size=1)

    def getRotation(self, theta):
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
            :return pos_lm_frame: polar coordinate of L in the X frame (rover frame)
            :return dist: cartesian coordinate of the vector from X to L (world frame)
        """
        dist = L - X[0:2]
        pos_lm_rover_frame = np.mat(
            [
                [np.linalg.norm(dist, 2)],
                [atan2(dist[0, 0], dist[1, 0]) - X[2]]
            ]
        )
        return pos_lm_rover_frame, dist

    def predict(self, motor_state, drive_cfg, encoder_precision):
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

        Q = np.mat(
            [
                [0.01, 0, 0],
                [0, 0.01, 0],
                [0, 0, 0.01]
            ])
        # TODO: compute the real F matrix...
        F = np.mat(
            [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ])

        # ultimately : 
        self.X += DeltaX
        self.P = F * self.P * F.T + Q

        self.lock.release()

    def update_ar(self, Z, L, uncertainty):
        """
        :param Z: observation in the rover frame [[d],[theta]]
        :param L: position of the landmark on world frame [[x],[y]]
        :param uncertainty:
        :return:
        """
        self.lock.acquire()
        print "Update: L=" + str(L.T) + " X=" + str(self.X.T) + " uncertainty=" + str(uncertainty)
        # Implement kalman update using landmarks here
        # TODO

        y_polar, _ = Z - self.h(self.X, L)
        theta = y_polar[1] + self.X[2, 0]
        y_cart = np.mat(
            [
                [cos(theta) * y_polar[0, 0]],
                [sin(theta) * y_polar[0, 0]],
                [y_polar[1, 0]]
            ]
        )
        squared_norme = np.sum(np.power(self.X[0:2], 2))
        norme = np.sqrt(squared_norme)
        H = np.mat(  # done ? H is the jacobian of h -> cf code mapping
            [
                [self.X[0, 0] / norme,          self.X[1, 0] / norme,         0],
                [-self.X[1, 0] / squared_norme, self.X[0, 0] / squared_norme, 0]
            ]
        )
        R = np.diag([uncertainty] * 2)  # 2x2  # done ? R is not zero
        S = H * self.P * H.T + R  # 2x2
        K = self.P * H.T * np.mat(np.linalg.inv(S))  # 2x3

        self.X += K * y_cart  # 1x3
        self.P = (np.identity(3) - K * H) * self.P  # 3x3
        self.lock.release()

    def update_compass(self, Z, uncertainty):
        self.lock.acquire()
        print "Update: S=" + str(Z) + " X=" + str(self.X.T)
        # Implement kalman update using compass here
        # TODO
        # self.X = 
        # self.P = 
        self.lock.release()
        return

        # this publishes the pose but also the pose with covariance and the error ellipse in rviz

    def publish(self, pose_pub, target_frame, stamp):
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
