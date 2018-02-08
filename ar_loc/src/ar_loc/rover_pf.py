import roslib;

roslib.load_manifest('ar_loc')
import rospy
# from numpy import *
import numpy as np
from numpy.linalg import pinv, inv
from math import pi, sin, cos
from geometry_msgs.msg import *
import tf
import bisect
import threading
import math

from rover_kinematics import *


class RoverPF(RoverKinematics):
    def __init__(self, initial_pose, initial_uncertainty):
        RoverKinematics.__init__(self)
        self.initial_uncertainty = initial_uncertainty
        self.lock = threading.Lock()
        self.X = np.mat(np.vstack(initial_pose))
        # Initialisation of the particle cloud around the initial position
        self.N = 500
        self.particles = np.array([self.X + self.drawNoise(initial_uncertainty) for i in range(0, self.N)])
        self.pa_pub = rospy.Publisher("~particles", PoseArray, queue_size=1)

    def getRotation(self, theta):
        R = np.mat(np.zeros((2, 2)))
        R[0, 0] = cos(theta)
        R[0, 1] = -sin(theta)
        R[1, 0] = sin(theta)
        R[1, 1] = cos(theta)
        return R

    # Draw a vector uniformly around [0,0,0], scaled by norm
    def drawNoise(self, norm):
        if type(norm) == list:
            return np.mat(np.vstack(norm) * (2 * np.random.rand(3, 1) - np.vstack([1, 1, 1])))
        else:
            return np.mat(np.multiply(norm, (2 * np.random.rand(3, 1) - np.vstack([1, 1, 1]))))

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

        # Apply the particle filter prediction step here

        # Estimate DeltaX using the pseudo-inverse method
        DeltaX = iW * S
        for p in self.particles:
            theta = p[2, 0]
            Rtheta = np.mat([[cos(theta), -sin(theta), 0],
                             [sin(theta), cos(theta),  0],
                             [0,          0,           1]])
            movement = np.matmul(Rtheta, DeltaX)
            dist = np.hypot(movement[0], movement[1])
            p += movement
            p += np.random.normal(0.0, encoder_precision * dist * 10, (3, 1))

        # self.particles = ...

        self.lock.release()

    def update_ar(self, Z, L, Uncertainty):
        self.lock.acquire()
        # print "Update: L=" + str(L.T)
        # Implement particle filter update using landmarks here
        # Note: the function bisect.bisect_left could be useful to implement
        # the resampling process efficiently
        score = np.zeros(len(self.particles))
        for i, p in enumerate(self.particles):
            theta = p[2, 0]
            Rtheta = np.mat([[cos(theta), -sin(theta)],
                             [sin(theta), cos(theta)]])
            diff = p[0:2] + np.matmul(Rtheta, Z) - L
            score[i] = np.sum(np.abs(diff))

        score = np.exp(score / -Uncertainty)
        score /= np.sum(score)
        indices = np.random.choice(len(self.particles), size=self.N, replace=True, p=score)
        survivor = self.particles[indices]
        noise = np.random.normal(0.0, 0.01, (self.N, 3, 1))
        noise[:, 2, 0] /= 2 * pi * 5
        self.particles = survivor + noise
        # self.particles = ...

        self.lock.release()

    def update_compass(self, angle, Uncertainty):
        self.lock.acquire()
        # print "Update: C=" + str(angle)
        # Implement particle filter update using landmarks here
        # Note: the function bisect.bisect_left could be useful to implement
        # the resampling process efficiently
        score = self.particles[:, 2, 0] - angle + 3 * pi
        score = np.fmod(score, 2 * pi) - pi
        score = np.exp(np.abs(score) / -Uncertainty)
        score /= np.sum(score)

        indices = np.random.choice(len(self.particles), size=self.N, replace=True, p=score)
        survivor = self.particles[indices]
        noise = np.random.normal(0.0, 0.01, (self.N, 3, 1))
        noise[:, 2, 0] /= 2 * pi * 5
        self.particles = survivor + noise

        # self.particles = ...

        self.lock.release()

    def updateMean(self):
        X = np.mat(np.zeros((3, 1)))
        for x in self.particles:
            X += x
        self.X = X / len(self.particles)

        return self.X

    def publish(self, pose_pub, target_frame, stamp):
        # Only compute the mean for plotting
        self.updateMean()
        pose = PoseStamped()
        pose.header.frame_id = target_frame
        pose.header.stamp = stamp
        pose.pose.position.x = self.X[0, 0]
        pose.pose.position.y = self.X[1, 0]
        pose.pose.position.z = 0.0
        Q = tf.transformations.quaternion_from_euler(0, 0, self.X[2, 0])
        pose.pose.orientation.x = Q[0]
        pose.pose.orientation.y = Q[1]
        pose.pose.orientation.z = Q[2]
        pose.pose.orientation.w = Q[3]
        pose_pub.publish(pose)

        pa = PoseArray()
        pa.header = pose.header
        for p in self.particles:
            po = Pose()
            po.position.x = p[0, 0]
            po.position.y = p[1, 0]
            q = tf.transformations.quaternion_from_euler(0, 0, p[2, 0])
            po.orientation = Quaternion(*q)
            pa.poses.append(po)
        self.pa_pub.publish(pa)

    def broadcast(self, br, target_frame, stamp):
        br.sendTransform((self.X[0, 0], self.X[1, 0], 0),
                         tf.transformations.quaternion_from_euler(0, 0, self.X[2, 0]),
                         stamp, "/%s/ground" % self.name, target_frame)
