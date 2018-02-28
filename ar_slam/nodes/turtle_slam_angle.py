#!/usr/bin/env python
import roslib

roslib.load_manifest('ar_slam')

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from tf.transformations import euler_from_matrix, decompose_matrix, quaternion_from_euler, euler_from_quaternion
import message_filters
import threading
import numpy as np

from math import atan2, hypot, pi, cos, sin, fmod, sqrt

from ar_track_alvar_msgs.msg import AlvarMarkers

DIST_NEW_LANDMARK_THRESHOLD = 4**2


def norm_angle(x):
    return fmod(x + pi, 2 * pi) - pi


class BubbleSLAM:
    def __init__(self):
        rospy.init_node('bubble_slam')
        rospy.loginfo("Starting bubble rob slam")
        self.ignore_id = rospy.get_param("~ignore_id", False)
        self.target_frame = rospy.get_param("~target_frame", "/map")
        self.body_frame = rospy.get_param("~body_frame", "/base_link")
        self.odom_frame = rospy.get_param("~odom_frame", "/odom")
        self.ar_precision = rospy.get_param("~ar_precision", 0.5)
        self.position_uncertainty = rospy.get_param("~position_uncertainty", 0.01)
        self.angular_uncertainty = rospy.get_param("~angular_uncertainty", 0.01)
        self.initial_x = rospy.get_param("~initial_x", 0.0)
        self.initial_y = rospy.get_param("~initial_y", 0.0)
        self.initial_theta = rospy.get_param("~initial_theta", 0.0)
        # instantiate the right filter based on launch parameters
        initial_pose = [self.initial_x, self.initial_y, self.initial_theta]
        initial_uncertainty = [0.01, 0.01, 0.01]

        self.lock = threading.Lock()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_cb)
        self.X = np.mat(np.vstack(initial_pose))
        self.P = np.mat(np.diag(initial_uncertainty))
        self.idx = dict()
        self.pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher("~landmarks", MarkerArray, queue_size=1)
        self.marker_pub_pose = rospy.Publisher("~landmarks_pose", PoseArray, queue_size=1)

        rospy.sleep(1.0)
        now = rospy.Time.now()
        self.listener.waitForTransform(self.odom_frame, self.body_frame, now, rospy.Duration(5.0))
        (trans, rot) = self.listener.lookupTransform(self.odom_frame, self.body_frame, now)
        self.old_odom = np.mat(self.listener.fromTranslationRotation(trans, rot))

    def predict(self, Delta):

        # Implement Kalman prediction here
        theta = self.X[2, 0]
        pose_mat = np.mat([[cos(theta), -sin(theta), 0, self.X[0, 0]],
                        [sin(theta), cos(theta), 0, self.X[1, 0]],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1],
                        ])
        pose_mat = Delta * pose_mat
        self.X[0:2, 0] = pose_mat[0:2, 3:4]
        euler = euler_from_matrix(pose_mat[0:3, 0:3], 'rxyz')
        self.X[2, 0] = euler[2]  # Ignore the others
        Jx = np.mat([[1, 0, -sin(theta) * Delta[0, 3] - cos(theta) * Delta[1, 3]],
                  [0, 1, cos(theta) * Delta[0, 3] - sin(theta) * Delta[1, 3]],
                  [0, 0, 1]])
        Qs = np.mat(np.diag([self.position_uncertainty ** 2] * 3))
        P = self.P[0:3, 0:3]
        self.P[0:3, 0:3] = Jx * P * Jx.T + Qs
        return self.X, self.P

    @staticmethod
    def getRotation(theta):
        R = np.mat(np.zeros((2, 2)))
        R[0, 0] = cos(theta)
        R[0, 1] = -sin(theta)
        R[1, 0] = sin(theta)
        R[1, 1] = cos(theta)
        return R

    def update_ar(self, Z, id, uncertainty):
        # Z is a dictionary of id->np.vstack([x,y])
        # print "Update: Z=" + str(Z.T) + " X=" + str(self.X.T) + " Id=" + str(id)
        (n, _) = self.X.shape
        theta = self.X[2, 0]
        Rtheta = self.getRotation(theta)
        Rmtheta = self.getRotation(-theta)
        # H = np.mat(np.zeros((0, n)))
        if id in self.idx.keys():
            min_dist = None
            l = None
            for i in self.idx[id]:
                dist = np.sum(np.power(self.X[i:i + 2, 0] - self.X[0:2, 0], 2))
                if min_dist is None or dist < min_dist:
                    min_dist = dist
                    l = i
            if min_dist > DIST_NEW_LANDMARK_THRESHOLD:
                print "same landmark but too far, dist=", sqrt(min_dist), "id=", id, "list_size=", len(self.idx[id])
                self.idx[id].append(n)
                p_landmark = self.X[:2, 0] + (Rtheta * np.mat(Z[:2, 0]).T)
                a_landmark = np.mat(Z[2, 0] - theta)
                self.X = np.concatenate([self.X, p_landmark, a_landmark])
                Pnew = np.mat(np.diag([uncertainty] * (n + 3)))
                Pnew[0:n, 0:n] = self.P
                self.P = Pnew
            else:
                H = np.mat(np.zeros((3, n)))
                H[0:2, 0:2] = -Rtheta
                H[0:3, 2] = np.mat(np.vstack(
                    [-(self.X[l + 0, 0] - self.X[0, 0]) * sin(theta) - (self.X[l + 1, 0] - self.X[1, 0]) * cos(theta),
                     (self.X[l + 0, 0] - self.X[0, 0]) * cos(theta) - (self.X[l + 1, 0] - self.X[1, 0]) * sin(theta),
                     -1]
                ))
                H[0:2, l:l + 2] = Rtheta
                H[0:3, l + 2] = np.mat(np.vstack(
                    [0,
                     0,
                     1]
                ))
                Zpred = np.mat(np.zeros((3, 1)))
                Zpred[:2, 0] = Rmtheta * (self.X[l:l + 2, 0] - self.X[0:2, 0])
                Zpred[2, 0] = norm_angle(self.X[l + 2, 0] - theta)
                R = np.mat(np.diag([uncertainty, uncertainty, pi / 2]))  # 3x3
                S = H * self.P * H.T + R  # 3x3
                K = self.P * H.T * np.linalg.inv(S)  # 6x3
                self.X += K * (Z - Zpred)
                self.P = (np.mat(np.eye(n)) - K * H) * self.P
        else:
            self.idx[id] = [n]
            p_landmark = self.X[:2, 0] + (Rtheta * np.mat(Z[:2, 0]).T)
            a_landmark = np.mat(Z[2, 0] - theta)
            self.X = np.concatenate([self.X, p_landmark, a_landmark])
            Pnew = np.mat(np.diag([uncertainty] * (n + 3)))
            Pnew[0:n, 0:n] = self.P
            self.P = Pnew
        return self.X, self.P

    def ar_cb(self, markers):
        for m in markers.markers:
            if m.id > 32:
                continue
            self.listener.waitForTransform(self.body_frame, m.header.frame_id, m.header.stamp, rospy.Duration(1.0))
            # m_pose = PointStamped()
            # m_pose.header = m.header
            # m_pose.point = m.pose.pose.position
            # m_pose = self.listener.transformPoint(self.body_frame, m_pose)
            m_pose = PoseStamped()
            m_pose.pose = m.pose.pose
            m_pose.header = m.header
            m_pose = self.listener.transformPose(self.body_frame, m_pose)
            q = m_pose.pose.orientation
            _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
            Z = np.vstack([m_pose.pose.position.x, m_pose.pose.position.y, theta])
            self.lock.acquire()
            if self.ignore_id:
                self.update_ar(Z, 0, self.ar_precision)
            else:
                self.update_ar(Z, m.id, self.ar_precision)
            self.lock.release()

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.listener.waitForTransform(self.odom_frame, self.body_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform(self.odom_frame, self.body_frame, now)
            new_odom = np.mat(self.listener.fromTranslationRotation(trans, rot))
            # print "================================================"
            # print new_odom
            # print self.old_odom
            odom = new_odom * np.linalg.inv(self.old_odom)
            self.old_odom = new_odom
            self.lock.acquire()
            self.predict(odom)
            theta = self.X[2, 0]
            pose_mat = np.mat([[cos(theta), -sin(theta), 0, self.X[0, 0]],
                            [sin(theta), cos(theta), 0, self.X[1, 0]],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1],
                            ])
            correction_mat = np.linalg.inv(new_odom) * pose_mat
            self.lock.release()
            scale, shear, angles, trans, persp = decompose_matrix(correction_mat)
            self.broadcaster.sendTransform(trans,
                                           quaternion_from_euler(*angles), now, self.odom_frame, self.target_frame)
            self.publish(now)
            rate.sleep()

    def publish(self, timestamp):
        # if np.min(self.P) < 0.0:
        #     print "negative value in P"
        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = timestamp
        pose.pose.position.x = self.X[0, 0]
        pose.pose.position.y = self.X[1, 0]
        pose.pose.position.z = 0.0
        Q = quaternion_from_euler(0, 0, self.X[2, 0])
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
        marker.scale.x = 3 * sqrt(abs(self.P[0, 0]))
        marker.scale.y = 3 * sqrt(abs(self.P[1, 1]))
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        ma.markers.append(marker)
        pa = PoseArray()
        pa.header = pose.header
        self.lock.acquire()
        for i in self.idx.iterkeys():
            for l in self.idx[i]:
                # pose
                landmark_pose = Pose()
                landmark_pose.position.x = self.X[l, 0]
                landmark_pose.position.y = self.X[l + 1, 0]
                landmark_pose.position.z = 0.0
                Q = quaternion_from_euler(0, 0, self.X[l + 2, 0])
                landmark_pose.orientation.x = Q[0]
                landmark_pose.orientation.y = Q[1]
                landmark_pose.orientation.z = Q[2]
                landmark_pose.orientation.w = Q[3]
                pa.poses.append(landmark_pose)
                # markers
                marker = Marker()
                marker.header.stamp = timestamp
                marker.header.frame_id = self.target_frame
                marker.ns = "landmark_kf"
                marker.id = l
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = self.X[l, 0]
                marker.pose.position.y = self.X[l + 1, 0]
                marker.pose.position.z = -0.1
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 1
                marker.pose.orientation.w = 0
                marker.scale.x = 3 * sqrt(abs(self.P[l, l]))
                marker.scale.y = 3 * sqrt(abs(self.P[l + 1, l + 1]))
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.lifetime.secs = 3.0
                ma.markers.append(marker)
                marker = Marker()
                marker.header.stamp = timestamp
                marker.header.frame_id = self.target_frame
                marker.ns = "landmark_kf"
                marker.id = 1000 + l
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = self.X[l + 0, 0]
                marker.pose.position.y = self.X[l + 1, 0]
                marker.pose.position.z = 1.0
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 1
                marker.pose.orientation.w = 0
                marker.text = str(i)
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.lifetime.secs = 3.0
                ma.markers.append(marker)
        self.lock.release()
        self.marker_pub.publish(ma)
        self.marker_pub_pose.publish(pa)


if __name__ == "__main__":
    demo = BubbleSLAM()
    demo.run()
