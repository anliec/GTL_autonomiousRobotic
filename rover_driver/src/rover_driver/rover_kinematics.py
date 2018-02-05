#!/usr/bin/env python
import roslib;

roslib.load_manifest('rover_driver')
import rospy
from geometry_msgs.msg import Twist
import numpy
from numpy.linalg import pinv
from math import atan2, hypot, pi, cos, sin, fmod, fabs

prefix = ["FL", "FR", "CL", "CR", "RL", "RR"]


class RoverMotors:
    def __init__(self):
        self.steering = {}
        self.drive = {}
        for k in prefix:
            self.steering[k] = 0.0
            self.drive[k] = 0.0

    def copy(self, value):
        for k in prefix:
            self.steering[k] = value.steering[k]
            self.drive[k] = value.drive[k]


class DriveConfiguration:
    def __init__(self, radius, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius


class RoverKinematics:
    def __init__(self):
        self.num_of_wheels = None
        self.X = numpy.asmatrix(numpy.zeros((3, 1)))
        self.motor_state = RoverMotors()
        self.first_run = True
        self.odo_dict = {}

    def twist_to_motors(self, twist, drive_cfg, skidsteer=False):
        motors = RoverMotors()
        if skidsteer:
            for k, w in drive_cfg.items():
                # Insert here the steering and velocity of 
                # each wheel in skid-steer mode
                ground_speed_x = (w.y * twist.angular.z) + twist.linear.x
                motors.steering[k] = 0
                motors.drive[k] = ground_speed_x / w.radius * 2 * pi
        else:
            for k, w in drive_cfg.items():
                # Insert here the steering and velocity of 
                # each wheel in rolling-without-slipping mode
                ground_speed_x = (w.y * twist.angular.z) + twist.linear.x
                ground_speed_y = (w.x * twist.angular.z) + twist.linear.y
                angle = -atan2(ground_speed_y, ground_speed_x)
                speed = hypot(ground_speed_x, ground_speed_y)
                if fabs(fmod(angle - self.motor_state.steering[k], 2 * pi)) > pi / 2.0:
                    speed = -speed
                    angle = fmod(angle + pi, 2 * pi)
                motors.steering[k] = angle
                motors.drive[k] = speed / w.radius * 2 * pi
        return motors

    def integrate_odometry(self, motor_state, drive_cfg):
        # The first time, we need to initialise the state
        if self.first_run:
            self.motor_state.copy(motor_state)
            self.num_of_wheels = len(drive_cfg.keys())
            for k, w in drive_cfg.items():
                self.compute_pseudoinverse(k, w)
            self.first_run = False
            return self.X
        # Insert here your odometry code
        Xvotes = numpy.zeros((3, self.num_of_wheels))
        for i in range(self.num_of_wheels):
            key = drive_cfg.keys()[i]
            motor_i = [
                motor_state.drive[key] * numpy.math.cos(motor_state.steering[key]),
                motor_state.drive[key] * numpy.math.sin(motor_state.steering[key])
            ]
            Xvotes[:, i] = numpy.matmul(self.odo_dict[key], motor_i)
        print(self.X[:, 0])
        print(numpy.transpose(Xvotes.mean(1)))
        print(numpy.add(self.X[:, 0], Xvotes.mean(1))[0, :])
        self.X[:, 0] = numpy.transpose(numpy.add(self.X[:, 0], Xvotes.mean(1))[0, :])
        self.motor_state.copy(motor_state)
        return self.X

    def compute_pseudoinverse(self, wheel_key, wheel_config):
        self.odo_dict[wheel_key] = pinv([[1, 0, -wheel_config.y], [0, 1, wheel_config.x]], 0)
