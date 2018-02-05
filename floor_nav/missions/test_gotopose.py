#!/usr/bin/python
# ROS specific imports
import roslib

roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server", "/task_server")
default_period = rospy.get_param("~period", 0.05)
tc = TaskClient(server_node, default_period)
rospy.loginfo("Mission connected to server: " + server_node)

smart = True
vel = 0.5
ang_vel = 3

tc.WaitForAuto()
try:
    tc.GoToPose(goal_x=2, goal_y=-0.5, goal_theta=0.5 * 3.1415, max_velocity=vel, max_angular_velocity=ang_vel,
                smart=smart, sigma2=5)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=0.8, goal_y=2, goal_theta=3.1415, max_velocity=vel, max_angular_velocity=ang_vel, smart=smart,
                sigma2=5)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=-2, goal_y=2, goal_theta=-0.5 * 3.1415, max_velocity=vel, max_angular_velocity=ang_vel,
                smart=smart, sigma2=5)
    tc.Wait(duration=1.0)
    tc.GoToPose(goal_x=0, goal_y=0, goal_theta=0, max_velocity=vel, max_angular_velocity=ang_vel, smart=smart, sigma2=5)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")
