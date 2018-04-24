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


tc.WaitForAuto()
try:
    # marche arriere, BANZAAAAI !!!!
    tc.TaskUndock(goal_x=-1.5, k_v=-2)
    tc.SetHeading(target=pi)
    tc.Wait(duration=3.0)
    gtb = tc.GoToBase(foreground=True)
    # w4roi = tc.WaitForFace(foreground=False)
    # tc.addCondition(ConditionIsCompleted("Face fund condition", tc, w4roi))
    # try:
    #     tc.Wander(max_linear_speed=0.5)
    # except TaskConditionException, e:
    #     rospy.loginfo("Path following interrupted on condition: %s"
    #                   "or ".join([str(c) for c in e.conditions]))
    #     tc.TaskStareAtFace(angle_threshold=0.01)
    #     tc.Wait(duration=3.0)
    # timer = tc.Wait(duration=5.0, foreground=False)
    # tc.addCondition(ConditionIsCompleted("Timer condition", tc, timer))
    # try:
    #     tc.Wander(max_linear_speed=0.5)
    # except TaskConditionException, e:
    #     pass

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")
