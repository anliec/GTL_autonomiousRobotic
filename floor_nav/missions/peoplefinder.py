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
    w4roi = tc.WaitForFace(foreground=False)
    tc.addCondition(ConditionIsCompleted("Face fund condition", tc, w4roi))
    try:
        tc.Wander(max_linear_speed=0.5)
    except TaskConditionException, e:
        rospy.loginfo("Path following interrupted on condition: %s"
                      "or ".join([str(c) for c in e.conditions]))

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()

rospy.loginfo("Mission completed")
