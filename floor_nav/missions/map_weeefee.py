#!/usr/bin/python
# ROS specific imports
import roslib
roslib.load_manifest('floor_nav')
import rospy
import tf
from math import *
from task_manager_lib.TaskClient import *
from std_msgs.msg import Empty, Bool
from geometry_msgs.msg import PoseStamped

rospy.init_node('task_client')
server_node = rospy.get_param("~server", "/task_server")
default_period = rospy.get_param("~period", 0.05)
tc = TaskClient(server_node, default_period)
rospy.loginfo("Mission connected to server: " + server_node)

pose_pub = rospy.Publisher("/planner/explore", Empty, queue_size=1)
goto_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
en_pub = rospy.Publisher("/enable_avoidance", Bool, queue_size=1)
def finished_callback(msg):
    global finished
    finished = True
finished_sub = rospy.Subscriber("/explore_finished", Bool, finished_callback)
def home_finished_callback(msg):
    global finished_home
    finished_home = True
finished_home_sub = rospy.Subscriber("/homing_finished", Bool, home_finished_callback)

tc.WaitForAuto()
finished = False
finished_home = False
try:
    en_pub.publish(Bool(False))
    # marche arriere, BANZAAAAI !!!!
    tc.Undock(goal_x=-1.5, k_v=-1)
    tc.SetHeading(target=pi/2)
    tc.Wait(duration=0.5)
    tc.SetHeading(target=pi)
    tc.Wait(duration=0.5)
    tc.SetHeading(target=3*pi/2)
    tc.Wait(duration=0.5)
    tc.SetHeading(target=0)
    tc.Wait(duration=0.5)
    tc.SetHeading(target=pi)
    # timer = tc.Wait(duration=60.0, foreground=False)
    # tc.addCondition(ConditionIsCompleted("Timer condition", tc, timer))
    try:
        while not finished:
            en_pub.publish(Bool(True))
            pose_pub.publish(Empty())
            tc.Wait(duration=5)
    except TaskConditionException, e:
        pass

    # en_pub.publish(Bool(False))
    # tc.Wait(duration=5)
    # tc.SetHeading(target=pi)
    # tc.Wait(duration=5)
    # tc.SetHeading(target=0)
    # tc.Wait(duration=5)
    en_pub.publish(Bool(True))

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "/map"
    pose.pose.position.x = -2
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    timer = tc.Wait(duration=45.0, foreground=False)
    tc.addCondition(ConditionIsCompleted("Timer condition", tc, timer))
    try:
        while not finished_home:
            en_pub.publish(Bool(True))
            goto_pub.publish(pose)
            tc.Wait(duration=3)
    finally:
        en_pub.publish(Bool(False))
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
