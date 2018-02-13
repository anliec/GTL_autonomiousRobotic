#!/usr/bin/env python
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('path_exporter')

    path = Path()
    path.header.frame_id = 'map'
    path.poses = []

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10)

    pub = rospy.Publisher('path', Path, queue_size=1)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'Hokuyo', rospy.Time(), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        current_pose = PoseStamped()
        current_pose.pose.position = trans.transform.translation
        current_pose.pose.orientation = trans.transform.rotation

        path.poses.append(current_pose)
        pub.publish(path)

        rate.sleep()
