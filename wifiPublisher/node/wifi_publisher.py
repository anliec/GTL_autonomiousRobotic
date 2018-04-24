#!/usr/bin/env python
import roslib

roslib.load_manifest('wifiPublisher')
import rospy
import os
import re
from wifiPublisher.msg import wifi


class DataNode():
    def __init__(self):
        pub = rospy.Publisher('wifi_data', wifi)

        r = rospy.Rate(rospy.get_param('~rate', 1))
        while not rospy.is_shutdown():
            os.system("iwlist wlan0 scanning >> datatemp.txt")

            wifi_raw = open("datatemp.txt").read()
            os.remove("datatemp.txt")

            addresses = re.findall("Address: ([0-9A-F:]{17})", wifi_raw)
            signals = re.findall("Signal level=.*?([0-9]+)", wifi_raw)

            if len(addresses) != len(signals):
                print("addresses length different to the one of signals")
                continue

            for a, s in zip(addresses, signals):
                w = wifi()
                w.MAC = a
                w.dB = int(s)
                pub.publish(w)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('wifi_publisher')
    try:
        node = DataNode()
    except rospy.ROSInterruptException:
        pass
