#!/usr/bin/env python
import roslib

roslib.load_manifest('wifi_publisher')
import rospy
import os
import re
from wifi_publisher.msg import wifi


class DataNode:
    def __init__(self):
        pub = rospy.Publisher('wifi_data', wifi, queue_size=10)

        r = rospy.Rate(rospy.get_param('~rate', 1))
        while not rospy.is_shutdown():
            os.system("iwlist wlan5 scanning >> datatemp.txt")

            wifi_raw = open("datatemp.txt").read()
            os.remove("datatemp.txt")

            essids = re.findall("ESSID:\"(.*)\"", wifi_raw)
            addresses = re.findall("Address: ([0-9A-F:]{17})", wifi_raw)
            signals = re.findall("Signal level=.*?([0-9]+)", wifi_raw)

            if len(addresses) != len(signals) and len(addresses) != len(essids):
                print("addresses length different to the one of signals or eSSID")
                continue

            for a, s, e in zip(addresses, signals, essids):
                w = wifi()
                w.MAC = a
                w.dB = int(s)
                w.ssid = e
                pub.publish(w)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('wifi_publisher')
    try:
        node = DataNode()
    except rospy.ROSInterruptException:
        pass
