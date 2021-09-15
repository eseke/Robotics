#!/usr/bin/env python3

import rospy
import message_filters
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import String

Oscilacije = 0

def callback(data1,data2):
    print(data1.data,data2.data)

def listener():
    rospy.init_node('prikaz', anonymous=False)
    vreme = message_filters.Subscriber('datum', String)
    temp = message_filters.Subscriber('Srednja', Float32)

    ts = message_filters.ApproximateTimeSynchronizer([vreme, temp], 10,0.1,allow_headerless=True)
    ts.registerCallback(callback)
    rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass