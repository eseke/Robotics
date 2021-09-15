#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

Oscilacije = 0

def callback(data):
    global Oscilacije;
    if(data.data):
        Oscilacije += 1;
        print(Oscilacije)

def listener():
    rospy.init_node('akcija', anonymous=False)
    rospy.Subscriber('Oscilacija', Bool, callback)
    rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass