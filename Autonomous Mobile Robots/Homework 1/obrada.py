#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
pub_diff = rospy.Publisher('Oscilacija', Bool, queue_size = 1);
pub_avg = rospy.Publisher('Srednja', Float32, queue_size = 1);

def callback(data):
    #rospy.loginfo(data.data)
    tempC = (np.array(data.data)-32.0)*5.0/9.0;
    if(tempC[0]-tempC[1]>15):
        pub_diff.publish(True);
    pub_avg.publish(tempC[2])
    #print((np.array(data.data)-32.0)*5.0/9.0)
    #print(np.array(data.data))
    #rospy.loginfo((data.data-32.0)*5.0/9.0)

def listener():
    rospy.init_node('obrada', anonymous=False)
    rospy.Subscriber('temperatureF', Float32MultiArray, callback)
    rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass