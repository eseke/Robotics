#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import csv

def talker():
    with open('weather_data_nyc_centralpark_2016.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',',)
        pub_dat = rospy.Publisher('datum', String, queue_size = 1);
        pub_temp = rospy.Publisher('temperatureF', Float32MultiArray, queue_size = 1);
        rospy.init_node('merenja',anonymous=False)
        r = rospy.Rate(10)
        next(csv_reader, None)
        for row in csv_reader:
            #rospy.loginfo(str)
            pub_dat.publish(row[0])
            #for i in range(1,4):
            data_to_send = Float32MultiArray()
            data_to_send.data = np.array(row[1:4]).astype(np.float)
            pub_temp.publish(data_to_send)
            r.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass