#!/usr/bin/env python3

import rospy
from hello_world.srv import Unos_datoteka, Unos_datotekaResponse
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import numpy as np
pub_dat = rospy.Publisher('datum', String, queue_size = 1);
pub_temp = rospy.Publisher('temperatureF', Float32MultiArray, queue_size = 1);

def response_callback(req):
    f = open('/home/ros/Workspaces/getting_started/src/hello_world/src/weather_data_nyc_centralpark_2016.csv','a+')
    MAX = req.max*9.0/5.0+32.0;
    MIN = req.min*9.0/5.0+32.0;
    AVG = req.avg*9.0/5.0+32.0;
    f.write(req.dat+','+str(MAX)+','+str(MIN)+','+str(AVG)+',0,0,0\n')
    pub_dat.publish(req.dat)
    data_to_send = Float32MultiArray()
    data_to_send.data = np.array([MAX,MIN,AVG])
    pub_temp.publish(data_to_send)
    f.close()
    return Unos_datotekaResponse(True)

rospy.init_node('add_value_to_file')
s = rospy.Service('Unos_datoteka',Unos_datoteka,response_callback)
rospy.loginfo('Service is ready !!!')
rospy.spin()