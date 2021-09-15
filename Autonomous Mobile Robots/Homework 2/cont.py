#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from hello_world.srv import kontrola, kontrolaResponse

v_last = 0
w_last = 0

v_step = 0.02
w_step = 0.9

v_max = 0.22
w_max = 2.84

def response_callback(req):
    twist = Twist();
    
    global v_last;
    global w_last;
    global v_step;
    global w_step;
    
    if req.v>v_last:
        if req.v>v_last+v_step:
            v_last += v_step;
        else:
            v_last = req.v;
    elif req.v<v_last:
        if req.v<v_last-v_step:
            v_last -= v_step;
        else:
            v_last = req.v;

    if req.w>w_last:
        if req.w>w_last+w_step:
            w_last += w_step;
        else:
            w_last = req.w;
    elif req.w<w_last:
        if req.w<w_last-w_step:
            w_last -= w_step;
        else:
            w_last = req.w;
    
    if(v_last>v_max):
        v_last = v_max;
    if(w_last>w_max):
        w_last = w_max;

    twist.linear.x = v_last; twist.linear.y = 0.0; twist.linear.z = 0.0;
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = w_last;
    pub.publish(twist);
    #print(v_last/w_last)
    return kontrolaResponse(True)

rospy.init_node('cont')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1);
s = rospy.Service('kontrola',kontrola,response_callback)
rospy.loginfo('Service is ready !!!')
rospy.spin()