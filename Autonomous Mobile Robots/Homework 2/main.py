#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from termios import tcflush, TCIFLUSH
from tf.transformations import euler_from_quaternion
from hello_world.srv import kontrola, kontrolaResponse
import sys
import os
import queue
import math
from pynput import keyboard


poruka = """Za rucni rezim pritisnite r, za automatski a, za izlaz i."""

poruka_rucni = """Za izlaz: i
Za napred: w
Za nazad: s
Za okretanje ulevo: a
Za okretanje udesno: d"""

poruka_automatski = """Unesite zeljene kordinate i orjentaciju."""

curr = ""
status = 'm';
last = '';
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1);
pokrenuto = False;
x_curr = 0.0
y_curr = 0.0
angle_curr = 0.0

def cur_pos(msg):
    global x_curr
    global y_curr
    global angle_curr

    x_curr = msg.pose.pose.position.x
    y_curr = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll,pitch,angle_curr) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
    #print(roll,pitch,angle_curr)


def on_press(key):
    try:
        global status;
        global curr;
        if(status=='r'):
            if(curr=="" and (key.char=='w' or key.char=='s' or key.char=='a' or key.char=='d' or key.char=='i')):
                curr = key.char;
    except AttributeError:
        
        pass
    print ("            \033[A")

def on_release(key):
    try:
        global status;
        global curr;
        if(status=="r"):
            if(key.char==curr):
                curr= "";
    except AttributeError:
        pass

v_max = 0.22
w_max = 2.84

v_man = 0;
w_man = 0
def rucni_rezim():
    global v_man;
    global w_man;
    global pokrenuto;
    global status;
    status = 'r';
    global curr;
    global last;
    global pub;
    print(poruka_rucni);
    if(not pokrenuto):
        listener.start();
        pokrenuto = True;
    r = rospy.Rate(10)
    while True:
        if(curr!=last):
            last = curr;
            if(curr=='i'):
                break;
            elif(curr==''):
                #twist = Twist();
                #twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
                #twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
                #pub.publish(twist)
                v_man = 0;
                w_man = 0;
            elif(curr in ('w','a','s','d')):
                velocity = {'w':[0.22,0],'a':[0,2.84],'s':[-0.22,0],'d':[0,-2.84]}
                #twist = Twist();
                #twist.linear.x = velocity[curr][0]; twist.linear.y = 0.0; twist.linear.z = 0.0;
                #twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = velocity[curr][1];
                v_man = velocity[curr][0];
                w_man = velocity[curr][1];
                #pub.publish(twist)
        r.sleep()
        cont(v_man,w_man);

def automatski_rezim():
    global x_curr;
    global y_curr;
    global angle_curr;
    global pub;
    k_ro = 0.25;
    k_b =-0.2;
    k_a = 0.5;
    print(poruka_automatski);
    tmp = input().split(' ');
    x = float(tmp[0])
    y = float(tmp[1])
    angle = float(tmp[2])
    print(x,y,angle)
    sub_odo = rospy.Subscriber("/odom",Odometry,cur_pos);
    a = (-angle_curr+math.atan2(y-y_curr,x-x_curr))%(6.28);
    if(a>math.pi):
        a -= 2*math.pi;
        
    if(a>-math.pi/2 and a<math.pi/2):
        while True:
            a = (-angle_curr+math.atan2(y-y_curr,x-x_curr))%(6.28);
            if(a>math.pi):
                a -= 2*math.pi;
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
            ro = math.sqrt(pow(x_curr-x,2)+pow(y_curr-y,2));
            if(ro>0.001):
                v = k_ro*ro
            else:
                v = 0;

            w = 0;
            b = -math.atan2(y-y_curr,x-x_curr)+angle;
            if(b>math.pi):
                b -= 2*math.pi;
            if(b<-math.pi):
                b += 2*math.pi;
            if(ro>0.001):
                w  = w+k_a*a + k_b*b;
            if(ro>0.2):
                if(abs(v/v_max)>abs(w/w_max)):
                    w *= abs(v_max/v);
                    v= v_max;
                else:
                    v *= abs(w_max/w);
                    w= w_max;

            resp1 = cont(v,w)
            #print(v,w,1)
            #twist.linear.x = v;
            #twist.angular.z = w;
            #pub.publish(twist);
            if(ro<=0.001):
                break;
    else:
        while True:
            a = (-angle_curr+math.atan2(y-y_curr,x-x_curr))%(6.28);
            if(a>math.pi):
                a -= 2*math.pi;
            a -=math.pi;
            if(a<-math.pi):
                a += 2*math.pi;
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
            ro = math.sqrt(pow(x_curr-x,2)+pow(y_curr-y,2));
            if(ro>0.001):
                v = -k_ro*ro;
            else:
                v = 0;
            w = 0;
            b = 0;
            ugao = (math.atan2(y-y_curr,x-x_curr)+math.pi);
            if(ugao>math.pi):
                ugao -= 2*math.pi;
            ugao1 = angle-ugao;
            if(ugao1>math.pi):
                ugao1 -= 2*math.pi;
            if(ugao1<-math.pi):
                ugao1 += 2*math.pi;
            if(ro>0.001):
                w  = w+k_a*a + k_b*(ugao1);


            if(ro>0.2):
                if(abs(v/v_max)>abs(w/w_max)):
                    w *= abs(v_max/v);
                    v= -v_max;
                else:
                    v *= abs(w_max/w);
                    w= w_max;
            
            twist.linear.x = v;
            twist.angular.z = w;
            
            #print(w,ugao1,2)
            resp1 = cont(v,w)

            if(ro<=0.001):
                break;
    sub_odo.unregister();

if __name__=="__main__":
    rospy.init_node('main')
    cont = rospy.ServiceProxy('kontrola',kontrola)
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    while(1):
        status = 'm';
        print(poruka);
        print ("            \033[A")
        tcflush(sys.stdin, TCIFLUSH)
        odabir = sys.stdin.read(2)
        if odabir=="r\n":
            print("Rucni rezim")
            rucni_rezim();
        elif odabir=='a\n':
            status = 'm'
            print("Automatski rezim")
            automatski_rezim();
        elif odabir=='i\n':
            print("Pozdrav")
            break;
        else:
            print("Los unos")
            
        