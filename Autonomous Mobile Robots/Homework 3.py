#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time
global odabir
def callback(msg):
    global odabir;
    if not odabir in {'r','i'}:
        return;
    start = time.time()  
    distance = np.asarray(msg.ranges);
    distance[distance==np.inf] = 3.5;
    angle = np.linspace(msg.angle_min,msg.angle_max,len(msg.ranges));

    x1 = (distance * np.cos(angle))
    x2 = np.ones((len(msg.ranges)))
    x = np.stack((x1, x2))

    y = distance * np.sin(angle)
    linije = []

    queue = []
    queue.append([0,len(msg.ranges)-1]);
    tacke = set()
    
    while(len(queue)):
        while(len(queue)):
            curr = queue.pop(0);
            if(curr[0]==curr[1]):
                continue;
            if(odabir=='r'):
                line = np.matmul(np.matmul(np.linalg.inv(np.matmul(x[:,curr[0]:curr[1]+1],np.transpose(x[:,curr[0]:curr[1]+1]))),x[:,curr[0]:curr[1]+1]),y[curr[0]:curr[1]+1])
                treshold = 0.1
            else:
                treshold = 0.15
            maxd = -1;
            maxid = -1;

            for i in range(curr[0],curr[1]+1):
                if(odabir=='r'):
                    currd = abs(line[0]*x[0,i]-1*y[i]+line[1])/math.sqrt(math.pow(line[0],2)+1);
                else:
                    currd = abs((x1[curr[1]]-x1[curr[0]])*(y[curr[0]]-y[i])-(y[curr[1]]-y[curr[0]])*(x1[curr[0]]-x1[i]))/math.sqrt(math.pow(x1[curr[1]]-x1[curr[0]],2)+math.pow(y[curr[1]]-y[curr[0]],2));
                if(currd>maxd):
                    maxd = currd;
                    maxid = i;
            if(maxd>treshold):
                if(maxid==curr[0]):
                    tacke.add(maxid);
                    queue.append([curr[0]+1,curr[1]])
                elif(maxid==curr[1]):
                    tacke.add(maxid);
                    queue.append([curr[0],curr[1]-1])
                else:
                    queue.append([curr[0],maxid])
                    queue.append([maxid,curr[1]])
            else:
                linije.append(curr);
        linije.sort()
        if(linije[0][0]!=0):
            queue.append([0,linije[0][0]])
        for linija in range(1,len(linije)):
            if(linije[linija][0]!=linije[linija-1][1]):
                queue.append([linije[linija-1][1],linije[linija][0]]);
        if(linije[-1][1]!=359):
            queue.append([linije[-1][1],359]);
    linije_fin = []
    ii=1;
    linije_fin.append(linije[0])
    while(ii<len(linije)-1):
        i = linije_fin[-1]
        line = np.matmul(np.matmul(np.linalg.inv(np.matmul(x[:,i[0]:i[1]+1],np.transpose(x[:,i[0]:i[1]+1]))),x[:,i[0]:i[1]+1]),y[i[0]:i[1]+1])
        alpha1 = math.atan(-1/line[0]) # in radians
        ro1 = line[1]/(math.sin(alpha1)-line[0]*math.cos(alpha1))
        i = linije[ii]
        line = np.matmul(np.matmul(np.linalg.inv(np.matmul(x[:,i[0]:i[1]+1],np.transpose(x[:,i[0]:i[1]+1]))),x[:,i[0]:i[1]+1]),y[i[0]:i[1]+1])
        alpha2 = math.atan(-1/line[0]) # in radians
        ro2 = line[1]/(math.sin(alpha2)-line[0]*math.cos(alpha2))
        if(abs(alpha1-alpha2)<0.7 and abs(ro1-ro2)<0.6):
            linije_fin[-1][1] = linije[ii][1];

        else:
            linije_fin.append(linije[ii]);
        ii += 1;


    for i in linije_fin:
        #print(i[0],i[1])
        #x[0,i[0]:i[1]+1] = x[0,i[0]:i[1]+1] -np.mean(x[0,i[0]:i[1]+1] )
        #y[i[0]:i[1]+1] = y[i[0]:i[1]+1]-np.mean(y[i[0]:i[1]+1])
        line = np.matmul(np.matmul(np.linalg.inv(np.matmul(x[:,i[0]:i[1]+1],np.transpose(x[:,i[0]:i[1]+1]))),x[:,i[0]:i[1]+1]),y[i[0]:i[1]+1])
        alpha = math.atan(-1/line[0]) # in radians
        ro = line[1]/(math.sin(alpha)-line[0]*math.cos(alpha))
        print(ro,alpha)
    end = time.time() 
    print("Vreme:",end - start)
    sub.unregister();
poruka = """
Odaberi algoritam:
    r - rekurzivni
    i - iterativni
"""


if __name__=="__main__":

    print(poruka)
    global odabir;
    odabir = input()
    if(odabir in {'r','i'}):
        rospy.init_node('main')
        sub = rospy.Subscriber("scan", LaserScan, callback)
        input()