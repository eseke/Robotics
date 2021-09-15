#!/usr/bin/env python3

import numpy as np
import threading
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from laser_line_extraction.msg import LineSegmentList
from termios import tcflush, TCIFLUSH
from tf.transformations import euler_from_quaternion
from hello_world.srv import kontrola, kontrolaResponse
import sys
import os
import queue
import math as m
import math
import scipy.linalg
import time

poruka = """Sa Kalmanom s, bez Kalmana b, za izlaz i."""

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

def transitionFunction(X_prev, U, b):
	d_sl,d_sr = U
	_,_,theta_prev = X_prev
	k1 = (d_sl+d_sr)/2
	k2 = (d_sr-d_sl)/2/b
	
	X_hat = X_prev + np.array([k1*m.cos(theta_prev+k2),k1*m.sin(theta_prev+k2),k2*2]).reshape(3,1)
	[_,_,theta] = X_hat
	Fx = np.array([[1, 0, -k1*m.sin(theta+k2)],[0,1,k1*m.cos(theta+k2)],[0, 0,1]])
	Fu11 = m.cos(theta+k2)/2+m.sin(theta+k2)*k1/2/b
	Fu12 = m.cos(theta+k2)/2-m.sin(theta+k2)*k1/2/b
	Fu21 = m.sin(theta+k2)/2-m.cos(theta+k2)*k1/2/b
	Fu22 = m.sin(theta+k2)/2+m.cos(theta+k2)*k1/2/b
	Fu31 = -1/b
	Fu32 = 1/b
	Fu = np.array([[Fu11, Fu12],[Fu21, Fu22],[Fu31, Fu32]])
	return X_hat, Fx, Fu
def getApriori(X_prev,P_prev, U, b):
	X_hat, Fx, Fu = transitionFunction(X_prev, U, b)
	d_sl,d_sr = U
	k = 0.01
	Q = np.array([[k*abs(d_sr), 0],[0, k*abs(d_sl)]])
	P_hat = np.dot(np.dot(Fx,P_prev),Fx.T) + np.dot(np.dot(Fu,Q),Fu.T)
	return X_hat, P_hat
	
def measurementFunction(X_hat, M_i):
	x_hat,y_hat,theta_hat = X_hat
	w_ro, w_alpha = M_i
	z_hat = np.array([w_alpha - theta_hat,w_ro-(x_hat*m.cos(w_alpha)+y_hat*m.sin(w_alpha))])
	H_hat = np.array([[0,0,-1],[-m.cos(w_alpha),-m.sin(w_alpha),0]])
	return z_hat, H_hat

def associateMeasurement(X_hat, P_hat, Z, R, M, g):
	H_hat = np.empty((M.shape[1],2,3))
	V = np.empty((M.shape[1],Z.shape[1],2))
	SIGMA_IN = np.empty((M.shape[1],Z.shape[1],2,2))
	for i in range(M.shape[1]):
		z_hat, H_hat_i = measurementFunction(X_hat,M[:,i])
		H_hat[i,:,:] = H_hat_i
		for j in range(Z.shape[1]):
			z = Z[:,j].reshape(2,1)
			V[i,j,:] = np.ravel(z-z_hat)
			SIGMA_IN[i,j,:,:] = np.dot(np.dot(H_hat_i,P_hat),H_hat_i.T) + R[j,:,:]
	Vout=[]
	Hout=[]
	Rout=[]
	for i in range(V.shape[0]):
		for j in range(V.shape[1]):
			dist = np.dot(np.dot(V[i,j,:].T,np.linalg.inv(SIGMA_IN[i,j,:,:])),V[i,j,:])
			if(dist < g**2): 
				Vout.append(V[i,j,:])
				Rout.append(R[j,:,:])
				Hout.append(H_hat[i,:,:])
	Vout = np.array(Vout,dtype=float)
	Rout = np.array(Rout,dtype=float)
	Hout = np.array(Hout,dtype=float)
	return Vout,Hout,Rout 

def filterStep(X_hat, P_hat, V, H_hat, R):
	P_hat = P_hat.astype(float)
	R = scipy.linalg.block_diag(*R)
	H = np.reshape(H_hat,(-1,3))
	V = np.reshape(V,(-1,1))
	K = np.dot(np.dot(P_hat,H.T),np.linalg.inv(np.dot(np.dot(H,P_hat),H.T) + R))
	X = X_hat + np.dot(K,V)
	P = np.dot((np.identity(3)-np.dot(K,H)),P_hat)
	return X,P

def loadMap():
	Map = np.genfromtxt('/home/ros/Workspaces/getting_started/src/hello_world/src/map.csv', delimiter=',')
	return np.transpose(Map)

M = loadMap()
b = 0.16
g = 10
X = np.array([[1],[0],[0]])
P = np.array([[1e-6,0,0],[0,1e-6,0],[0,0,1e-6]])
U = np.array([0,0])
U_last = np.array([0,0])
def jointState_callback(data):
	global U;
	U = np.array(data.position);

def curr_pos_lidar_kalman(data):
	k_ro = 0.25;
	k_b =-0.2;
	k_a = 0.5;
	global M,b,g;
	global X,P;
	global U,U_last;
	if(U_last[0]==0):
		U_last = U;
		return
	deltaU = U-U_last;
	deltaU /=10;
	U_last = U;
	#print(deltaU)
	X_prev = X
	P_prev = P
	
	lines = data.line_segments	
	Z_temp = []
	R_temp = []
	#print(lines)
	for i,line in enumerate(lines):
		#print(line.angle,line.radius)
		Z_temp.append(np.array([line.angle,line.radius]))
		covariance = np.asarray(line.covariance)
		R_temp.append(covariance.reshape((2,2)))

	if(len(Z_temp) == 0):
		sys.exit("### The robot is stuck ###")
	Z = np.array(Z_temp).T # Z.shape = 2xk
	R = np.array(R_temp) # R.shape = kx2x2


	X_hat,P_hat = getApriori(X_prev,P_prev, deltaU, b)
	V,H_hat,R_orig = associateMeasurement(X_hat, P_hat, Z, R, M, g)
	X,P = filterStep(X_hat, P_hat, V, H_hat, R_orig)
	#print(X)
	global x_curr;
	global y_curr;
	global angle_curr;
	global P_curr
	x_curr = X[0];
	y_curr = X[1];
	angle_curr = X[2];
	P_curr = P


def cur_pos_odom(msg):
	global x_curr_odom
	global y_curr_odom
	global angle_curr_odom
	global cov_odom;
	cov_odom=np.array([[msg.pose.covariance[0],msg.pose.covariance[1],msg.pose.covariance[5]],
	[msg.pose.covariance[6],msg.pose.covariance[7],msg.pose.covariance[11]],
	[msg.pose.covariance[30],msg.pose.covariance[31],msg.pose.covariance[35]]]);
	x_curr_odom = msg.pose.pose.position.x
	y_curr_odom = msg.pose.pose.position.y
	#print(msg)
	rot_q = msg.pose.pose.orientation
	(roll,pitch,angle_curr_odom) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
	#print(roll,pitch,angle_curr)

v_max = 0.1#0.22
w_max = 2#2.84
x_curr_odom = 0
y_curr_odom = 0
angle_curr_odom = 0
P_curr = np.zeros((3,3))
cov_odom = np.zeros((3,3))

def automatski_rezim():
	#x = 1
	#y = 0
	#angle = 0
	global x_curr;
	global y_curr;
	global angle_curr;
	global P_curr;

	global x_curr_odom
	global y_curr_odom
	global angle_curr_odom
	global cov_odom;
	#print(x_curr,y_curr,angle_curr)
	global pub;
	k_ro = 0.25;
	k_b =-0.2;
	k_a = 0.5;
	unos = np.array([[2,0,0],[2,-1,0],[1,-1,0],[1,0,0]]);
	for i in range(0,4):
		#print(poruka_automatski);
		#tmp = input().split(' ');
		#x = float(tmp[0])
		#y = float(tmp[1])
		#angle = float(tmp[2])
		x = unos[i][0];
		y = unos[i][1]
		angle = unos[i][2]
		a = (-angle_curr+math.atan2(y-y_curr,x-x_curr))%(6.28);
		if(a>math.pi):
			a -= 2*math.pi;
			
		if(a>-math.pi/2 and a<math.pi/2):
			while True:
				#time.sleep(0.05)
				os.system('clear')
				print("Kalman  ",float(x_curr),float(y_curr),float(angle_curr))
				print(P_curr)
				print("Odometar",x_curr_odom,y_curr_odom,angle_curr_odom)
				print(cov_odom)
				a = (-angle_curr+math.atan2(y-y_curr,x-x_curr))%(6.28);
				if(a>math.pi):
					a -= 2*math.pi;
				twist = Twist()
				twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
				ro = math.sqrt(pow(x_curr-x,2)+pow(y_curr-y,2));
				if(ro>0.01):
					v = k_ro*ro
				else:
					v = 0;

				w = 0;
				b = -math.atan2(y-y_curr,x-x_curr)+angle;
				if(b>math.pi):
					b -= 2*math.pi;
				if(b<-math.pi):
					b += 2*math.pi;
				if(ro>0.01):
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
				if(ro<=0.01):
					break;
		else:
			while True:
				os.system('clear')
				print("Kalman  ",float(x_curr),float(y_curr),float(angle_curr))
				print(P_curr)
				print("Odometar",x_curr_odom,y_curr_odom,angle_curr_odom)
				print(cov_odom)
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
				if(ro>0.01):
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
				if(ro>0.01):
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

				if(ro<=0.01):
					break;

if __name__=="__main__":
	rospy.init_node('main')
	cont = rospy.ServiceProxy('kontrola',kontrola)
	x = threading.Thread(target=automatski_rezim)
	sub_odo = rospy.Subscriber("/odom",Odometry,cur_pos_odom);
	print("Sa Kalmanom")
	rospy.Subscriber("line_segments",LineSegmentList, curr_pos_lidar_kalman)
	rospy.Subscriber("joint_states", JointState,jointState_callback)
	x.start()
	x.join()
		
		
			
		