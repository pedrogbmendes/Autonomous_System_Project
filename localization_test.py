

import numpy as np
from numpy import linalg as LA
from transforms3d import quaternions
import roslib
import sys
import rospy
import time
from sensor_msgs.msg import Image, Imu
import datetime
import math
import matplotlib.pyplot as plt
from PIL import Image
from matplotlib import colors
import matplotlib.animation as anim
import pylab as pl



#-----------------------------------------------------------------------------I
#
#   Global constants
#
#-----------------------------------------------------------------------------
I = np.identity(6)
cov_x = 1
cov_y = 1
cov_teta = 1
matrix_R = np.array([[0,0,0,0,0,0],[0,cov_x,0,0,0,0],[0,0,0,0,0,0],[0,0,0,cov_y,0,0],[0,0,0,0,0,0],[0,0,0,cov_teta,0,0]])

#Camera coordenate frames vectors
v_x = np.array([1,0,0])
v_y = np.array([0,1,0])
v_z = np.array([0,0,1])

#map info
resolution = 0.05 #meters/pixel

size_vector = 2
xs = 1
ys = 2
xp = 1
yp = 3

#-----------------------------------------------------------------------------
#
#   EKF
#
#-----------------------------------------------------------------------------

class EKF_localization:

    def __init__(self):

        #previous time
        self.prev_time = 0
        #actual time
        self.act_time = 0
        #difference of time
        self.delta_time = 0
        #previous state
        self.prev_state = np.array([[0],[0],[0],[0],[0],[0]])
        #actual state
        self.act_state = np.array([[1.1],[0],[1],[0],[np.pi/2],[0]])
        #predicted state
        self.pred_state = np.array([[0],[0],[0],[0],[0],[0]])
        #motion model
        self.matrix_A = np.array([[1,self.delta_time,0,0,0,0],[0,1,0,0,0,0],[0,0,1,self.delta_time,0,0],[0,0,0,1,0,0],[0,0,0,0,1,self.delta_time],[0,0,0,0,0,1]])
        #covariance of instance t-1
        self.prev_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #covariance of instance t
        self.act_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #predicted covariance
        self.pred_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #Jacobian matrix
        self.matrix_H = np.zeros((1, 6))
        #covariance of noise observation matrix
        self.matrix_Q = np.identity(2)
        #rotation matrix
        self.rotation_matrix = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        #frame
        self.frame = np.zeros((480,640))#size of the image taken by the camera
        #line fo frame
        self.line_z = 0
        #observation modes
        self.h = 0
        #map
        #self.map = self.openImage()


    def openImage(self):

        np.set_printoptions(threshold=np.inf) #see all matrix

        img = Image.open("map.pgm")
        area = (950, 950, 1600, 1130) #left, top, right, bottom
        cropped_img = img.crop(area)
        img_matrix = np.array(cropped_img)

        #unknown positions of map
        BW_img_des = img_matrix == 205
        BW_img_des = BW_img_des * - 1
        #occupied positions of the map
        BW_img_oc = img_matrix == 254
        BW_img_oc = BW_img_oc* 1  #0 and 1 instead of False and True

        return BW_img_des+BW_img_oc


    def robot_localization(self):

        self.predition_step()
        self.update_step()
	#print "real position:  (4, 1)"
	#print "prediction:"
	
	#print self.act_cov
	
	
	time.sleep(1)
        
	'''
	#node of drone to Subscribe IMU data
	self.subsIMU = rospy.Subscriber('/imu/data_raw',Imu,self.sub_pub_calRot)
	#and to to Subscribe camera data
	self.image_sub = rospy.Subscriber('/camera/depth/image',Image,self.save_image)
	rate = rospy.Rate(10)


	rospy.spin()
	'''


    def predition_step(self):

        self.prev_time = self.act_time
	print("a=%s" % self.act_time)
        self.act_time +=1
        self.delta_time = self.act_time - self.prev_time

	self.prev_state = self.act_state
	self.prev_cov = self.act_cov

        self.matrix_A[0][1] = self.delta_time
        self.matrix_A[2][3] = self.delta_time
        self.matrix_A[4][5] = self.delta_time

        self.pred_state = self.matrix_A.dot(self.prev_state)
        self.pred_cov = ((self.matrix_A.dot(self.prev_cov)).dot(self.matrix_A.transpose())) + matrix_R
	

    def update_step(self):

        #Kalman gain
        self.line_z = np.array([[1],[np.pi/2]])
	global ys
	ys = self.pred_state[2]
	global xs
	xs = self.pred_state[0]
	#if 2 > self.pred_state[2]:		
	#	ys = 2 - self.pred_state[2]
	#else:
	#	ys =  self.pred_state[2] - 2
	
	self.h = np.array([[math.sqrt(np.power(xs-xp, 2)+np.power(ys-yp, 2))],self.pred_state[4]])

        self.jacobian()
        self.matrix_Q = np.identity(size_vector)
        k = (self.pred_cov.dot(self.matrix_H.transpose())).dot(LA.inv((self.matrix_H.dot(self.pred_cov)).dot(self.matrix_H.transpose()) + self.matrix_Q))
	
        self.act_state = self.pred_state + k.dot(self.line_z - self.h)
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov)

	print self.pred_state
     
       # fig = plt.figure()
        #x = np.array(range(-10,10))
        #y = self.pred_state[4]*(xs-1) - 1
	#print x.shape
        #print y
        endy = ys +(3* math.sin(self.pred_state[4]))
        endx = xs +(3* math.cos(self.pred_state[4]))
	plt.ion()
        fig=plt.figure(1)
        pl.figure(1)
        ax = fig.add_subplot(111)
	ax.plot([xs, endx], [ys, endy])
        line1, = ax.plot(xs, ys, 'ro') 
	line1, = ax.plot(xp, yp, 'go') 
	line1, = ax.plot(xp, yp, 'b-')
  
        s = -2 * math.log(1 - 0.95)
	w, v=LA.eig(self.pred_cov*s)

        t = np.linspace(0, 2*math.pi, 100)
        plt.plot( xs+w[1]*np.cos(t) , ys+w[2]*np.sin(t) )
        plt.grid(color='lightgray',linestyle='--')

        #pl.plot(xs,y)
        plt.axis([-15, 15, -15, 15])
        #line1.set_ydata(np.sin(0.5 * x + phase))
        fig.canvas.draw()
	#plt.axis([0, 5, 0, 5])
        #a = anim.FuncAnimation(fig, update, frames=10, repeat=False)
        #plt.show()
        plt.gcf().clear()

    def jacobian(self):

        self.matrix_H= np.zeros((size_vector,6))

        #for it in range(0, size_vector-1):
				
        #	self.matrix_H[it,:] = self.partial_jacobian1(xs[it], ys[it], xp[it], yp[it])
	self.matrix_H[0,:] = self.partial_jacobian1(xs, ys, xp, yp)
	self.matrix_H[size_vector-1, :] = self.partial_jacobian2()
    	

    def partial_jacobian1(self, xs1, ys1, xp1, yp1):

        d_h = np.zeros(6)

        d_h[0] = (xs1-xp1)/math.sqrt(np.power(xs1-xp1, 2)+np.power(ys1-yp1, 2))
        d_h[1] = 0
        d_h[2] = (ys1-yp1)/math.sqrt(np.power(xs1-xp1, 2)+np.power(ys1-yp1, 2))
        d_h[3] = 0
        d_h[4] = 0
        d_h[5] = 0
	
    	return d_h

    def partial_jacobian2(self):

        d_h = np.zeros(6)

        d_h[0] = 0
        d_h[1] = 0
        d_h[2] = 0
        d_h[3] = 0
	d_h[4] = 1
	d_h[5] = 0

    	return d_h
#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    #try:

        prog = EKF_localization()

        #rospy.init_node('drone', anonymous=True)
	while(1):
        	prog.robot_localization()



    #except rospy.RosInterruptException:
        pass
