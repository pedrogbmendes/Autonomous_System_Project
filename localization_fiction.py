# coding=utf-8
#-----------------------------------------------------------------------------
#
#                           Autonomous Systems
#
#       Project: Extended Kalman Filter (EKF) localization using a drone with
#   a depth camera and a IMU
#
#       Drone node: Extended Kalman Filter
#
#   Authors:
#       - Pedro Gonçalo Mendes, 81046, pedrogoncalomendes@tecnico.ulisboa.pt
#       - Miguel Matos Malaca, 81702, miguelmmalaca@tecnico.ulisboa.pt
#       - João José Rosa, 84089, joao.c.rosa@tecnico.ulisboa.pt
#       - João Pedro Ferreira, 78101, joao.pedro.ferreira@tecnico.ulisboa.pt
#
#                         1st semestre, 2018/19
#                       Instítuto Superior Técnico
#
#-----------------------------------------------------------------------------


#-----------------------------------------------------------------------------I
#
#   Import libraries
#
#-----------------------------------------------------------------------------

import numpy as np
from numpy import linalg as LA
#from transforms3d import quaternions
import roslib
import sys
import rospy
#from sensor_msgs.msg import Image, Imu, MagnecticField
import time
import math
from PIL import Image
from matplotlib import colors
import matplotlib.animation as anim
import pylab as pl
import matplotlib.pyplot as plt


#-----------------------------------------------------------------------------I
#
#   Global constants
#
#-----------------------------------------------------------------------------
I = np.identity(6)
cov_x = .1
cov_y = .1
cov_teta = .01
matrix_R = np.array([[0,0,0,0,0,0],[0,cov_x,0,0,0,0],[0,0,0,0,0,0],[0,0,0,cov_y,0,0],[0,0,0,0,0,0],[0,0,0,cov_teta,0,0]])

#Camera coordenate frames vectors
v_x = np.array([1,0,0])
v_y = np.array([0,1,0])
v_z = np.array([0,0,1])

#map info
resolution = 0.1 #meters/pixel

#INITIAL CONDITIONS
x_init = 60
vx_init = 0
y_init = 60
vy_init = 0
orientation_init = -np.pi/4
ang_vel_init = 0

np.set_printoptions(threshold=4)
xr = [10]
yr = [-10]
orir = np.array([[-np.pi/4]])
dm = np.array([[0.91], [1.41], [0.91]])

global ct
ct = 0

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
        self.act_state = np.array([[x_init],[vx_init],[y_init],[vy_init],[orientation_init],[ang_vel_init]])
        #predicted state
        self.pred_state = np.array([[0],[0],[0],[0],[0],[0]])
        #motion model
        self.matrix_A = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #covariance of instance t-1
        self.prev_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #covariance of instance t
        self.act_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #predicted covariance
        self.pred_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #Jacobian matrix
        self.matrix_H = np.zeros((2, 6))
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
        self.map = self.openImage()

        self.gama = 10


    def openImage(self):

        new_map = np.zeros((101, 101))

        new_map[30, :] = 1
        new_map[70, :] = 1
        new_map[:, 30] = 1
        new_map[:, 70] = 1

        for i in range(0, 101):
            for j in range(0, 101):
                if (i < 30 or i > 70):
                    new_map[i, j] = -1
                if (j < 30 or j > 70):
                    new_map[i, j] = -1

        return new_map


    def robot_localization(self):

        while(1):

            global ct
            if(ct > 0):
                ct = 0

            print(self.prev_state);

            ys = self.prev_state[2] +0.0
            xs = self.prev_state[0] +0.0

            plt.ion()
            fig=plt.figure(1)
            pl.figure(1)
            ax = fig.add_subplot(111)
            line1, = ax.plot(xr[ct], yr[ct], 'ro')
            
            line1, = ax.plot(xs-50, 50-ys, 'go')

            s = -2 * math.log(1 - 0.95)
            w, v=LA.eig(self.pred_cov*s)

            t = np.linspace(0, 2*math.pi, 100)
            plt.plot( -50+xs+w[1]*np.cos(t) , 50-ys+w[2]*np.sin(t) )
            plt.grid(color='lightgray',linestyle='--')

            #pl.plot(xs,y)
            plt.axis([-50, 50, -50, 50])
            #line1.set_ydata(np.sin(0.5 * x + phase))
            fig.canvas.draw()
            #plt.axis([0, 5, 0, 5])
            #a = anim.FuncAnimation(fig, update, frames=10, repeat=False)
            #plt.show()
            plt.gcf().clear()

            self.predition_step()

        # #node of drone to Subscribe IMU data
        # self.subsIMU = rospy.Subscriber('/imu/data_raw',Imu,self.sub_pub_calRot)
        # #and to to Subscribe camera data
        # self.image_sub = rospy.Subscriber('/camera/depth/image',Image,self.save_image)
        # rate = rospy.Rate(10)

        # rospy.spin()
            self.line_z = np.concatenate((dm,orir))

            points = self.observation_model(len(self.line_z))
            
            if(self.matching_step(points).all() <= self.gama):
                self.update_step()

            #global ct
            ct += 1




    def predition_step(self):

        self.prev_time = self.act_time + 0.0
        self.act_time +=1
        self.delta_time = self.act_time - self.prev_time +0.0


        self.matrix_A[0][1] = self.delta_time +0.0
        self.matrix_A[2][3] = self.delta_time +0.0
        self.matrix_A[4][5] = self.delta_time +0.0

        self.prev_state = self.act_state +0.0
        self.prev_cov = self.act_cov +0.0

        self.pred_state = self.matrix_A.dot(self.prev_state) +0.0

        if (self.pred_state[4] > np.pi):
            self.pred_state[4] = self.pred_state[4] - 2 * np.pi * int(self.pred_state[4]/(2*np.pi)+1)
        elif (self.pred_state[4] <= -np.pi):
            self.pred_state[4] = 2 * np.pi * int(self.pred_state[4]/(2*np.pi)+1) - self.pred_state[4]

        

        self.pred_cov = ((self.matrix_A.dot(self.prev_cov)).dot(self.matrix_A.transpose())) + matrix_R +0.0


    def matching_step(self, points):

        size_v = len(self.line_z)

        time.sleep(1)
        self.jacobian(size_v, points[0,:], points[1,:], points[2,:], points[3,:])
        self.matrix_Q = np.identity(size_v)

        v_p = self.line_z - self.h +0.0
        print(self.line_z)
        print(self.h)

        S = self.matrix_H.dot(self.pred_cov.dot(self.matrix_H.transpose()))+self.matrix_Q +0.0
        match = v_p.transpose().dot(LA.inv(S).dot(v_p))
        

        return match


    def update_step(self):

        #Kalman gain

        k = (self.pred_cov.dot(self.matrix_H.transpose())).dot(LA.inv((self.matrix_H.dot(self.pred_cov)).dot(self.matrix_H.transpose()) + self.matrix_Q))

        self.act_state = self.pred_state + k.dot(self.line_z - self.h) +0.0
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov) +0.0
 
        
        


    def sub_pub_calRot(self, data):
        #subscribe the imu data (quaternions) and calculate the rotation matrix
        self.rotation_matrix = quaternions.quat2mat(data)


    def save_image(self, photo):
        self.frame = photo 
        self.line_z = self.take_frame_line()

        points = self.observation_model(len(self.line_z) )
        if(self.matching_step(points) <= self.gama):
            update_step()




    def take_frame_line(self):
        #select the line

        line = self.frame[320,:]

        ori = 1

        if self.rotation_matrix[1, 0] != 0:
            ori = (self.rotation_matrix[1, 0]/abs(self.rotation_matrix[1, 0]))

        ori = ori * np.arccos(self.rotation_matrix[0, 0]/LA.norm(np.array([self.rotation_matrix[0, 0], self.rotation_matrix[1, 0], 0])))

        #print(ori)

        line_orient = np.concatenate((a, ori))
        return line



    def observation_model(self, size_vector):
        #function that recieves the pose of the robot and locates the robot in
        #the map. Determines what the drone should see in the given position
        #and orientation
        map = self.map
        #map's size
        length_map = self.map.shape[1]#no of columns
        width_map = self.map.shape[0]#no of rows

        self.h = np.zeros((size_vector, 1))#vector to return with the distances
        middle = int(np.floor(size_vector/2))
        points = np.zeros((4,size_vector-1))

        orient = self.pred_state[4]+0.0

        if (self.pred_state[0] > 69 or self.pred_state[0] < 29 or self.pred_state[2] > 69 or self.pred_state[2] < 29):
            points[0, 0] = self.pred_state[0] +0.0
            points[1, 0] = self.pred_state[2] +0.0
            points[2, 0] = 100000;
            points[3, 0] = 100000;

            self.h[0] = 1000000000;
        else:

            
            #first 2 rows are the points in the photo plane (xs,ys)
            #thrid and fourth row are the points of the object (xf,yf)

            #all the angles are between -pi(exclusive) and pi(inclusive)
            #predicted orientation of the drone
            
            while orient <= -np.pi:
                orient += 2*np.pi
            while orient > np.pi:
                orient -= 2*np.pi

            margin_angle = np.pi/60
            count_pixels = 1
            distance_max = count_pixels * resolution

            #field of view +- 29 degrees
            incr_angle = (29.0*np.pi)/(180*((size_vector-1)/2))
            angle_incre = orient + 0.0

            #predicted position of the drone
            x_incr = self.pred_state[0] +0.0
            x_s = int(self.pred_state[0]) +0
            y_s = int(self.pred_state[2]) +0
            x_m = int(self.pred_state[0]) +0
            y_m = int(self.pred_state[2]) +0



            for i in range(middle, size_vector-1):

                #prediction of position point by the camera
                #Stops when find a obstacle or reachs the max range of camera (5 meters)
                while map[y_m, x_m] == 0 and distance_max < 50 and x_m in range(0, length_map) and y_m in range(0, width_map):
                    while angle_incre <= -np.pi:
                        angle_incre += 2*np.pi
                    while angle_incre > np.pi:
                        angle_incre -= 2*np.pi

                    #determine the increment that depends of the angle
                    #angle next to +- pi/2 the increment are small(slope of the tangent is high)
                    if angle_incre >= 0 and angle_incre <= np.pi/2:
                        #first quadrant
                        aux_yaw = angle_incre
                    elif angle_incre > np.pi/2 and angle_incre <=np.pi:
                        #sendon quadrant
                        aux_yaw = np.pi - angle_incre
                    elif angle_incre < 0 and angle_incre >= -np.pi/2:
                        #fourth quadrant
                        aux_yaw = -angle_incre
                    elif angle_incre >= -np.pi and angle_incre <-np.pi/2:
                        #thrid quadrant
                        aux_yaw = angle_incre + np.pi

                    if aux_yaw > np.pi/3 and aux_yaw < 7*np.pi/18:
                        #between 60 and 70 degrees
                        increment = 0.05
                    elif aux_yaw >= 7*np.pi/18 and aux_yaw < 8*np.pi/18:
                        #between 70 and 80 degrees
                        increment = 0.01
                    elif aux_yaw >= 8*np.pi/18 and aux_yaw < 85*np.pi/180:
                        #between 80 an 85 degrees
                        increment = 0.005
                    elif aux_yaw >= 85*np.pi/180 and aux_yaw <= np.pi/2:
                        #between 85 an 90 degrees
                        increment = 0.001
                    else:
                        increment = 0.1

                    #straight line since the camere to the front
                    if angle_incre >= 0 and angle_incre < np.pi/2-margin_angle :
                        #first quadrant
                        x_incr = x_incr + increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

                    elif angle_incre > np.pi/2+margin_angle  and angle_incre <= np.pi:
                        #sencond quadrant
                        x_incr = x_incr - increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(np.pi-angle_incre)*( x_s - x_incr ) + y_s)

                    elif angle_incre > -np.pi/2+margin_angle  and angle_incre < 0:
                        #fourth quadrant
                        x_incr = x_incr + increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

                    elif angle_incre > -np.pi-margin_angle and angle_incre < -np.pi/2 :
                        #third quadrant
                        x_incr = x_incr - increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(-np.pi-angle_incre)*(x_s - x_incr) + y_s)

                    elif angle_incre > np.pi/2-margin_angle and angle_incre < np.pi/2+margin_angle:
                        y_m = int(y_m - 1)

                    elif angle_incre > -np.pi/2-margin_angle and angle_incre < -np.pi/2+margin_angle:
                        y_m = int(y_m + 1)

                    count_pixels += 1
                    distance_max = count_pixels * resolution * increment


                p_radial = np.array([[x_s-x_m],[y_s-y_m]])
                dis_radial = LA.norm(p_radial)
                print(i)
                self.h[i] = resolution *dis_radial*np.cos(angle_incre-orient)

                points[0, i] = dis_radial*np.cos(angle_incre-orient)*np.sin(angle_incre-orient) + x_s
                points[1, i] = dis_radial*np.power(np.sin(angle_incre-orient),2)+y_s
                points[2, i] = x_m
                points[3, i] = y_m

                angle_incre -= incr_angle

                count_pixels = 1
                distance_max = count_pixels * resolution
                x_m = x_s
                y_m = y_s
                x_incr = x_s


            count_pixels = 1
            distance_max = count_pixels * resolution
            angle_incre = orient + incr_angle
            x_m = x_s
            y_m = y_s
            x_incr = x_s


            for j in range (middle-1, -1, -1):
                while map[y_m, x_m] == 0 and distance_max < 50 and x_m in range(0, length_map) and y_m in range(0, width_map):
                    while angle_incre <= -np.pi:
                        angle_incre += 2*np.pi
                    while angle_incre > np.pi:
                        angle_incre -= 2*np.pi

                    if angle_incre >= 0 and angle_incre <= np.pi/2:
                        #first quadrant
                        aux_yaw = angle_incre
                    elif angle_incre > np.pi/2 and angle_incre <=np.pi:
                        #sendon quadrant
                        aux_yaw = np.pi - angle_incre
                    elif angle_incre < 0 and angle_incre >= -np.pi/2:
                        #fourth quadrant
                        aux_yaw = -angle_incre
                    elif angle_incre >= -np.pi and angle_incre <-np.pi/2:
                        #thrid quadrant
                        aux_yaw = angle_incre + np.pi

                    if aux_yaw > np.pi/3 and aux_yaw < 7*np.pi/18:
                        #between 60 and 70 degrees
                        increment = 0.05
                    elif aux_yaw >= 7*np.pi/18 and aux_yaw < 8*np.pi/18:
                        #between 70 and 80 degrees
                        increment = 0.01
                    elif aux_yaw >= 8*np.pi/18 and aux_yaw < 85*np.pi/180:
                        #between 80 an 85 degrees
                        increment = 0.005
                    elif aux_yaw >= 85*np.pi/180 and aux_yaw <= np.pi/2:
                        #between 85 an 90 degrees
                        increment = 0.001
                    else:
                        increment = 0.1

                    if angle_incre >= 0 and angle_incre < np.pi/2-margin_angle :
                        #first quadrant
                        x_incr = x_incr + increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

                    elif angle_incre > np.pi/2+margin_angle  and angle_incre <= np.pi:
                        #sencond quadrant
                        x_incr = x_incr - increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(np.pi-angle_incre)*( x_s - x_incr ) + y_s)

                    elif angle_incre > -np.pi/2+margin_angle  and angle_incre < 0:
                        #fourth quadrant
                        x_incr = x_incr + increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

                    elif angle_incre > -np.pi-margin_angle and angle_incre < -np.pi/2 :
                        #third quadrant
                        x_incr = x_incr - increment
                        x_m = int(x_incr)
                        y_m = int(-np.tan(-np.pi-angle_incre)*(x_s - x_incr) + y_s)

                    elif angle_incre > np.pi/2-margin_angle and angle_incre < np.pi/2+margin_angle:
                        y_m = int(y_m - 1)

                    elif angle_incre > -np.pi/2-margin_angle and angle_incre < -np.pi/2+margin_angle:
                        y_m = int(y_m + 1)


                    count_pixels += 1
                    distance_max = count_pixels * resolution * increment


                p_radial = np.array([[x_s-x_m],[y_s-y_m]])
                dis_radial = LA.norm(p_radial)
                self.h[j] = resolution *dis_radial*np.cos(angle_incre-orient)

                points[0, j] = dis_radial*np.cos(angle_incre-orient)*np.sin(angle_incre-orient) + x_s
                points[1, j] = dis_radial*np.power(np.sin(angle_incre-orient),2)+y_s
                points[2, j] = x_m
                points[3, j] = y_m

                angle_incre += incr_angle

                count_pixels = 1
                distance_max = count_pixels * resolution
                x_m = x_s
                y_m = y_s
                x_incr = x_s

        #predited orientation
        self.h[size_vector-1] = self.pred_state[4] +0.0

        return points


    def jacobian(self, size_vector, xs, ys, xp, yp):
        #determine the jacobian of h
        self.matrix_H = np.zeros((size_vector, 6))

        for it in range(0, size_vector-1):
            self.matrix_H[it,:] = self.partial_jacobian1(xs[it], ys[it], xp[it], yp[it])

        self.matrix_H[size_vector-1, :] = self.partial_jacobian2()


    def partial_jacobian1(self, xs1, ys1, xp1, yp1):

        d_h = np.zeros(6)
        d_h[0] = (xs1-xp1)/math.sqrt(np.power(xs1-xp1, 2)+np.power(ys1-yp1, 2))
        d_h[2] = (ys1-yp1)/math.sqrt(np.power(xs1-xp1, 2)+np.power(ys1-yp1, 2))

        return d_h

    def partial_jacobian2(self):

        d_h = np.zeros(6)
        d_h[4] = 1

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
        prog.robot_localization()



    #except rospy.RosInterruptException:
        #pass
