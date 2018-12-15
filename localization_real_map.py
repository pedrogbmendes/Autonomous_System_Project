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
from sensor_msgs.msg import Image, Imu, MagneticField
from std_msgs.msg import String
import time
import math
from PIL import Image as img_pil
from cv_bridge import CvBridge
#for plot the results
from matplotlib import colors
import matplotlib.animation as anim
import pylab as pl
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

#-----------------------------------------------------------------------------I
#
#   Global constants
#
#-----------------------------------------------------------------------------
I = np.identity(6)
cov_x =1000
cov_y = 1000
cov_teta = 1500
matrix_R = np.array([[0,0,0,0,0,0],[0,cov_x,0,0,0,0],[0,0,0,0,0,0],[0,0,0,cov_y,0,0],[0,0,0,0,0,0],[0,0,0,0,0,cov_teta]])

#Camera coordenate frames vectors
v_x = np.array([1,0,0])
v_y = np.array([0,1,0])
v_z = np.array([0,0,1])

#map info
resolution = 50 #meters/pixel

#INITIAL CONDITIONS
x_init = 350
vx_init = 0
y_init = 60
vy_init = 0
orientation_init = -np.pi/2
ang_vel_init = 0

np.set_printoptions(threshold=4)

global c
c = 0

global no_update
no_update = 0
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
        self.act_cov = np.identity(6)*1000
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

        self.gama = 100000000


    def openImage(self):

        img = img_pil.open("map.pgm")
        area = (950, 950, 1600, 1130) #left, top, right, bottom
        cropped_img = img.crop(area)
        img_matrix = np.array(cropped_img)

        #unknown positions of map
        BW_img_des = img_matrix == 205
        BW_img_des = BW_img_des * - 1
        #occupied positions of the map
        BW_img_oc = img_matrix == 0
        BW_img_oc = BW_img_oc* 1  #0 and 1 instead of False and True

        return BW_img_des+BW_img_oc



    def robot_localization(self):

        xr = 350
        yr = 70
        orir = np.array([[-np.pi/2]])

        while(1):
            dm = self.calc_dist(20, xr, yr, orir)

            #print(self.act_state);

            ys = self.act_state[2] +0.0
            xs = self.act_state[0] +0.0

            self.predition_step()

            print self.act_state

        # #node of drone to Subscribe IMU data
        # self.subsIMU = rospy.Subscriber('/imu/data_raw',Imu,self.sub_pub_calRot)
        # #and to to Subscribe camera data
        # self.image_sub = rospy.Subscriber('/camera/depth/image',Image,self.save_image)
        # rate = rospy.Rate(10)

        # rospy.spin()
            self.line_z = dm

            self.observation_model(len(self.line_z))
            global no_update
            if(no_update == 0):
                if(self.matching_step() <= self.gama):

                    self.update_step()

            elif no_update == 5:
                #the robot is lost
                #solve the kidnapping
                #intialization
                no_update = 0
                #
                # self.prev_time = self.act_time + 0.0
                # self.act_time +=1
                # self.delta_time = self.act_time - self.prev_time +0.0
                #
                # self.prev_state = np.array([[0],[0],[0],[0],[0],[0]])
                # ori_init = self.act_state[4]
                # self.act_state = np.array([[300],[vx_init],[50],[vy_init],[ori_init],[ang_vel_init]])
                # self.pred_state = np.array([[0],[0],[0],[0],[0],[0]])
                #
                # self.prev_cov = np.identity(6)
                # self.act_cov = np.identity(6)*2
                # self.pred_cov = np.identity(6)
                #
                # ys = self.act_state[2] +0.0
                # xs = self.act_state[0] +0.0
                # self.predition_step()
                #
                # self.line_z = dm
                # points = self.observation_model(len(self.line_z))
                # if(self.matching_step() <= self.gama):
                #     self.update_step()



            #print(xr, yr, orir)

            # if ( xr >=200 and xr < 400 and yr == 30 and orir == 0):
            #     xr += 1
            # elif (xr == 400 and yr == 30 and orir <= 0 and orir > -np.pi/2 ):
            #     orir = orir - np.array([[np.pi/40]])
            #
            # elif (orir == -np.pi/2 and yr >= 30 and yr < 65 and xr == 400):
            #     yr += 1
            #
            #
            # elif ( xr == 400  and yr == 65 and orir <= -np.pi/2 and orir > -np.pi):
            #     if ct == 10:
            #         orir =  orir - np.array([[np.pi/40]])
            #     else:
            #         ct += 1
            #
            # elif (orir == np.pi and xr <= 400 and yr==65 and xr>200):
            #     ct = 0
            #     xr -= 1
            #
            # elif(orir <= np.pi and orir > np.pi/2 and xr == 200 and yr==65):
            #     orir = orir - np.array([[np.pi/40]])
            #
            # elif (orir == np.pi/2 and yr <= 65 and yr>50 and xr == 200):
            #     yr -= 1
            #
            # elif (orir <= np.pi/2  and orir >= 0 and yr==50 and xr == 200):
            #     orir = orir - np.array([[np.pi/40]])
            #
            # elif ( xr >=200 and xr < 400 and yr == 50 and orir == 0):
            #     xr += 1

            print self.h
            print self.line_z

            plt.ion()
            fig=plt.figure(1,figsize=(80,60))
            pl.figure(1)
            ax = fig.add_subplot(111)
            cmap = colors.ListedColormap(['grey', 'yellow', 'black'])
            ax.pcolor(self.map[::-1], cmap=cmap, edgecolors='k')


            cov_plot = np.array([[self.act_cov[0, 0], self.act_cov[0, 2]], [self.act_cov[2, 0], self.act_cov[2, 2]]])


            endymin = 179-ys +(10* math.sin(self.act_state[4]-0.5061))
            endxmin = xs +(10* math.cos(self.act_state[4]-0.5061))
            endymax = 179-ys +(10* math.sin(self.act_state[4]+0.5061))
            endxmax = xs +(10* math.cos(self.act_state[4]+0.5061))

            endyrmin = 179-yr +(10* math.sin(orir-0.5061))
            endyrmax = 179-yr +(10* math.sin(orir+0.5061))
            endxrmin = xr +(10* math.cos(orir-0.5061))
            endxrmax = xr +(10* math.cos(orir+0.5061))


            line1, = ax.plot(xr, 179-yr, 'ro')
            line1, = ax.plot(xs, 179-ys, 'go')

            s = -2 * math.log(1 - 0.95)

            w, v = LA.eig(cov_plot*s)
            order = w.argsort()[::-1]

            w_ = w[order]
            v_ = v[:,order]

            angle = np.degrees(np.arctan2(*v_[:,0][::-1]))
            #angle = np.degrees(np.arctan2(v[1, 0], v[0,0]))
            #print cov_plot

            pos = [xs, 179-ys]

            width =  2 * np.sqrt(w_[0])
            height = 2 * np.sqrt(w_[1])




            # ells = Ellipse([xs-50, 50-ys], 2*np.sqrt(w[0]), 2*np.sqrt(w[1]), angle * 180 / np.pi)
            # ax.add_artist(ells)

            #t = np.linspace(0, 2*math.pi, 1000)
            #plt.plot( xs+((2*np.sqrt(w[0])))*np.cos(t) , 179-ys+((2*np.sqrt(w[1])))*np.sin(t) )
            #plt.grid(color='lightgray',linestyle='--')


            ax.plot([xr, endxrmin], [179-yr, endyrmin], 'r')
            ax.plot([xr, endxrmax], [179-yr, endyrmax], 'r')
            ax.plot([xs, endxmin], [179-ys, endymin], 'g')
            ax.plot([xs, endxmax], [179-ys, endymax], 'g')
            ells= Ellipse(xy=pos, width=width, height=height, angle=angle, color='black')
            ells.set_facecolor('none')
            ax.add_artist(ells)


            fig.canvas.draw()
            #plt.axis([0, 5, 0, 5])
            #a = anim.FuncAnimation(fig, update, frames=10, repeat=False)
            #plt.show()
            plt.gcf().clear()






    def predition_step(self):

        self.prev_time = self.act_time + 0.0
        self.act_time +=1
        self.delta_time = self.act_time - self.prev_time +0.0


        self.matrix_A[0][1] = self.delta_time +0.0
        self.matrix_A[2][3] = self.delta_time +0.0
        self.matrix_A[4][5] = self.delta_time +0.0

        self.prev_state = self.act_state +0.0
        self.prev_cov = self.act_cov +0.0

        self.pred_state = self.matrix_A.dot(self.act_state) +0.0

        while self.pred_state[4] <= -np.pi:
            self.pred_state[4] += 2*np.pi
        while self.pred_state[4] > np.pi:
            self.pred_state[4] -= 2*np.pi


        # if (self.pred_state[4] > np.pi):
        #     self.pred_state[4] = self.pred_state[4] - 2 * np.pi * int(self.pred_state[4]/(2*np.pi)+1)
        # elif (self.pred_state[4] <= -np.pi+0.01):
        #     self.pred_state[4] = 2 * np.pi * int(self.pred_state[4]/(2*np.pi)+1) - self.pred_state[4]


        self.pred_cov = ((self.matrix_A.dot(self.act_cov)).dot(self.matrix_A.transpose())) + matrix_R +0.0


    def matching_step(self):

        size_v = len(self.line_z)
        #time.sleep(1)
        self.matrix_Q = np.identity(size_v)*1000

        v_p = self.line_z - self.h +0.0

        #print(self.line_z)
        #print(self.h)

        S = self.matrix_H.dot(self.pred_cov.dot(self.matrix_H.transpose()))+self.matrix_Q +0.0
        match = v_p.transpose().dot(LA.inv(S).dot(v_p))

        return match


    def update_step(self):
        #Kalman gain
        k = (self.pred_cov.dot(self.matrix_H.transpose())).dot(LA.inv((self.matrix_H.dot(self.pred_cov)).dot(self.matrix_H.transpose()) + self.matrix_Q))

        state_correction = k.dot(self.line_z - self.h)
        for m in range(0,4):
            state_correction[m] = state_correction[m]/resolution

        while state_correction[4] <= -np.pi:
            state_correction[4]  += 2*np.pi
        while state_correction[4]  > np.pi:
            state_correction[4]  -= 2*np.pi


        print "hhhhhdda"
        print state_correction

        self.act_state = self.pred_state + state_correction
        while self.act_state[4] <= -np.pi:
            self.act_state[4]  += 2*np.pi
        while self.act_state[4]  > np.pi:
            self.act_state[4]  -= 2*np.pi


        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov) +0.0





    # def sub_pub_calRot(self, data):
    #     #subscribe the imu data (quaternions) and calculate the rotation matrix
    #     self.rotation_matrix = quaternions.quat2mat(data)
    #
    #
    # def save_image(self, photo):
    #     self.frame = photo
    #     self.line_z = self.take_frame_line()
    #
    #     points = self.observation_model(len(self.line_z) )
    #     if(self.matching_step(points) <= self.gama):
    #         update_step()
    #
    #
    #
    #
    # def take_frame_line(self):
    #     #select the line
    #
    #     line = self.frame[320,:]
    #
    #     ori = 1
    #
    #     if self.rotation_matrix[1, 0] != 0:
    #         ori = (self.rotation_matrix[1, 0]/abs(self.rotation_matrix[1, 0]))
    #
    #     ori = ori * np.arccos(self.rotation_matrix[0, 0]/LA.norm(np.array([self.rotation_matrix[0, 0], self.rotation_matrix[1, 0], 0])))
    #
    #     #print(ori)
    #
    #     line_orient = np.concatenate((a, ori))
    #     return line



    def observation_model(self, size_vector):
     #function that recieves the pose of the robot and locates the robot in
        #the map. Determines what the drone should see in the given position
        #and orientation
        map = self.map
        #map's size
        length_map = self.map.shape[1]#no of columns
        width_map = self.map.shape[0] #no of rows

        self.h = np.zeros((size_vector, 1))#vector to return with the distances
        middle = int(np.floor((size_vector)/2))
        points = np.zeros((4,size_vector))
        v_angles = np.zeros(size_vector)
        v_dis = np.zeros(size_vector)
        #predicted position of the drone
        x_incr = self.pred_state[0] +0.0
        x_s = int(self.pred_state[0]) +0
        y_s = int(self.pred_state[2]) +0
        x_m = int(self.pred_state[0]) +0
        y_m = int(self.pred_state[2]) +0

        ct = 0

        global no_update

        orient = self.pred_state[4]+0.0

        if x_s in range(0, length_map-1) and y_s in range(0, width_map-1):
            if (map[y_s, x_s] != 0):
                no_update += 1
            else:

                no_update = 0
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
                incr_angle = (29.0*np.pi)/(180*((size_vector)/2))
                angle_incre = orient + 0.0

                for i in range(middle, size_vector):
                    #prediction of position point by the camera
                    #Stops when find a obstacle or reachs the max range of camera (5 meters)
                    while map[y_m, x_m] == 0 and distance_max < 5000 and x_m in range(0, length_map) and y_m in range(0, width_map):

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
                            y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)


                        elif angle_incre > -np.pi/2+margin_angle  and angle_incre < 0:
                            #fourth quadrant
                            x_incr = x_incr + increment
                            x_m = int(x_incr)
                            y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)


                        elif angle_incre > -np.pi and angle_incre < -np.pi/2-margin_angle :
                            #third quadrant
                            x_incr = x_incr - increment
                            x_m = int(x_incr)
                            y_m = int(-np.tan(angle_incre)*(x_incr - x_s) + y_s)

                        elif angle_incre >= np.pi/2-margin_angle and angle_incre <= np.pi/2+margin_angle:
                            y_m = int(y_m - 1)

                        elif angle_incre >= -np.pi/2-margin_angle and angle_incre <= -np.pi/2+margin_angle:
                            y_m = int(y_m + 1)

                        count_pixels += 1
                        distance_max = count_pixels * resolution * increment


                    p_radial = np.array([[x_s-x_m],[y_s-y_m]])
                    dis_radial = LA.norm(p_radial)*resolution

                    x_sreal = x_s * resolution
                    y_sreal = y_s * resolution

                    x_mreal = x_m * resolution
                    y_mreal = y_m * resolution

                    phi = (np.pi/2)-(incr_angle*ct)

                    cateto = dis_radial * np.cos(phi)

                    x_i = x_sreal + cateto*np.cos(orient-(np.pi/2))
                    y_i = y_sreal + cateto*np.sin(orient-(np.pi/2))


                    points[0, i] = x_i
                    points[1, i] = y_i
                    points[2, i] = x_mreal
                    points[3, i] = y_mreal

                    h_vector = np.array([[x_i-x_sreal],[y_i-y_sreal]])
                    self.h[i] = LA.norm(h_vector)

                    v_angles[i] = orient - incr_angle*ct
                    v_dis[i] = dis_radial

                    angle_incre -= incr_angle
                    ct += 1

                    count_pixels = 1
                    distance_max = count_pixels * resolution
                    x_m = x_s + 0
                    y_m = y_s + 0
                    x_incr = x_s + 0


                count_pixels = 1
                distance_max = count_pixels * resolution
                angle_incre = orient + incr_angle
                x_m = x_s +0
                y_m = y_s +0
                x_incr = x_s +0
                ct = 1


                for j in range (middle-1, -1, -1):
                    while map[y_m, x_m] == 0 and distance_max < 5000 and x_m in range(0, length_map) and y_m in range(0, width_map):
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

                        elif angle_incre > -np.pi and angle_incre < -np.pi/2-margin_angle  :
                            #third quadrant
                            x_incr = x_incr - increment
                            x_m = int(x_incr)
                            y_m = int(-np.tan(-np.pi-angle_incre)*(x_s - x_incr) + y_s)

                        elif angle_incre >= np.pi/2-margin_angle and angle_incre <= np.pi/2+margin_angle:
                            y_m = int(y_m - 1)

                        elif angle_incre >= -np.pi/2-margin_angle and angle_incre <= -np.pi/2+margin_angle:
                            y_m = int(y_m + 1)


                        count_pixels += 1
                        distance_max = count_pixels * resolution * increment


                    p_radial = np.array([[x_s-x_m],[y_s-y_m]])
                    dis_radial = LA.norm(p_radial)*resolution

                    x_sreal = x_s * resolution
                    y_sreal = y_s * resolution

                    x_mreal = x_m * resolution
                    y_mreal = y_m * resolution

                    phi = (np.pi/2)-(incr_angle*ct)

                    cateto = dis_radial * np.cos(phi)

                    x_i = x_sreal + cateto*np.cos(orient+(np.pi/2))
                    y_i = y_sreal + cateto*np.sin(orient+(np.pi/2))

                    points[0, j] = x_i
                    points[1, j] = y_i
                    points[2, j] = x_mreal
                    points[3, j] = y_mreal

                    h_vector = np.array([[x_i-x_sreal],[y_i-y_sreal]])
                    self.h[j] = LA.norm(h_vector)

                    v_angles[j] = orient + incr_angle*ct
                    v_dis[j] = dis_radial

                    angle_incre += incr_angle
                    ct += 1

                    count_pixels = 1
                    distance_max = count_pixels * resolution
                    x_m = x_s +0
                    y_m = y_s +0
                    x_incr = x_s +0


        else:
            #position outside the map
            no_update += 1;


        self.jacobian(size_vector, points[0,:], points[1,:],points[2,:], points[3,:],  v_dis, v_angles)



    def calc_dist(self, size_vector, xr, yr, orient):
            #function that recieves the pose of the robot and locates the robot in
        #the map. Determines what the drone should see in the given position
        #and orientation
        map = self.map
        #map's size
        length_map = self.map.shape[1]#no of columns
        width_map = self.map.shape[0]#no of rows

        dist = np.zeros((size_vector, 1))#vector to return with the distances
        middle = int(np.floor((size_vector)/2))


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
        incr_angle = (29.0*np.pi)/(180*((size_vector)/2))
        angle_incre = orient + 0.0

        #predicted position of the drone
        x_incr = xr
        x_s = xr
        y_s = yr
        x_m = xr
        y_m = yr

        ct = 0

        for i in range(middle, size_vector):

            #prediction of position point by the camera
            #Stops when find a obstacle or reachs the max range of camera (5 meters)
            while map[y_m, x_m] == 0 and distance_max < 5000 and x_m in range(0, length_map) and y_m in range(0, width_map):
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
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre > np.pi/2+margin_angle  and angle_incre <= np.pi:
                    #sencond quadrant
                    x_incr = x_incr - increment
                    x_m = int(x_incr)
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre > -np.pi/2+margin_angle  and angle_incre < 0:
                    #fourth quadrant
                    x_incr = x_incr + increment
                    x_m = int(x_incr)
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre > -np.pi and angle_incre < -np.pi/2 -margin_angle:
                    #third quadrant
                    x_incr = x_incr - increment
                    x_m = int(x_incr)
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre >= np.pi/2-margin_angle and angle_incre <= np.pi/2+margin_angle:
                    y_m = int(y_m - 1)

                elif angle_incre >= -np.pi/2-margin_angle and angle_incre <= -np.pi/2+margin_angle:
                    y_m = int(y_m + 1)

                count_pixels += 1
                distance_max = count_pixels * resolution * increment

            p_radial = np.array([[x_s-x_m],[y_s-y_m]])
            dis_radial = LA.norm(p_radial)*resolution

            x_sreal = x_s * resolution
            y_sreal = y_s * resolution


            phi = (np.pi/2)-(incr_angle*ct)
            cateto = dis_radial * np.cos(phi)

            x_i = x_sreal + cateto*np.cos(orient-(np.pi/2))
            y_i = y_sreal + cateto*np.sin(orient-(np.pi/2))

            h_vector = np.array([[x_i-x_sreal],[y_i-y_sreal]])
            dist[i] = LA.norm(h_vector)

            ct += 1
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
        ct = 1

        for j in range (middle-1, -1, -1):
            while map[y_m, x_m] == 0 and distance_max < 5000 and x_m in range(0, length_map) and y_m in range(0, width_map):
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
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre > np.pi/2+margin_angle  and angle_incre <= np.pi:
                    #sencond quadrant
                    x_incr = x_incr - increment
                    x_m = int(x_incr)
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre > -np.pi/2+margin_angle  and angle_incre < 0:
                    #fourth quadrant
                    x_incr = x_incr + increment
                    x_m = int(x_incr)
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre > -np.pi and angle_incre < -np.pi/2-margin_angle :
                    #third quadrant
                    x_incr = x_incr - increment
                    x_m = int(x_incr)
                    y_m = int(-np.tan(angle_incre)*( x_incr - x_s ) + y_s)

                elif angle_incre >= np.pi/2-margin_angle and angle_incre < np.pi/2+margin_angle:
                    y_m = int(y_m - 1)

                elif angle_incre >= -np.pi/2-margin_angle and angle_incre < -np.pi/2+margin_angle:
                    y_m = int(y_m + 1)


                count_pixels += 1
                distance_max = count_pixels * resolution * increment

            p_radial = np.array([[x_s-x_m],[y_s-y_m]])
            dis_radial = LA.norm(p_radial)*resolution

            x_sreal = x_s * resolution
            y_sreal = y_s * resolution

            phi = (np.pi/2)-(incr_angle*ct)
            cateto = dis_radial * np.cos(phi)

            x_i = x_sreal + cateto*np.cos(orient-(np.pi/2))
            y_i = y_sreal + cateto*np.sin(orient-(np.pi/2))

            h_vector = np.array([[x_i-x_sreal],[y_i-y_sreal]])
            dist[j] = LA.norm(h_vector)


            angle_incre += incr_angle
            ct += 1
            count_pixels = 1
            distance_max = count_pixels * resolution
            x_m = x_s
            y_m = y_s
            x_incr = x_s

        return dist


    def jacobian(self, size_vector, xs, ys,xp, yp,  v_d, ang):
        #determine the jacobian of h
        self.matrix_H = np.zeros((size_vector, 6))

        for it in range(0, size_vector):
            self.matrix_H[it,:] = self.partial_jacobian(xs[it], ys[it],xp[it], yp[it], v_d[it], ang[it])




    def partial_jacobian(self, xs1, ys1,xp1, yp1, d, ang):

        d_h = np.zeros(6)
        xr = self.pred_state[0]*resolution;
        yr = self.pred_state[2]*resolution;

        if xs1 == xp1 and ys1 == yp1:
            d_h[0] = 0
            d_h[2] = 0
        else:
            d_h[0] = (xr+d*np.cos(ang)-xs1) / math.sqrt(np.power(xr+d*np.cos(ang)-xs1, 2) + np.power(yr+d*np.sin(ang)-ys1, 2))
            d_h[2] = (yr+d*np.sin(ang)-ys1) / math.sqrt(np.power(xr+d*np.cos(ang)-xs1, 2) + np.power(yr+d*np.sin(ang)-ys1, 2))
            d_h[4] =(((xr+d*np.cos(ang)-xs1)*(-d*np.sin(ang)))+((yr+d*np.sin(ang)-ys1)*(d*np.cos(ang))))/math.sqrt(np.power(xr+d*np.cos(ang)-xs1,2)+np.power(yr+d*np.sin(ang)-ys1,2))

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
