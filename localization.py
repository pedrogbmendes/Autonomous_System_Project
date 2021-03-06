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
from transforms3d import quaternions
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

#-----------------------------------------------------------------------------I
#
#   Global constants
#
#-----------------------------------------------------------------------------
I = np.identity(6)
cov_x = .1
cov_y = .1
cov_teta = .01
matrix_R = np.array([[0,0,0,0,0,0],[0,cov_x,0,0,0,0],[0,0,0,0,0,0],[0,0,0,cov_y,0,0],[0,0,0,0,0,0],[0,0,0,0,0,cov_teta]])

#Camera coordenate frames vectors
v_x = np.array([1,0,0])
v_y = np.array([0,1,0])
v_z = np.array([0,0,1])

#map info
resolution = 50 #milimeters/pixel

#INITIAL CONDITIONS
x_init = 300
vx_init = 0
y_init = 50
vy_init = 0
orientation_init = np.pi/2
ang_vel_init = 0

#matching step
global no_update
no_update = 0

np.set_printoptions(threshold=4)

#-----------------------------------------------------------------------------
#
#   EKF
#
#-----------------------------------------------------------------------------

class EKF_localization:

    def __init__(self):

        #previous time
        self.prev_time = rospy.get_time()
        #actual time
        self.act_time = rospy.get_time()
        #difference of time
        self.delta_time = rospy.get_time()
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
        #matching threshold
        self.gama = 1000^2


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

        #to convert the frame of teh camera in the correct byte size
        self.bridge = CvBridge()
        #node of drone to Subscribe IMU data
        self.subsIMU = rospy.Subscriber('/imu/data',Imu,self.sub_pub_calRot)
        #and to to Subscribe camera data
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw',Image,self.save_image)
        rate = rospy.Rate(20)

        rospy.spin()



    def predition_step(self):

        self.prev_time = self.act_time + 0.0
        self.act_time = rospy.get_time()
        self.delta_time = self.act_time - self.prev_time


        self.matrix_A[0][1] = self.delta_time + 0.0
        self.matrix_A[2][3] = self.delta_time + 0.0
        self.matrix_A[4][5] = self.delta_time + 0.0

        self.prev_state = self.act_state + 0.0
        self.prev_cov = self.act_cov + 0.0


        self.pred_state = self.matrix_A.dot(self.prev_state)

        if (self.pred_state[4] > np.pi):
            self.pred_state[4] = self.pred_state[4] - 2 * np.pi * int(self.pred_state[4]/(2*np.pi)+1)
        elif (self.pred_state[4] <= -np.pi):
            self.pred_state[4] = 2 * np.pi * int(self.pred_state[4]/(2*np.pi)+1) - self.pred_state[4]

        self.pred_cov = ((self.matrix_A.dot(self.prev_cov)).dot(self.matrix_A.transpose())) + matrix_R


    def matching_step(self, points):

        size_v = len(self.line_z)

        self.matrix_Q = np.identity(size_v)

        v_p = self.line_z - self.h

        S = self.matrix_H.dot(self.pred_cov.dot(self.matrix_H.transpose()))+self.matrix_Q
        match = v_p.transpose().dot(LA.inv(S).dot(v_p))
        print match
        return match


    def update_step(self):

        #Kalman gain

        k = (self.pred_cov.dot(self.matrix_H.transpose())).dot(LA.inv((self.matrix_H.dot(self.pred_cov)).dot(self.matrix_H.transpose()) + self.matrix_Q))

        self.act_state = self.pred_state + k.dot(self.line_z - self.h)
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov)


    def sub_pub_calRot(self, data):
        #subscribe the imu data (quaternions) and calculate the rotation matrix
        quat = data.orientation
        quaternion = [quat.w, quat.x, quat.y, quat.z]
        self.rotation_matrix = quaternions.quat2mat(quaternion)


    def save_image(self, photo):

        self.predition_step()


        self.frame = self.bridge.imgmsg_to_cv2(photo)
        self.line_z = self.take_frame_line()

        self.observation_model(len(self.line_z) )

        if(no_update == 0):
            if(self.matching_step(points) <= self.gama):

                self.update_step()
        elif no_update == 10:
            #the robot is lost
            #solve the kidnapping
            self.__init__()

        #print self.act_state
        print self.line_z
        print self.h
        plt.ion()
        fig=plt.figure(1)
        pl.figure(1)

        ax = fig.add_subplot(111)

        cmap = colors.ListedColormap(['grey', 'yellow', 'black'])
        ax.pcolor(self.map[::-1], cmap=cmap, edgecolors='k')

        x_plot = self.act_state[0]
        y_plot = 179 - self.act_state[2]
        ax.plot(x_plot,y_plot, 'ro')

        fig.canvas.draw()
        plt.gcf().clear()



    def take_frame_line(self):
        #select the line
        line = np.zeros((50,1));
        aux = 50
        for w in range(0,50):
            line[w] = self.frame[240,aux] + 0.0
            aux+= 10

        return line


    def observation_model(self, size_vector):
        #function that recieves the pose of the robot and locates the robot in
        #the map. Determines what the drone should see in the given position
        #and orientation

        #map's size
        length_map = self.map.shape[1]#no of columns
        width_map = self.map.shape[0]#no of rows


        self.h = np.zeros((size_vector, 1))#vector to return with the distances
        middle = int(np.floor((size_vector)/2))
        points = np.zeros((4,size_vector))
        v_angles = np.zeros(size_vector)
        v_dis = np.zeros(size_vector)

        #first 2 rows are the points in the photo plane (xs,ys)
        #thrid and fourth row are the points of the object (xf,yf)
        global no_update

        if self.pred_state[0] in range(0, length_map-1) and self.pred_state[2] in range(0, width_map-1):

            #all the angles are between -pi(exclusive) and pi(inclusive)
            #predicted orientation of the drone
            orient = self.pred_state[4] + 0.0
            while orient <= -np.pi:
                orient += 2*np.pi
            while orient > np.pi:
                orient -= 2*np.pi

            margin_angle = np.pi/60
            count_pixels = 1
            distance_max = count_pixels * resolution

            #fill of view +- 29 degrees
            incr_angle = (29.0*np.pi)/(180*((size_vector)/2))
            angle_incre = orient + 0.0

            #predicted position of the drone
            x_incr = self.pred_state[0] + 0.0
            x_s = int(self.pred_state[0]) + 0
            y_s = int(self.pred_state[2]) + 0
            x_m = int(self.pred_state[0]) + 0
            y_m = int(self.pred_state[2]) + 0

            if self.map[y_s, x_s] != 0:
                #outside the free known space
                no_update += 1;
            else:

                no_update = 0;

                for i in range(middle, size_vector):

                    #prediction of position point by the camera
                    #Stops when find a obstacle or reachs the max range of camera (5 meters)
                    while self.map[y_m, x_m] == 0 and distance_max < 5000 and x_m in range(0, length_map) and y_m in range(0, width_map):
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

                        elif angle_incre > -np.pi and angle_incre < -np.pi/2-margin_angle :
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


                    if distance_max > 4900:

                        self.h[i] = 0

                        points[0, i] = dis_radial*np.cos(orient-angle_incre)*np.sin(orient-angle_incre) + x_s
                        points[1, i] = dis_radial*np.power(np.sin(orient-angle_incre),2)+y_s
                        points[2, i] = points[0, i] + 0.0
                        points[3, i] = points[1, i] + 0.0

                    else:
                        p_radial = np.array([[x_s-x_m],[y_s-y_m]])
                        dis_radial = LA.norm(p_radial)
                        self.h[i] = resolution *dis_radial*np.cos(orient-angle_incre)

                        points[0, i] = dis_radial*np.cos(orient-angle_incre)*np.sin(orient-angle_incre) + x_s
                        points[1, i] = dis_radial*np.power(np.sin(orient-angle_incre),2)+y_s
                        points[2, i] = x_m
                        points[3, i] = y_m

                    v_angles[i] = orient-angle_incre
                    v_dis[i] = dis_radial*resolution

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
                    while self.map[y_m, x_m] == 0 and distance_max < 5000 and x_m in range(0, length_map) and y_m in range(0, width_map):
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

                        elif angle_incre > -np.pi and angle_incre < -np.pi/2-margin_angle :
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


                    if distance_max > 4900:
                        self.h[j] = 0

                        points[0, j] = dis_radial*np.cos(angle_incre-orient)*np.sin(angle_incre-orient) + x_s
                        points[1, j] = dis_radial*np.power(np.sin(angle_incre-orient),2)+y_s
                        points[2, j] = points[0, j] + 0.0
                        points[3, j] = points[1, j] + 0.0

                    else:
                        p_radial = np.array([[x_s-x_m],[y_s-y_m]])
                        dis_radial = LA.norm(p_radial)
                        self.h[j] = resolution *dis_radial*np.cos(angle_incre-orient)

                        points[0, j] = dis_radial*np.cos(angle_incre-orient)*np.sin(angle_incre-orient) + x_s
                        points[1, j] = dis_radial*np.power(np.sin(angle_incre-orient),2)+y_s
                        points[2, j] = x_m
                        points[3, j] = y_m

                    v_angles[j] = angle_incre-orient
                    v_dis[j] = dis_radial*resolution


                    angle_incre += incr_angle

                    count_pixels = 1
                    distance_max = count_pixels * resolution
                    x_m = x_s
                    y_m = y_s
                    x_incr = x_s

        else:
            #position outside the map
            no_update += 1;

        self.jacobian(size_vector, points[0,:], points[1,:],points[2,:], points[3,:],  v_dis, v_angles)



    def jacobian(self, size_vector, xs, ys,xp, yp,  v_d, ang):
        #determine the jacobian of h
        self.matrix_H = np.zeros((size_vector, 6))

        for it in range(0, size_vector):
            self.matrix_H[it,:] = self.partial_jacobian(xs[it], ys[it],xp[it], yp[it], v_d[it], ang[it])



    def partial_jacobian(self, xs1, ys1,xp1, yp1, d, ang):

        d_h = np.zeros(6)
        xr = self.pred_state[0] + 0
        yr = self.pred_state[2] + 0

        if xs1 == xp1 and ys1 == yp1:
            d_h[0] = 0
            d_h[2] = 0
        else:
            d_h[0] = (-(xs1-xr+d*np.cos(ang))) / math.sqrt(np.power(xs1-xr+d*np.cos(ang), 2) + np.power(ys1-yr+d*np.sin(ang), 2))
            d_h[2] = (-(ys1-yr+d*np.sin(ang))) / math.sqrt(np.power(xs1-xr+d*np.cos(ang), 2) + np.power(ys1-yr+d*np.sin(ang), 2))
            d_h[4] = ((xs1-xr+d*np.cos(ang))*(-d*np.sin(ang)) + (ys1-yr+d*np.sin(ang))*(d*np.cos(ang)) ) / math.sqrt(np.power(xs1-xr+d*np.cos(ang), 2) + np.power(ys1-yr+d*np.sin(ang), 2))

        return d_h




#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    #try:
        rospy.init_node('drone', anonymous=True)
        prog = EKF_localization()
        prog.robot_localization()



    #except rospy.RosInterruptException:
        #pass
      #pass
