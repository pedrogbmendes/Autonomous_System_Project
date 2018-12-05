

import numpy as np
from numpy import linalg as LA
from transforms3d import quaternions
import roslib
import sys
import rospy
from sensor_msgs.msg import Image, Imu
import time
import math
from PIL import Image


#-----------------------------------------------------------------------------I
#
#   Global constants
#
#-----------------------------------------------------------------------------
I = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
cov_x = 1
cov_y = 1
cov_teta = 1
matrix_R = np.array([[0,0,0,0],[0,cov_x,0,0],[0,0,0,0],[0,0,0,cov_y],[0,0,0,0],[0,0,0,cov_teta]])

#Camera coordenate frames vectors
v_x = np.array([1,0,0])
v_y = np.array([0,1,0])
v_z = np.array([0,0,1])

#map info
resolution = 0.05 #meters/pixel
origin = np.array([-51.224998, -51.224998, 0.000000])


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
        self.prev_state = np.array([0,0,0,0,0,0])
        #actual state
        self.act_state = np.array([0,0,0,0,0,0])
        #predicted state
        self.pred_state = np.array([0,0,0,0,0,0])
        #motion model
        self.matrix_A = np.array([[1,self.delta_time,0,0,0,0],[0,1,0,0,0,0],[0,0,1,self.delta_time,0,0],[0,0,0,1,0,0],[0,0,0,0,1,self.delta_time],[0,0,0,0,0,1]])
        #covariance of instance t-1
        self.prev_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #covariance of instance t
        self.act_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #predicted covariance
        self.pred_cov = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        #Jacobian matrix
        self.matrix_H = np.zeros((6, 1))
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

        #node of drone to Subscribe IMU data
        self.subsIMU = rospy.Subscriber('/imu/data_raw',Imu,self.sub_pub_calRot)
        #and to to Subscribe camera data
        self.image_sub = rospy.Subscriber('/camera/depth/image',Image,self.save_image)
        rate = rospy.Rate(10)

        rospy.spin()


    def predition_step(self):

        self.prev_time = self.act_time
	print("a=%s" % self.act_time)
	print("b=%s" % self.prev_time)
        self.act_time = rospy.Time.now()
	print("c=%s" % self.act_time)
        self.delta_time = self.act_time - self.prev_time
	print("d=%s" % self.delta_time)

        self.matrix_A[0][1] = self.delta_time
        self.matrix_A[2][3] = self.delta_time
        self.matrix_A[4][5] = self.delta_time

        self.pred_state = self.matrix_A.dot(self.prev_state)
        self.pred_cov = ((self.matrix_A.dot(self.prev_cov)).dot(self.matrix_A.transpose())) + matrix_R


    def update_step(self):

        #Kalman gain
        k = (self.pred_cov.dot(self.matrix_H.transpose())).dot(inv((self.matrix_H.dot(self.pred_cov)).dot(self.matrix_H.transpose()) + matrix_Q))

        self.act_state = self.pred_state + k.dot(self.line_z - self.h)
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov)


    def sub_pub_calRot(self, data):
        self.rotation_matrix = quaternions.quat2mat(data)


    def save_image(self, photo):
        self.frame = photo
        self.line_z = self.take_frame_line()
        self.h = self.observation_model()
        self.update_step()

    '''
    def take_frame_line(self):

        index_W = np.arange(640)
        index_H = np.repeat(240,480)

        v_x_new = self.rotation_matrix[:,0]
        v_y_new = self.rotation_matrix[:,1]
        v_z_new = self.rotation_matrix[:,2]

        #pitch rotation - rotation of axis y
        angle_pitch = np.arccos( np.dot(v_x, v_x_new )/np.dot( LA.norm(v_x), LA.norm(v_x_new)) )
        if angle_pitch > 0 :
            d_min = self.frame[240] #vector with 640 index
            d_next = self.frame[239]
            d_real_wall = d_min * tan(angle_pitch); #real distance between to points
            dist_pixel = np.sqrt(np.square(d_next)-np.square(d_min))
            if dist_pixel < 0
                dist_pixel = np.absolute(dist_pixel)

            index_aux = np.divide(d_real_wall, dist_pixel)
            index_H = 240 - index_aux #vector containing the 640 index of a line to take of the frame

            if angle_pitch > np.arctan((240*dist_pixel[320])/ d_min[320]):
                #maximum angle achieved
                index_H.fill(0)

        elif angle_pitch < 0
            d_min = self.frame[240] #vector with 640 index
            d_next = self.frame[241]
            d_real_wall = d_min * tan(-angle_pitch); #real distance between to points
            dist_pixel = np.sqrt(np.square(d_next)-np.square(d_min))
            if dist_pixel < 0
                dist_pixel = np.absolute(dist_pixel)

            index_aux = np.divide(d_real_wall, dist_pixel)
            index_H = 240 + index_aux #vector containing the 640 index of a line to take of the frame

            if angle_pitch < (-1*np.arctan((240*dist_pixel[320])/ d_min[320]) ):
                #minimum angle achieved
                index_H.fill(0)


        #roll rotation - rotation of axis x
        cos_ang_roll = np.dot(v_y, v_y_new )/np.dot( LA.norm(v_y), LA.norm(v_y_new))
        angle_roll = np.arccos(cos_ang_roll)
        sen_ang_roll = np.sin(angle_roll)

        if angle_roll != 0:
            #rotation of axis x
            opos_ang = 180-angle_roll
            cos_opos_roll = np.cos(opos_ang)
            sen_opos_roll = np.sin(opos_ang)

            for aux1 in range(320, 640):
                index_W[aux1+1] = np.floor(index_W[aux1]+cos_ang_roll+0.5)
                index_H[aux1+1] = np.floor(index_H[aux1]-sen_ang_roll+0.5)

            for aux2 in range(0, 320)
                aux3 = 320 - aux2
                index_W[aux3-1] = np.floor(index_W[aux3]-cos_opos_roll+0.5)
                index_H[aux3-1] = np.floor(index_H[aux3]-sen_opos_roll+0.5)

        line = self.frame[index_H, index_W]

        return line
    '''

    '''
    def self.observation_model(self, size_vector):

        h_v = np.zeros(size_vector, 1)
        middle = floor(size_vector/2)
        jacH = np.zeros(size_vector, 4)

        x_s = self.pred_state[0]
        y_s = self.pred_state[2]

        yaw = p.arccos( np.dot(v_z, self.rotation_matrix[:,2] )/np.dot( LA.norm(v_z), LA.norm(self.rotation_matrix[:,2])) )

        new_yaw = yaw +

        direction1 = new_yaw - (np.pi/2)
        direction2 = new_yaw + (np.pi/2)

        for i in range (middle, size_vector)
            x_m = floor( x_s + cos(new_yaw) + 0.5)
            y_m = floor( y_s - sin(new_yaw) + 0.5)

            count_pixels = 1
            distance_max = count_pixels * resolution
            #prediction of position point by the camera
            #Stops when find a obstacle or reachs the max range of camera (5 meters)
            while (map[x_m, y_m] != 1 and distance_max < 5 ):
                x_m = floor(x_m + cos(new_yaw) + 0.5)
                y_m = floor(y_m - sen(new_yaw) + 0.5)
                count_pixels += 1
                distance_max = count_pixels * resolution


            vector_dis = np.array([[x_s-x_m],[y_s -y_m]])
            h_v[i] = LA.norm(vector_dis)

            jacH[i,:] = self.jacobian(x_s, y_s, x_m, y_m)

            x_s = floor( x_s + cos(direction1) + 0.5)
            y_s = floor( y_s - sen(direction1) + 0.5)


        x_s = floor( self.pred_state[0] + cos(direction2) + 0.5)
        y_s = floor( self.pred_state[2] - sen(direction2) + 0.5)

        for j in range (middle-1, -1, -1)
            x_m = floor( x_s + cos(new_yaw) + 0.5)
            y_m = floor( y_s - sen(new_yaw) + 0.5)

            count_pixels = 1
            distance_max = count_pixels * resolution
            #prediction of position point by the camera
            #Stops when find a obstacle or reachs the max range of camera (5 meters)
            while (map[x_m, y_m] != 1 and distance_max < 5 ):
                x_m = floor(x_m + cos(new_yaw) + 0.5)
                y_m = floor(y_m - sen(new_yaw) + 0.5)
                count_pixels += 1
                distance_max = count_pixels * resolution


            vector_dis = np.array([[x_s-x_m],[y_s -y_m]])
            h_v(j) = LA.norm(vector_dis)

            jacH[j,:] = self.jacobian(x_s, y_s, x_m, y_m)

            x_s = floor( x_s + cos(direction2) + 0.5)
            y_s = floor( y_s - sen(direction2) + 0.5)

        self.matrix_H = jacH

        return h_v
    '''

    def jacobian(self, xs, ys, xp, yp):

        d_h = np.zeros(4)

        d_h[0] = (xs-xp)/sqrt((xs-xp)^2+(xs-xp)^2)
        d_h[1] = -(xs-xp)/sqrt((xs-xp)^2+(xs-xp)^2)
        d_h[2] = (ys-yp)/sqrt((xs-xp)^2+(xs-xp)^2)
        d_h[3] = -(ys-yp)/sqrt((xs-xp)^2+(xs-xp)^2)

        return h_


#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    #try:

        prog = EKF_localization()

        rospy.init_node('drone', anonymous=True)
        prog.robot_localization()



    #except rospy.RosInterruptException:
        pass
