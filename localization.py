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


import numpy as np
from numpy import linalg as LA
from transforms3d import quaternions
import roslib
import sys
import rospy
from sensor_msgs.msg import Image, Imu
import time
import math


#-----------------------------------------------------------------------------I
#
#   Global constants
#
#-----------------------------------------------------------------------------
I = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
cov_x = 1
cov_y = 1
matrix_R = np.array([[0,0,0,0],[0,cov_x,0,0],[0,0,0,0],[0,0,0,cov_y]])
covq_x = 1
covq_y = 1
matrix_Q = np.array([[0,0,0,0],[0,covq_x,0,0],[0,0,0,0],[0,0,0,covq_y]])

#Camera coordenate frames vectors
v_x = np.array([1,0,0])
v_y = np.array([0,1,0])
v_z = np.array([0,0,1])


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
        self.prev_state = np.array([0,0,0,0])
        #actual state
        self.act_state = np.array([0,0,0,0])
        #predicted state
        self.pred_state = np.array([0,0,0,0])
        #motion model
        self.matrix_A = np.array([[1,self.delta_time,0,0],[0,1,0,0],[0,0,1,self.delta_time],[0,0,0,1]])
        #covariance of instance t-1
        self.prev_cov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        #covariance of instance t
        self.act_cov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        #predicted covariance
        self.pred_cov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]))
        #Jacobian matrix
        self.matrix_H = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        #rotation matrix
        self.rotation_matrix = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        #frame
        self.frame = np.zeros((480,640))#size of the image taken by the camera
        #line fo frame
        self.line_z = 0

    def robot_localization():

        self.predition_step()

        #node of drone to Subscribe IMU data
        self.subsIMU = rospy.Subscriber('/imu/data_raw',Imu,self.sub_pub_calRot)
        #and to to Subscribe camera data
		self.image_sub = rospy.Subscriber('/camera/depth/image',Image,self.save_image)
        rate = rospy.Rate(10)

        rospy.spin()


    def predition_step(self):

        self.prev_time = self.act_time
        self.act_time = rospy.Time.now()
        self.delta_time = self.act_time - self.prev_time

        self.matrix_A[0][1] = self.delta_time
        self.matrix_A[2][3] = self.delta_time

        self.pred_state = self.matrix_A.dot(self.prev_state)
        self.pred_cov = ((self.matrix_A.dot(self.prev_cov)).dot(self.matrix_A.transpose())) + matrix_R


    def update_step(self):

        #Kalman gain
        k = (self.pred_cov.dot(self.matrix_H.transpose())).dot(inv((self.matrix_H.dot(self.pred_cov)).dot(self.matrix_H.transpose()) + matrix_Q))

        self.act_state = self.pred_state + k.dot(self.line_z - h)
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov)


    def sub_pub_calRot(self, data):
        self.rotation_matrix = quaternions.quat2mat(data)


    def save_image(self, photo):
        self.frame = photo
        self.line_z = self.take_frame_line()
        self.update_step()


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








#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    try:

        prog = EKF_localization()

        rospy.init_node('drone', anonymous=True)
        prog.robot_localization()



    except rospy.RosInterruptException:
        pass
