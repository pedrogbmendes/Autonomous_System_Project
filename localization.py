#-----------------------------------------------------------------------------
#
#                           Autonomous Systems
#
#       Project: Extended Kalman Filter (EKF) localization using a drone with
#   a depth camera and a IMU
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

#-----------------------------------------------------------------------------
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
#
#
#-----------------------------------------------------------------------------

class drone_measures:





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


    def predition_step(self):

        self.prev_time = self.act_time
        self.act_time = rostime now # TODO:
        self.delta_time = self.act_time - self.prev_time

        self.matrix_A[0][1] = self.delta_time
        self.matrix_A[2][3] = self.delta_time

        self.pred_state = self.matrix_A.dot(self.prev_state)
        self.pred_cov = ((self.matrix_A.dot(self.prev_cov)).dot(self.matrix_A.transpose())) + matrix_R



    def update_step(self):

        #Kalman gain
        k = (self.pred_cov.dot(self.matrix_H.transpose())).dot(inv((self.matrix_H.dot(self.pred_cov)).dot(self.matrix_H.transpose()) + matrix_Q))

        z = take_frame_line()


        self.act_state = self.pred_state + k.dot(z-h)
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov)



    def take_frame_line():

        frame = take_image()
        rotation_matrix = read_IMU()

        v_x_new = rotation_matrix[:,0]
        v_y_new = rotation_matrix[:,1]
        v_z_new = rotation_matrix[:,2]

        #pitch rotation - rotation of axis y
        angle_pitch = np.arccos( np.dot(v_x, v_x_new )/np.dot( LA.norm(v_x), LA.norm(v_x_new)) )
        if angle_pitch > 0 :
            d_min = frame[240] #vector with 640 index
            d_next = frame[239]
            d_real_wall = d_min * tan(angle_pitch); #real distance between to points
            dist_pixel = np.sqrt(np.square(d_next)-np.square(d_min))
            if dist_pixel < 0
                dist_pixel = np.absolute(dist_pixel)

            index_aux = np.divide(d_real_wall, dist_pixel)
            index = 240 - index_aux #vector containing the 640 index of a line to take of the frame

            if angle_pitch > np.arctan((240*dist_pixel[320])/ d_min[320]):
                #maximum angle achieved
                index = 0

        elif angle_pitch < 0
            d_min = frame[240] #vector with 640 index
            d_next = frame[241]
            d_real_wall = d_min * tan(-angle_pitch); #real distance between to points
            dist_pixel = np.sqrt(np.square(d_next)-np.square(d_min))
            if dist_pixel < 0
                dist_pixel = np.absolute(dist_pixel)

            index_aux = np.divide(d_real_wall, dist_pixel)
            index = 240 + index_aux #vector containing the 640 index of a line to take of the frame

            if angle_pitch < (-1*np.arctan((240*dist_pixel[320])/ d_min[320]) ):
                #minimum angle achieved
                index = 0
        else:
            #no rotation on axis y



        #roll rotation - rotation of axis x
        angle_roll = np.arccos( np.dot(v_y, v_y_new )/np.dot( LA.norm(v_y), LA.norm(v_y_new)) )
        if angle_roll > 0 :


    def take_image():

        matrix_frame = drone_measures().

        return matrix_frame

    def read_IMU():

        quaternions = drone_measures().

        rotation_matrix = quaternions.quat2mat(quaternions)

        return rotation_matrix






#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        robot = drone_measures()
        prog = EKF_localization()
    except rospy.RosInterruptException:
        pass
