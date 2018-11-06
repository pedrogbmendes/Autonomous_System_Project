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

        self.act_state = self.pred_state + k.dot(z-h)
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov)













#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        depthcam();
    except rospy.RosInterruptException:
        pass
