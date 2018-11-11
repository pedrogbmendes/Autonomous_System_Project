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
import roslib
import sys
import rospy
from sensor_msgs.msg import Image, Imu, CameraInfo, MagneticField
import smbus
import time
import struct
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


#address on the bus
bus = smbus.SMBus(1)
accel_address = 0x53 # accelerometer I2C address
gyro_address = 0x68  # gyroscope I2C address
magn_address = 0x1e  # magnetometer I2C address

#calibration of IMU
ACCEL_X_SCALE = 0.004
ACCEL_Y_SCALE = 0.004
ACCEL_Z_SCALE = 0.004
ACCEL_X_OFFSET = 0
ACCEL_Y_OFFSET = 0
ACCEL_Z_OFFSET = 0

GYRO_GAIN = 0.069565 # 1/14.375 : multiplication is faster than division
# GYRO_GAIN converts raw gyroscope data to deg/s values
# the gyroscope is calibrated every time the program starts, these
# variable are initialized here to make them global; the global tag is
# given in the functions where they are used
GYRO_AVERAGE_OFFSET_X = 0
GYRO_AVERAGE_OFFSET_Y = 0
GYRO_AVERAGE_OFFSET_Z = 0



#-----------------------------------------------------------------------------
#
#
#
#-----------------------------------------------------------------------------

class IMU_measures:

    def __init__(self):

        #initialization_IMU();
        self.IMU_data = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.accel_xyz = np.array([0, 0, 0])
        self.gyros_xyz = np.array([0, 0, 0])
        self.magn_xyz = np.array([0, 0, 0])

    def initialization_IMU():

    	global GYRO_AVERAGE_OFFSET_X
    	global GYRO_AVERAGE_OFFSET_Y
    	global GYRO_AVERAGE_OFFSET_Z

    	set_IMU()

    	# gyro must be stationary for correct calibration
    	gyro_offset = np.zeros(3)
    	for i in range(0,255):
    		gyro_result = get_gyro()
    		gyro_offset[0] += gyro_result[0]
    		gyro_offset[1] += gyro_result[1]
    		gyro_offset[2] += gyro_result[2]

    	GYRO_AVERAGE_OFFSET_X = gyro_offset[0] * 0.00390625 # 1/256
    	GYRO_AVERAGE_OFFSET_Y = gyro_offset[1] * 0.00390625 # 1/256
    	GYRO_AVERAGE_OFFSET_Z = gyro_offset[2] * 0.00390625 # 1/256


    def set_IMU():

    	bus.write_byte_data(accel_address, 0x2c, 0x0b) # BW_RATE 100Hz
    	bus.write_byte_data(accel_address, 0x31, 0x00) # DATA_FORMAT +-2g 10-bit resolution
    	bus.write_byte_data(accel_address, 0x2d, 0x08) # Power Control Register measurement mode

    	bus.write_byte_data(gyro_address, 0x15, 0x07) # SMPLRT_DIV - 125Hz (output sample rate)
    	bus.write_byte_data(gyro_address, 0x16, 0x1a) # DLPF_FS - +-2000deg/s ; # DLPF_CFG - low pass 98Hz, internal sample rate 1kHz

        bus.write_byte_data(magn_address, 0x02, 0x00) # MODE continuous - 15Hz default
        bus.write_byte_data(magn_address, 0x00, 0x18) # Config_REG_A - Output rate 75Hz


    def get_accel():

    	accel = np.empty([3])
    	accel_x = bytearray()
    	accel_y = bytearray()
    	accel_z = bytearray()

    	accel_x.append(bus.read_byte_data(accel_address, 0x33))
    	accel_x.append(bus.read_byte_data(accel_address, 0x32))
    	accel[0] = struct.unpack('>h',bytes(accel_x))[0]

    	accel_y.append(bus.read_byte_data(accel_address, 0x35))
    	accel_y.append(bus.read_byte_data(accel_address, 0x34))
    	accel[1] = struct.unpack('>h',bytes(accel_y))[0]

    	accel_z.append(bus.read_byte_data(accel_address, 0x37))
    	accel_z.append(bus.read_byte_data(accel_address, 0x36))
    	accel[2] = struct.unpack('>h',bytes(accel_z))[0]

    	return accel

    def get_gyro():

    	gyro = np.empty([3])
    	gyro_x = bytearray()
    	gyro_y = bytearray()
    	gyro_z = bytearray()

    	gyro_x.append(bus.read_byte_data(gyro_address, 0x1d)) # GYRO_XOUT_H
    	gyro_x.append(bus.read_byte_data(gyro_address, 0x1e)) # GYRO_XOUT_L
    	gyro[0] = struct.unpack('>h',bytes(gyro_x))[0]

    	gyro_y.append(bus.read_byte_data(gyro_address, 0x1f)) # GYRO_YOUT_H
    	gyro_y.append(bus.read_byte_data(gyro_address, 0x20)) # GYRO_YOUT_L
    	gyro[1] = struct.unpack('>h',bytes(gyro_y))[0]

    	gyro_z.append(bus.read_byte_data(gyro_address, 0x21)) # GYRO_ZOUT_H
    	gyro_z.append(bus.read_byte_data(gyro_address, 0x22)) # GYRO_ZOUT_L
    	gyro[2] = struct.unpack('>h',bytes(gyro_z))[0]

    	return gyro

    def get_magn():

    	magn = np.empty([3])
    	magn_x = bytearray()
    	magn_y = bytearray()
    	magn_z = bytearray()

    	magn_x.append(bus.read_byte_data(magn_address, 0x03))
    	magn_x.append(bus.read_byte_data(magn_address, 0x04))
    	magn[0] = struct.unpack('>h',bytes(magn_x))[0]

    	magn_y.append(bus.read_byte_data(magn_address, 0x05))
    	magn_y.append(bus.read_byte_data(magn_address, 0x06))
    	magn[1] = struct.unpack('>h',bytes(magn_y))[0]

    	magn_z.append(bus.read_byte_data(magn_address, 0x07))
    	magn_z.append(bus.read_byte_data(magn_address, 0x08))
    	magn[2] = struct.unpack('>h',bytes(magn_z))[0]

    	return magn

    def compensate_sensor_errors():

    	self.accel_xyz[0] = (accel_xyz[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE
    	self.accel_xyz[1] = (accel_xyz[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE
        self.accel_xyz[2] = (accel_xyz[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE

    	self.gyro_xyz[0] = (gyro_xyz[0] - GYRO_AVERAGE_OFFSET_X) * GYRO_GAIN
    	self.gyro_xyz[1] = (gyro_xyz[1] - GYRO_AVERAGE_OFFSET_Y) * GYRO_GAIN
    	self.gyro_xyz[2] = (gyro_xyz[2] - GYRO_AVERAGE_OFFSET_Z) * GYRO_GAIN



    def IMU_read():

    	self.accel_xyz = get_accel()
    	self.gyro_xyz = get_gyro()
    	self.magn_xyz = get_magn()

    def IMU_Data():

		IMU_read()
		compensate_sensor_errors()
		self.IMU_data[0,:] = accel_xyz
		self.IMU_data[1,:] = gyro_xyz
		self.IMU_data[2,:] = magn_xyz







#-----------------------------------------------------------------------------
#
#
#
#-----------------------------------------------------------------------------

class drone_measures:

    def __init__(self):

        #frame
		self.image_sub = rospy.Subscriber('/camera/depth/image',Image,self.save_image)
		self.image_pub = rospy.Publisher('/image', Image, queue_size = 10)

        #camera information
		self.info_sub = rospy.Subscriber('/camera/depth/camera_info',CameraInfo,self.save_info)
		self.info_pub = rospy.Publisher('/camera_info',CameraInfo, queue_size=10)

        #IMU data (angular velocities and linear accelerations)
        self.IMU_data = rospy.Subscriber('/imu/data_raw', Imu, self.IMU_readData)
        #Magnetic field data
        self.IMU_data = rospy.Subscriber('/imu/mag', MagneticField, self.IMU_readMag)
        #publish the quaternions
        self.IMU_data = rospy.Publisher('/data', Imu, queue_size = 10)


        rospy.spin()


    def save_image(self, data):
    	return data

    def save_info(self, data_info):
        return data_info

    def IMU_readData(self, dataIMU):
        return dataIMU

    def IMU_readMag(self, data_mag):
        return data_mag





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

        rotation_matrix = read_IMU()
        z = take_frame_line(rotation_matrix)


        self.act_state = self.pred_state + k.dot(z-h)
        self.act_cov = (I - k.dot(self.matrix_H)).dot(self.pred_cov)



    def take_frame_line(rotation_matrix):

        frame = take_image() #matrix with 480 rows and 640 collumns
        #W = 640;  H = 480;

        index_W = np.arange(640)
        index_H = np.repeat(240,480)

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
            index_H = 240 - index_aux #vector containing the 640 index of a line to take of the frame

            if angle_pitch > np.arctan((240*dist_pixel[320])/ d_min[320]):
                #maximum angle achieved
                index_H.fill(0)

        elif angle_pitch < 0
            d_min = frame[240] #vector with 640 index
            d_next = frame[241]
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


        line = frame[index_H, index_W]

        return line




    def take_image():

        matrix_frame = robot_camera.save_image()

        return matrix_frame

    def read_IMU():

        quaternions = robot_camera.
        rotation_matrix = quaternions.quat2mat(quaternions)

        return rotation_matrix






#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rospy.init_node('drone', anonymous=True)
        robot_IMU = IMU_measures()
        robot_IMU.initialization_IMU()
        robot_camera = drone_measures()
        prog = EKF_localization()
    except rospy.RosInterruptException:
        pass
