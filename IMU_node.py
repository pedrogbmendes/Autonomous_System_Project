# coding=utf-8
#-----------------------------------------------------------------------------
#
#                           Autonomous Systems
#
#       Project: Extended Kalman Filter (EKF) localization using a drone with
#   a depth camera and a IMU
#
#       IMU node: read IMU data and publish then
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
import roslib
import sys
import rospy
from sensor_msgs.msg import Imu, MagneticField
import smbus
import time
import struct
import math


#-----------------------------------------------------------------------------I
#
#   Global constants
#
#-----------------------------------------------------------------------------
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

# Manual magnetometer calibration
#MAGN_X_MAX = 430
#MAGN_X_MIN = -415
#MAGN_Y_MAX = 520
#MAGN_Y_MIN = -530
#MAGN_Z_MAX = 355
#MAGN_Z_MIN = -431
#MAGN_X_OFFSET = (MAGN_X_MIN + MAGN_X_MAX) / 2.
#MAGN_Y_OFFSET = (MAGN_Y_MIN + MAGN_Y_MAX) / 2.
#MAGN_Z_OFFSET = (MAGN_Z_MIN + MAGN_Z_MAX) / 2.
#MAGN_X_SCALE = 100. / (MAGN_X_MAX - MAGN_X_OFFSET)
#MAGN_Y_SCALE = 100. / (MAGN_Y_MAX - MAGN_Y_OFFSET)
#MAGN_Z_SCALE = 100. / (MAGN_Z_MAX - MAGN_Z_OFFSET)



#-----------------------------------------------------------------------------
#
#   IMU_measures read the value of the bus provenient from the IMU
#
#-----------------------------------------------------------------------------

class IMU_measures:

    def __init__(self):
        self.accel_xyz = np.array([0.0, 0.0, 0.0])
        self.gyro_xyz = np.array([0.0, 0.0, 0.0])
        self.magn_xyz = np.array([0.0, 0.0, 0.0])

        self.initialization_IMU()

        #node of the IMU sensor to publish the IMU's data
        self.Imu_data_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size = 10)
        self.Imu_data_mag = rospy.Publisher('/imu/mag', MagneticField, queue_size = 10)

    def initialization_IMU(self):

    	global GYRO_AVERAGE_OFFSET_X
    	global GYRO_AVERAGE_OFFSET_Y
    	global GYRO_AVERAGE_OFFSET_Z

    	self.set_IMU()

    	# gyro must be stationary for correct calibration
    	gyro_offset = np.zeros(3)
    	for i in range(0,255):
    		gyro_result = self.get_gyro()
    		gyro_offset[0] += gyro_result[0]
    		gyro_offset[1] += gyro_result[1]
    		gyro_offset[2] += gyro_result[2]

    	GYRO_AVERAGE_OFFSET_X = gyro_offset[0] * 0.00390625 # 1/256
    	GYRO_AVERAGE_OFFSET_Y = gyro_offset[1] * 0.00390625 # 1/256
    	GYRO_AVERAGE_OFFSET_Z = gyro_offset[2] * 0.00390625 # 1/256


    def set_IMU(self):

    	bus.write_byte_data(accel_address, 0x2c, 0x0b) # BW_RATE 100Hz
    	bus.write_byte_data(accel_address, 0x31, 0x00) # DATA_FORMAT +-2g 10-bit resolution
    	bus.write_byte_data(accel_address, 0x2d, 0x08) # Power Control Register measurement mode

    	bus.write_byte_data(gyro_address, 0x15, 0x07) # SMPLRT_DIV - 125Hz (output sample rate)
    	bus.write_byte_data(gyro_address, 0x16, 0x1a) # DLPF_FS - +-2000deg/s ; # DLPF_CFG - low pass 98Hz, internal sample rate 1kHz

        bus.write_byte_data(magn_address, 0x02, 0x00) # MODE continuous - 15Hz default
        bus.write_byte_data(magn_address, 0x00, 0x18) # Config_REG_A - Output rate 75Hz



    def get_accel(self):

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


    def get_gyro(self):

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

    def get_magn(self):

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

    def compensate_sensor_errors(self):

    	self.accel_xyz[0] = (self.accel_xyz[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE
    	self.accel_xyz[1] = (self.accel_xyz[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE
        self.accel_xyz[2] = (self.accel_xyz[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE

    	self.gyro_xyz[0] = (self.gyro_xyz[0] - GYRO_AVERAGE_OFFSET_X) * GYRO_GAIN
    	self.gyro_xyz[1] = (self.gyro_xyz[1] - GYRO_AVERAGE_OFFSET_Y) * GYRO_GAIN
    	self.gyro_xyz[2] = (self.gyro_xyz[2] - GYRO_AVERAGE_OFFSET_Z) * GYRO_GAIN

        #convert from deg/s to rad/s (pi/180 = 0.0174532925)
        self.gyro_xyz[0] = self.gyro_xyz[0] * 0.0174532925
        self.gyro_xyz[1] = self.gyro_xyz[0] * 0.0174532925
        self.gyro_xyz[2] = self.gyro_xyz[0] * 0.0174532925


        #self.magn_xyz[0] = (self.magn_xyz[0] - MAGN_X_OFFSET) * MAGN_X_SCALE
        #self.magn_xyz[1] = (self.magn_xyz[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE
        #self.magn_xyz[2] = (self.magn_xyz[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE



    def IMU_read(self):

    	self.accel_xyz = self.get_accel()
    	self.gyro_xyz = self.get_gyro()
        self.magn_xyz = self.get_magn()
        self.compensate_sensor_errors()

        self.publish_data()


    def publish_data(self):

        data_pub = Imu()
        data_pub_mag = MagneticField()

        data_pub.angular_velocity.x = self.gyro_xyz[0] #gyros_x
        data_pub.angular_velocity.y = self.gyro_xyz[1] #gyros_y
        data_pub.angular_velocity.z = self.gyro_xyz[2] #gyros_z

        data_pub.angular_velocity_covariance[0] = 0

        data_pub.linear_acceleration.x = self.accel_xyz[0] #accel_x
        data_pub.linear_acceleration.y = self.accel_xyz[1] #accel_y
        data_pub.linear_acceleration.z = self.accel_xyz[2] #accel_z

        data_pub.linear_acceleration_covariance[0] = 0

        data_pub_mag.magnetic_field.x = self.magn_xyz[0] #accel_x
        data_pub_mag.magnetic_field.y = self.magn_xyz[1] #accel_y
        data_pub_mag.magnetic_field.z = self.magn_xyz[2] #accel_z

        data_pub_mag.magnetic_field_covariance[0] = 0 #variance unknown

        self.Imu_data_pub.publish(data_pub)
        self.Imu_data_mag.publish(data_pub_mag)



#Initialization of the class
Imu_sensor = IMU_measures()



def work_IMU():
    while not rospy.is_shutdown():
        Imu_sensor.IMU_read()


#-----------------------------------------------------------------------------
#
#   __main__
#
#-----------------------------------------------------------------------------
if __name__ == '__main__':
    #try:
        rospy.init_node('imu', anonymous=True)
        rate = rospy.Rate(10)
        work_IMU()

    #except rospy.RosInterruptException:
    #    pass
