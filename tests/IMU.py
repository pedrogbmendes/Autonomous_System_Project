# coding=utf-8
#--------------------------------------------------------------------$
#
#                           Autonomous Systems
#
#       Project: Extended Kalman Filter (EKF) localization using a dr$
#   a depth camera and a IMU
#
#       IMU node: read IMU data and publish then
#
#   Authors:
#       - Pedro Gonçalo Mendes, 81046, pedrogoncalomendes@tecnico.ul$
#       - Miguel Matos Malaca, 81702, miguelmmalaca@tecnico.ulisboa.pt
#       - João José Rosa, 84089, joao.c.rosa@tecnico.ulisboa.pt
#       - João Pedro Ferreira, 78101, joao.pedro.ferreira@tecnico.ul$
#
#                         1st semestre, 2018/19
#                       Instítuto Superior Técnico
#
#--------------------------------------------------------------------$


import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Imu, MagneticField
import smbus
import time
import struct
import math



bus = smbus.SMBus(1)
accel_address = 0x53 # accelerometer I2C address
gyro_address = 0x68  # gyroscope I2C address
magn_address = 0x1e  # magnetometer I2C address

accel_xyz = np.empty([3])
gyro_xyz = np.empty([3])
magn_xyz = np.empty([3])

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


def set_accel():

    bus.write_byte_data(accel_address, 0x2c, 0x0b) # BW_RATE 100Hz
    bus.write_byte_data(accel_address, 0x31, 0x00) # DATA_FORMAT +-2g 10-bit resolution
    bus.write_byte_data(accel_address, 0x2d, 0x08) # Power Control Register measurement mode

def set_gyro():

    bus.write_byte_data(gyro_address, 0x15, 0x07) # SMPLRT_DIV - 125Hz (output sample rate)
    #bus.write_byte_data(gyro_address, 0x16, 0x1d) # DLPF_FS - +-2000deg/s ; # DLPF_CFG - low pass 10Hz, internal sample rate 1kHz
    bus.write_byte_data(gyro_address, 0x16, 0x1a) # DLPF_FS - +-2000deg/s ; # DLPF_CFG - low pass 98Hz, internal sample rate 1kHz

def set_magn():

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

    global accel_xyz
    global gyro_xyz
    global magn_xyz
    #global heading_rad
    #global heading_deg

    accel_xyz[0] = (accel_xyz[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE
    accel_xyz[1] = (accel_xyz[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE
    accel_xyz[2] = (accel_xyz[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE

    gyro_xyz[0] = (gyro_xyz[0] - GYRO_AVERAGE_OFFSET_X) * GYRO_GAIN
    gyro_xyz[1] = (gyro_xyz[1] - GYRO_AVERAGE_OFFSET_Y) * GYRO_GAIN
    gyro_xyz[2] = (gyro_xyz[2] - GYRO_AVERAGE_OFFSET_Z) * GYRO_GAIN




def publish_data():

    data_pub = Imu()
    data_pub_mag = MagneticField()

    data_pub.header.stamp = rospy.get_rostime()
    data_pub_mag.header.stamp = rospy.get_rostime()

    #convert from deg/s to rad/s (pi/180 = 0.0174532925
    data_pub.angular_velocity.x = gyro_xyz[0]* 0.0174532925 #gyros_x
    data_pub.angular_velocity.y = gyro_xyz[1]* 0.0174532925  #gyros_y
    data_pub.angular_velocity.z = gyro_xyz[2]* 0.0174532925 #gyros_z

    data_pub.angular_velocity_covariance[0] = 0

    #convert from g to m/s^2
    data_pub.linear_acceleration.x = accel_xyz[0]*9.81
    data_pub.linear_acceleration.y = accel_xyz[1]*9.81
    data_pub.linear_acceleration.z = accel_xyz[2]*9.81

    data_pub.linear_acceleration_covariance[0] = 0

    #convert from Guass to Tesla
    data_pub_mag.magnetic_field.x = magn_xyz[0]*0.0001 #accel_x
    data_pub_mag.magnetic_field.y = magn_xyz[1]*0.0001 #accel_y
    data_pub_mag.magnetic_field.z = magn_xyz[2]*0.0001 #accel_z

    data_pub_mag.magnetic_field_covariance[0] = 0 #variance unkno$


    print("accel = [%5.2f,%5.2f,%5.2f]m/s^2" % (data_pub.linear_acceleration.x, data_pub.linear_acceleration.y, data_pub.linear_acceleration.z))
    print("gyro = [%6.1f,%6.1f,%6.1f]rad/s" % (data_pub.angular_velocity.x,data_pub.angular_velocity.y, data_pub.angular_velocity.z))
    print("magn = [%6.1f,%6.1f,%6.1f]T\n" % (data_pub_mag.magnetic_field.x, data_pub_mag.magnetic_field.y, data_pub_mag.magnetic_field.z))

    Imu_data_pub.publish(data_pub)
    Imu_data_mag.publish(data_pub_mag)



def set_things_up():

    global GYRO_AVERAGE_OFFSET_X
    global GYRO_AVERAGE_OFFSET_Y
    global GYRO_AVERAGE_OFFSET_Z

    set_accel()
    set_gyro()
    set_magn()

    # gyro must be stationary for correct calibration
    gyro_offset = np.zeros(3)
    #for i in range(0,63):
    for i in range(0,255):
        gyro_result = get_gyro()
        gyro_offset[0] += gyro_result[0]
        gyro_offset[1] += gyro_result[1]
        gyro_offset[2] += gyro_result[2]

    #GYRO_AVERAGE_OFFSET_X = gyro_offset[0] * 0.015625 # 1/64
    #GYRO_AVERAGE_OFFSET_Y = gyro_offset[1] * 0.015625 # 1/64
    #GYRO_AVERAGE_OFFSET_Z = gyro_offset[2] * 0.015625 # 1/64
    GYRO_AVERAGE_OFFSET_X = gyro_offset[0] * 0.00390625 # 1/256
    GYRO_AVERAGE_OFFSET_Y = gyro_offset[1] * 0.00390625 # 1/256
    GYRO_AVERAGE_OFFSET_Z = gyro_offset[2] * 0.00390625 # 1/256


def read_sensors():
    global accel_xyz
    global gyro_xyz
    global magn_xyz
    accel_xyz = get_accel()
    gyro_xyz = get_gyro()
    magn_xyz = get_magn()






rospy.init_node('imu', anonymous=True)
rate = rospy.Rate(10)

Imu_data_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size = 10)
Imu_data_mag = rospy.Publisher('/imu/mag', MagneticField, queue_size = 10)


set_things_up()

while not rospy.is_shutdown():
    read_sensors()
    compensate_sensor_errors()
    publish_data()
