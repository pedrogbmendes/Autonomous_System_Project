

import roslib
import sys
import rospy
from sensor_msgs.msg import Imu, MagneticField
import smbus
import time
import struct
import math
import numpy as np
from std_msgs.msg import Float32

#GLOBAL VARIABLES

#address on the bus
bus = smbus.SMBus(1)
accel_address = 0x53 # accelerometer I2C address
gyro_address = 0x68  # gyroscope I2C address
magn_address = 0x1e  # magnetometer I2C address

IMU_data = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
accel_xyz = np.array([0.0, 0.0, 0.0])
gyros_xyz = np.array([0.0, 0.0, 0.0])
magn_xyz = np.array([0.0, 0.0, 0.0])

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

class take_dataIMU:

	def __init__(self):

		#self.IMU_data = rospy.Publisher('/imu/data_raw', Imu, queue_size = 10)
		self.IMU_data_angx = rospy.Publisher('/imu/angularx', Float32, queue_size = 10)
		self.IMU_data_linx = rospy.Publisher('/imu/linearx', Float32, queue_size = 10)

    #self.IMU_data = rospy.Publisher('/imu/data_raw', Imu, queue_size = 10)
        self.IMU_data_angy = rospy.Publisher('/imu/angulary', Float32, queue_size = 10)
        self.IMU_data_liny = rospy.Publisher('/imu/lineary', Float32, queue_size = 10)


    #self.IMU_data = rospy.Publisher('/imu/data_raw', Imu, queue_size = 10)
                self.IMU_data_angz = rospy.Publisher('/imu/angularz', Float32, queue_size = 10)
                self.IMU_data_linz = rospy.Publisher('/imu/linearz', Float32, queue_size = 10)

		rate = rospy.Rate(10)#10hz

		while True:
			self.read_IMU()
			self.compensate_sensor_errors()
			self.IMU_readData()
       #IMU data (angular velocities and linear accelerations)

        #Magnetic field data
		#self.IMU_mag = rospy.Subscriber('/imu/mag',MagneticField,self.IMU_readMag)
        #publish the quaternions
		#self.IMU_quaternions = rospy.Publisher('/data', Imu, queue_size = 10)



   	def IMU_readData(self):
		# Header header
		#
		# geometry_msgs/Quaternion orientation
		# float64[9] orientation_covariance # Row major about x, y, z axes
		#
		# geometry_msgs/Vector3 angular_velocity
		# float64[9] angular_velocity_covariance # Row major about x, y, z axes
		#
		# geometry_msgs/Vector3 linear_acceleration
		# float64[9] linear_acceleration_covariance # Row major x, y z
		#data_pub = Imu()

		#data_pub.orientation = 0
		#data_pub.orientation_covariance[0] = -1

		#data_pub.angular_velocity.x = IMU_data[1,0] #gyros_xyz
		#data_pub.angular_velocity.y = IMU_data[1,1] #gyros_xyz
		#data_pub.angular_velocity.z = IMU_data[1,2] #gyros_xyz

		#data_pub.angular_velocity_covariance[0] = -1

		#data_pub.linear_acceleration.x = IMU_data[0,0] #accel_xyz
		#data_pub.linear_acceleration.y = IMU_data[0,1] #accel_xyz
		#data_pub.linear_acceleration.z = IMU_data[0,2] #accel_xyz

		#data_pub.linear_acceleration_covariance[0] = -1

		#self.data_pub.header.stamp = rospy.Time.now()
		#self.data_pub.header.frame_id =
	#self.data_pub.header.seq = self.seq

                self.IMU_data_angx.publish(IMU_data[1, 0])
                self.IMU_data_linx.publish(IMU_data[0, 0])

                self.IMU_data_angy.publish(IMU_data[1, 1])
                self.IMU_data_liny.publish(IMU_data[0, 1])

		self.IMU_data_angz.publish(IMU_data[1, 2])
		self.IMU_data_linz.publish(IMU_data[0, 2])
		#self.seq += 1




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

		global accel_xyz
		global gyro_xyz
		global magn_xyz

		accel_xyz[0] = (accel_xyz[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE
		accel_xyz[1] = (accel_xyz[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE
		accel_xyz[2] = (accel_xyz[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE

		gyro_xyz[0] = (gyro_xyz[0] - GYRO_AVERAGE_OFFSET_X) * GYRO_GAIN
		gyro_xyz[1] = (gyro_xyz[1] - GYRO_AVERAGE_OFFSET_Y) * GYRO_GAIN
		gyro_xyz[2] = (gyro_xyz[2] - GYRO_AVERAGE_OFFSET_Z) * GYRO_GAIN


		IMU_data[1,:] = gyro_xyz
		IMU_data[2,:] = magn_xyz

	def read_IMU(self):

		global accel_xyz
		global gyro_xyz
		global magn_xyz
		global IMU_data

		accel_xyz = self.get_accel()
		gyro_xyz = self.get_gyro()
		magn_xyz = self.get_magn()





def initialization_IMU():

	global GYRO_AVERAGE_OFFSET_X
	global GYRO_AVERAGE_OFFSET_Y
	global GYRO_AVERAGE_OFFSET_Z

	set_IMU()

	# gyro must be stationary for correct calibration
	gyro_offset = np.zeros(3)
	for i in range(0,255):
		gyro_result = get_gyro_in()
		gyro_offset[0] += gyro_result[0]
		gyro_offset[1] += gyro_result[1]
		gyro_offset[2] += gyro_result[2]

	GYRO_AVERAGE_OFFSET_X = gyro_offset[0] * 0.00390625 # 1/256
	GYRO_AVERAGE_OFFSET_Y = gyro_offset[1] * 0.00390625 # 1/256
	GYRO_AVERAGE_OFFSET_Z = gyro_offset[2] * 0.00390625 # 1/256

def get_gyro_in():

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

def set_IMU():

	bus.write_byte_data(accel_address, 0x2c, 0x0b) # BW_RATE 100Hz
	bus.write_byte_data(accel_address, 0x31, 0x00) # DATA_FORMAT +-2g 10-bit resolution
	bus.write_byte_data(accel_address, 0x2d, 0x08) # Power Control Register measurement mode

	bus.write_byte_data(gyro_address, 0x15, 0x07) # SMPLRT_DIV - 125Hz (output sample rate)
	bus.write_byte_data(gyro_address, 0x16, 0x1a) # DLPF_FS - +-2000deg/s ; # DLPF_CFG - low pass 98Hz, internal sample rate 1kHz

	bus.write_byte_data(magn_address, 0x02, 0x00) # MODE continuous - 15Hz default
	bus.write_byte_data(magn_address, 0x00, 0x18) # Config_REG_A - Output rate 75Hz


def data_imu():
	rospy.init_node('droneimu', anonymous=True)
	initialization_IMU()
	robot = take_dataIMU()



if __name__ == '__main__':
	data_imu()
#	try:
#		data_imu()
#	except rospy.RosInterruptException:
#		pass
