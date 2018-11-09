
import roslib
import sys
import rospy
from sensor_msgs.msg import Imu, MagneticField


class take_dataIMU:

	def __init__(self):

        #IMU data (angular velocities and linear accelerations)
		self.IMU_data = rospy.Subscriber('/imu/data_raw', Imu, self.IMU_readData)
        #Magnetic field data
		self.IMU_mag = rospy.Subscriber('/imu/mag',MagneticField,self.IMU_readMag)
        #publish the quaternions
		self.IMU_quaternions = rospy.Publisher('/data', Imu, queue_size = 10)
		rospy.spin()

   	def IMU_readData(self, photo):
		return

	def IMU_readMag(self, data):
		return


def data_imu():

	rospy.init_node('drone-imu', anonymous=True)
	im = take_dataIMU()


if __name__ == '__main__':
	try:
		data_imu()
	except rospy.RosInterruptException:
		pass
