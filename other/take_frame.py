

import roslib
#roslib.load_manifest('store_stereo_image')
import sys
import rospy
#import cv
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class take_image:

	def __init__(self):
        
		self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.save_scan)
		self.image_sub = rospy.Subscriber('/camera/depth/image',Image,self.save_image)
		self.image_pub = rospy.Publisher('/image', Image, queue_size = 10)
		self.info_sub = rospy.Subscriber('/camera/depth/camera_info',CameraInfo,self.save_info)
		self.info_pub = rospy.Publisher('/camera_info',CameraInfo, queue_size=10)

		rospy.spin()

   	def save_scan(self, photo):
#         W = 640;
#         H = 480;
		return

	def save_image(self, data):
		self.image_pub.publish(data)

	def save_info(self, data):
		self.info_pub.publish(data)




def depthcam():

	rospy.init_node('depthcam', anonymous=True)
	im = take_image()

    
    





if __name__ == '__main__':
    
	depthcam()
    # try:
    #     depthcam();
    # except rospy.RosInterruptException:
    # 	pass