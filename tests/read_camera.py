import numpy as np
from numpy import linalg as LA
from transforms3d import quaternions
import roslib
import sys
import rospy
from sensor_msgs.msg import Image, Imu, MagneticField
import time
import math
import cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/depth/image_raw',Image,self.save_image)
        self.c= 0


    def save_image(self, photo):

        cv_image = self.bridge.imgmsg_to_cv2(photo)
        print cv_image[240,100:600]
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)




def photo():
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    rospy.spin()

if __name__ == '__main__':
    photo()
