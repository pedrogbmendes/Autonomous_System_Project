

import roslib
#roslib.load_manifest('store_stereo_image')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class take_image:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/detph/images',Image,self.save_image)

    def save_image(self, photo):
        W = 640;
        H = 480;

        try:
            cv_image.SaveImage("depth_camera_msg.jpg", cv_image)
            print "image saved!"
        except CvBridgeError, e:
            print e




def depthcam():

    im = take_image()

    rospy.init_node('depthcam', anonymous=True)





if __name__ == '__main__':
    try:
        depthcam();
    except rospy.RosInterruptException:
        pass
