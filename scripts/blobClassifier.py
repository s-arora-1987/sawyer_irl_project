#!/usr/bin/env python
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
from cv_bridge import CvBridge

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

# Blob specific headers
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped, PointStamped
from cmvision.msg import Blobs, Blob
# from cmvision_3d.msg import Blob3d



VERBOSE=False

class bClassifier:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage,  queue_size = 1)
        self.bridge = CvBridge()

        self.subscriber1 = rospy.Subscriber('blobs', Blobs, self.blob_callback,  queue_size=1)
        # subscribed Topic
        self.subscriber2 = rospy.Subscriber("/kinect_V2/rgb/image_raw/compressed",
            CompressedImage, self.imagecallback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /kinect_V2/rgb/image_raw/compressed"


    def imagecallback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0

        # convert np image to grayscale
        grayImg = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)

        # cv2.imshow('gray_img', grayImg)
        # cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.grayImage = grayImg
        # Publish new image
        self.image_pub.publish(msg)

    def blob_callback(self, Blobs):
        
        pass
            



def main(args):
    '''Initializes and cleanup ros node'''
    bc = bClassifier()
    rospy.init_node('blobClassifier', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)