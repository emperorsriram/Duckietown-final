#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import BoolStamped
from duckietown_utils.jpg import bgr_from_jpg
from sensor_msgs.msg import CompressedImage, Image
from socket import *
import cv2
import numpy as np
import os
import rospy
import threading


class DuckieDetectionNode(object):
    def __init__(self):

        # This is the host's IP address
        self.serverName = '127.0.0.1'

        # This is an arbitrary serverPort, could be changed
        self.serverPort = 12000

        # Generates a client socket
        # where AF_INET indicates IPv4
        # and SOCK_DGRAM indicates it is a UDP socket
        self.clientSocket = socket(AF_INET, SOCK_DGRAM)

        self.bridge = CvBridge()
        self.thread_lock = threading.Lock()	

        # Subscribers 
        self.image_recieved = rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.cbImage)

        # Publishers
        self.pub_duckie_detection = rospy.Publisher("~duckie_detection", Image, queue_size=1)
        self.pub_duckie_detection_debug = rospy.Publisher("~duckie_detection_debug", Image, queue_size=10)
        self.pub_duckie_in_lane = rospy.Publisher("~duckie_in_lane", BoolStamped, queue_size=1)

        # Set up the SimpleBlobdetector with default parameters.
        self.params = cv2.SimpleBlobDetector_Params()
     
        # Change thresholds
        # Handles brightness, this only focuses on bright blobs
        self.params.minThreshold = 1
        self.params.maxThreshold = 100
     
        # Filter by Area.
        # Setting it to 450 makes it so we only detect the duckie 6inches away
        # Lowering it improves range but makes it more likely to detect garbage
        # Range caps at 1 foot.
        self.params.filterByArea = True
        self.params.minArea = 100
     
        # Filter by Circularity
        self.params.filterByCircularity = True
        # 0.75 Seems to WORK
        self.params.minCircularity = 0.70
     
        # Filter by Convexity
        # Seems to help with filtering, this range works
        # Looks like a complete shape, no deep curves
        self.params.filterByConvexity = True
        self.params.minConvexity = 0.95
        self.params.maxConvexity = 1
     
        # Filter by Inertia
        self.params.filterByInertia = False
        self.params.minInertiaRatio = 0

        self.detector = cv2.SimpleBlobDetector_create(self.params)
        

    def publishImage(self, publisher, image_msg):
        publisher.publish(image_msg)


    def cbImage(self, image_msg):
        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # Returns rightaway

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            # Return immediately if the thread is locked
            return

        try:
            self.duckie_detection(image_msg)
        finally:
            # Release the thread lock
            self.thread_lock.release()
            

    def duckie_detection(self, image_msg):
        # Decode from compressed image with OpenCV
        try:
            image_cv = bgr_from_jpg(image_msg.data)

        except ValueError as e:
            print('Could not decode image: %s' % e)
            return

        # Switch image from BGR colorspace to HSV
        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        # First reduce noise of image by blurring
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        orange_mask = cv2.inRange(hsv, (10, 100, 20), (20, 255, 255))

        # Helps to remove other blobs
        erosion_image_cv = cv2.erode(orange_mask,kernel,iterations = 3)

        # Dilate image that contains what we need after noise removed, (Removes a little noise too)
        orange_dilate_cv = cv2.dilate(erosion_image_cv, kernel, iterations=1)
            
        # Bitwise-AND of mask and yellow only image
        orange_only_image_cv = cv2.bitwise_and(image_cv, image_cv, mask = orange_dilate_cv)

        # Detect blobs.
        reversemask = 255 - orange_dilate_cv
        keypoints = self.detector.detect(reversemask)

        # We can keep track of the point with the largest diameter, it is likely a duckie
        largestBlob = 0
        duckPoint = None
        if len(keypoints) != 0:
            for points in range(len(keypoints)):
                    x = int(keypoints[points].pt[0])
                    y = int(keypoints[points].pt[1])
                    diameter = int(keypoints[0].size)
                    print("Value of X = ", x, "Value of Y = ", y, "Diameter is : ", diameter)
                    if diameter > largestBlob:
                        duckPoint = points
                        largestBlob = diameter

            duckieX = int(keypoints[duckPoint].pt[0])
            duckieY = int(keypoints[duckPoint].pt[1])
            duckieD = int(keypoints[duckPoint].size)
            cv2.putText(image_cv, "Duckie", (duckieX - 20, duckieY - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Handle sending UDP message to Server
            message = str(duckieD)
            self.clientSocket.sendto(message.encode(), (self.serverName, self.serverPort))
            # returnMessage, serverAddress = self.clientSocket.recvfrom(2048)

        # Draw detected blobs as green circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(image_cv, keypoints, np.array([]), (0,255,0),
cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        keypoints_image_ros = self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8")
        
        # Publish images
        self.publishImage(self.pub_duckie_detection, keypoints_image_ros)
        
        # Send the Server what it has detected


if __name__ == '__main__': 
    rospy.init_node('duckie_detector_node', anonymous=False)
    duckie_detection_node = DuckieDetectionNode()
    rospy.spin()

