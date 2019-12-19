#!/usr/bin/env python
import cv2
import numpy as np
import os
import time
import sys
import yaml

def DuckieDetectionNode(image):

    # Switch image from BGR colorspace to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # First reduce noise of image by blurring
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
    orange_mask = cv2.inRange(hsv, (10, 150, 20), (20, 255, 255))

    cv2.imshow("Orange Filter Image", orange_mask)

    # Helps to remove other blobs
    erosion_image = cv2.erode(orange_mask,kernel,iterations = 1)

    cv2.imshow("Orange Eroded Image", erosion_image)

    # Dilate image that contains what we need after noise removed, (Removes a little noise too)
    orange_dilate = cv2.dilate(erosion_image, kernel, iterations=1)

    cv2.imshow("Orange Dilated Image", orange_dilate)
        
    # Bitwise-AND of mask and yellow only image
    orange_only_image = cv2.bitwise_and(image, image, mask = orange_dilate)

    cv2.imshow("orange only image", orange_only_image)

    # Set up the SimpleBlobdetector with default parameters.
    params = cv2.SimpleBlobDetector_Params()
     
    # Change thresholds
    # Handles brightness, this only focuses on bright blobs
    # Chose 30, since 20 and 40 works, dont want to limit range too much
    params.minThreshold = 1;
    params.maxThreshold = 30;
     
    # Filter by Area.
    # Setting it to 450 makes it so we only detect the duckie 6inches away
    # Lowering it improves range but makes it more likely to detect garbage
    # Range caps at 1 foot.
    params.filterByArea = True
    params.minArea = 50
     
    # Filter by Circularity
    params.filterByCircularity = True
    # 0.75 Seems to WORK
    params.minCircularity = 0.75
     
    # Filter by Convexity
    # Seems to help with filtering, this range works
    # Looks like a complete shape, no deep curves
    params.filterByConvexity = True
    params.minConvexity = 0.95
    params.maxConvexity = 1
     
    # Filter by Inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.2

    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs.
    reversemask = 255 - orange_dilate
    keypoints = detector.detect(reversemask)

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
        cv2.putText(image, "Duckie", (duckieX - 20, duckieY - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    cv2.imwrite("DetectionofDuckie.png", im_with_keypoints)

    # Wait for key press to close images
    cv2.waitKey()
                    
 


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s image_filename.png" % sys.argv[0])
        exit()
        
    image_filename = sys.argv[1]
    image = cv2.imread(image_filename)
    DuckieDetectionNode(image)
