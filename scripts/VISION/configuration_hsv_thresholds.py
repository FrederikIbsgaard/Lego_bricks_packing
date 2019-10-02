# This code has parts taken from the following sources:
# - Thresholding Operations using inRange  [https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html]
# - Reading and Writing JSON to a File in Python [https://stackabuse.com/reading-and-writing-json-to-a-file-in-python/]
#
# Description:
# This script opens an image so the user can select a threshold values in HSV colorspace in order to apply color segmentation. The selected threshold values are saved into a json file.
#
# Instructions:
# Run with: python3 configuration_hsv_thresholds.py 
#           -i input_image.jpg 
#           -o r  # r for red color filtering ,g for green and b for blue
#           -c HSV_configuration values

# import the necessary packages
import argparse
import cv2
import json
import numpy as np
import os
import utilities as utils

# Function definitions:
def nothing(x):
    pass

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
ap.add_argument("-o", "--option", required=True,
                help="Option selecting the color to be checked")
ap.add_argument("-c", "--conf", required=True,
                help="Path to the Json file in which to save the threshold values")
args = vars(ap.parse_args())

# load the image:
state,image = utils.loadImage(args["image"])

# Check that the input image was loaded correctly
if state == utils.SUCCESS:

    # Creating a window for later use
    window_title = "HSV Threshold configuration (" + args["option"] + ")"
    cv2.namedWindow(window_title)

    # Get previous configuration values:
    state,configuration = utils.getConfigurationValues(args["option"],args["conf"])
    
    # Check that the values were retrieved correctly
    if state == utils.SUCCESS:
    
        # Creating track bar
        cv2.createTrackbar('h_min', window_title, configuration[0], 179, nothing)
        cv2.createTrackbar('h_max', window_title, configuration[1], 179, nothing)
        cv2.createTrackbar('s_min', window_title, configuration[2], 255, nothing)
        cv2.createTrackbar('s_max', window_title, configuration[3], 255, nothing)
        cv2.createTrackbar('v_min', window_title, configuration[4], 255, nothing)
        cv2.createTrackbar('v_max', window_title, configuration[5], 255, nothing)

        while(1):

            # converting to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # get info from track bar and appy to HSV Threshold configuration
            h_min = cv2.getTrackbarPos('h_min', window_title)
            h_max = cv2.getTrackbarPos('h_max', window_title)
            s_min = cv2.getTrackbarPos('s_min', window_title)
            s_max = cv2.getTrackbarPos('s_max', window_title)
            v_min = cv2.getTrackbarPos('v_min', window_title)
            v_max = cv2.getTrackbarPos('v_max', window_title)

            # Normal masking algorithm
            lower_bound = np.array([h_min, s_min, v_min])
            upper_bound = np.array([h_max, s_max, v_max])

            # Construct mask
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            output = cv2.bitwise_and(image, image, mask=mask)

            # show the images
            cv2.imshow(window_title, np.hstack([image, output]))

            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

        
        # Save configuration values:
        configuration = np.array([h_min, h_max,s_min,s_max, v_min,v_max])

        # Update Json file 
        utils.updateColorConfigurationJson(args["option"],configuration,args["conf"])
        