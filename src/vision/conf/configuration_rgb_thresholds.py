# This code has parts taken from the following sources:
# - Thresholding Operations using inRange  [https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html]
# - Reading and Writing JSON to a File in Python [https://stackabuse.com/reading-and-writing-json-to-a-file-in-python/]
#
# Description:
# This script opens an image so the user can select a threshold values in RGB colorspace in order to apply color segmentation. The selected threshold values are saved into a json file.
#
# Instructions:
# Run with: python3 configuration_rgb_thresholds.py 
#           -i input_image.jpg 
#           -o r  # red,green or yellow for color filtering 
#           -c RGB_configuration values
 
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
    window_title = "RGB Threshold configuration (" + args["option"] + ")"
    cv2.namedWindow(window_title)

    # Get previous configuration values:
    state,configuration = utils.getConfigurationValues(args["option"],args["conf"])
    
    # Check that the values were retrieved correctly
    if state == utils.SUCCESS:
        # Creating track bars
        cv2.createTrackbar('r_min', window_title, configuration[0], 255, nothing)
        cv2.createTrackbar('r_max', window_title, configuration[1], 255, nothing)
        cv2.createTrackbar('g_min', window_title, configuration[2], 255, nothing)
        cv2.createTrackbar('g_max', window_title, configuration[3], 255, nothing)
        cv2.createTrackbar('b_min', window_title, configuration[4], 255, nothing)
        cv2.createTrackbar('b_max', window_title, configuration[5], 255, nothing)

        while(1):

            # copying the image
            rgb = image.copy()

            # get info from track bar and appy to RGB Threshold configuration
            r_min = cv2.getTrackbarPos('r_min', window_title)
            r_max = cv2.getTrackbarPos('r_max', window_title)
            g_min = cv2.getTrackbarPos('g_min', window_title)
            g_max = cv2.getTrackbarPos('g_max', window_title)
            b_min = cv2.getTrackbarPos('b_min', window_title)
            b_max = cv2.getTrackbarPos('b_max', window_title)

            # Normal masking algorithm
            lower_bound = np.array([r_min, g_min, b_min])
            upper_bound = np.array([r_max, g_max, b_max])

            # Construct mask
            mask = cv2.inRange(rgb, lower_bound, upper_bound)
            output = cv2.bitwise_and(rgb, image, mask=mask)

            # show the images
            cv2.imshow(window_title, np.hstack([image, output]))

            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break


        # Save configuration values:
        configuration = np.array([r_min,r_max,g_min,g_max, b_min,b_max])

        # Update Json file 
        utils.updateColorConfigurationJson(args["option"],configuration,args["conf"])
        