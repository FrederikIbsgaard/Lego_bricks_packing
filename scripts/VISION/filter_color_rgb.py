# import the necessary packages
import argparse
import cv2
import json
import numpy as np
import os
import utilities as utils


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
ap.add_argument("-f", "--filename_output", required=True,
                help="Filename for the cropped image")
ap.add_argument("-o", "--option", required=True,
                help="Option selecting the color to be checked")
ap.add_argument("-c", "--conf", required=True,
                help="Path to the Json file containing the configuration points for the cropping ")
args = vars(ap.parse_args())

# load the image:
state, image = utils.loadImage(args["image"])

# Check that the input image exist
if state == utils.SUCCESS:

    # Get previous configuration values:
    state, configuration = utils.getConfigurationValues(
        args["option"], args["conf"])

    # copying the image
    rgb = image.copy()

    # Normal masking algorithm
    lower_bound = np.array([configuration[0], configuration[2], configuration[4]])
    upper_bound = np.array([configuration[1], configuration[3], configuration[5]])

    # Construct mask
    mask = cv2.inRange(rgb, lower_bound, upper_bound)
    output = cv2.bitwise_and(rgb, image, mask=mask)

    avg = cv2.mean(output)
    
    if avg[0] >= configuration[0] and avg[0] <= configuration[1] and avg[1] >= configuration[2] and avg[1] <= configuration[3] and avg[2] >= configuration[4] and avg[2] <= configuration[5]:

       print("INFO: predominant color matches: " + args["option"])

    else:
        
       print("INFO: predominant color  doesn't match: " + args["option"])


    # show the images
    cv2.imshow("Result of applying color segmentation", np.hstack([image, output]))
    cv2.waitKey(0)  
