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
    lower_bound = np.array(
        [configuration[0], configuration[2], configuration[4]])
    upper_bound = np.array(
        [configuration[1], configuration[3], configuration[5]])

    # Construct mask
    mask = cv2.inRange(rgb, lower_bound, upper_bound)
    output = cv2.bitwise_and(rgb, image, mask=mask)

    valid_points = np.array([0, 0, 0])
    for points in output:
        #print(points)
        if points[1][0] != 0 or points[1][1] != 0 or points[1][2] != 0:
            valid_points = np.vstack([valid_points, [points[1][0],points[1][1],points[1][2]]])

    # Remove the first entry sincs its not real
    valid_points = valid_points[1:]
    
    # Compute the average rgb value:
    avg = np.average(valid_points, axis=0)

    # In case there aren't any valid points the returned avg value is a float instead of an array, therefore it needs to be redeclared in order not to mess with the code
    if isinstance(avg, np.float64):
       avg = np.array([0, 0, 0])

    print("INFO: Threshold and currrent values")
    print("\tcurrent = " + str(int(avg[0])) + " Threshold values [" + str(configuration[0]) + " , " + str(configuration[1]) + "]")
    print("\tcurrent = " + str(int(avg[1])) + " Threshold values [" + str(configuration[2]) + " , " + str(configuration[3]) + "]")
    print("\tcurrent = " + str(int(avg[2])) + " Threshold values [" + str(configuration[4]) + " , " + str(configuration[5]) + "]")

    if avg[0] >= configuration[0] and avg[0] <= configuration[1] and avg[1] >= configuration[2] and avg[1] <= configuration[3] and avg[2] >= configuration[4] and avg[2] <= configuration[5]:

        print("INFO: predominant color matches: " + args["option"])

    else:

        print("INFO: predominant color  doesn't match: " + args["option"])

    # save result image:
    utils.saveImage(output,args["filename_output"])

    # show the images
    cv2.imshow("Result of applying RGB color segmentation",
               np.hstack([image, output]))
    cv2.waitKey(0)
