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
ap.add_argument("-c", "--conf", required=True,
                help="Path to the Json file containing the configuration points for the cropping ")
args = vars(ap.parse_args())

# load the image:
state,image = utils.loadImage(args["image"])

# Check that the input image exist
if state == utils.SUCCESS:
    # load Json file:
    state,obj_json = utils.loadJson(args["conf"])

    if state == utils.SUCCESS:
          
        # crop image using the Json information:
        output = image[obj_json["Corner1"][0]:obj_json["Corner2"][0], obj_json["Corner1"][1]:obj_json["Corner2"][1]]

        # Show results:
        padded_output = utils.addPaddingToMatchSize(output, image)
        cv2.imshow("Cropping result", np.hstack([image, padded_output]))

        # save result:       
        if  utils.saveImage(output,args["filename_output"]) == utils.SUCCESS:
            cv2.waitKey(0)        
        
