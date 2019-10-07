# This code has parts taken from the following sources:
# - Capturing mouse click events with Python and OpenCV [https://www.pyimagesearch.com/2015/03/09/capturing-mouse-click-events-with-python-and-opencv/]
# - Reading and Writing JSON to a File in Python [https://stackabuse.com/reading-and-writing-json-to-a-file-in-python/]
#
# Description:
# This script opens an image so the user can select a rectangular patch to be cropped. The corners of the rectangle are saved into a json file.
#
# Instructions:
# Run with: python3 configuration_image_crop.py -i input_image.jpg -f output_filename

# import the necessary packages
import argparse
import cv2
import json
import os
import utilities as utils

# Functions definition:
def clickAndCrop(event, x, y, flags, param):
	# grab references to the global variables
	global refPt, cropping

	# if the left mouse button was clicked, record the starting
	# (x, y) coordinates and indicate that cropping is being
	# performed
	if event == cv2.EVENT_LBUTTONDOWN:
		refPt = [(x, y)]
		cropping = True

	# check to see if the left mouse button was released
	elif event == cv2.EVENT_LBUTTONUP:
		# record the ending (x, y) coordinates and indicate that
		# the cropping operation is finished
		refPt.append((x, y))
		cropping = False

		# draw a rectangle around the region of interest
		cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
		cv2.imshow("image", image)

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=True, help="Path to the image")
ap.add_argument("-c", "--conf", required=True, help="Configuration file for the Json containing the cropping points")
args = vars(ap.parse_args())

# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
refPt = []
cropping = False

# load the image:
state,image = utils.loadImage(args["image"])

# Check that the input image exist
if state == utils.SUCCESS:

	clone = image.copy()
	cv2.namedWindow("image")
	cv2.setMouseCallback("image", clickAndCrop)

	# keep looping until the 'q' key is pressed
	while True:
		# display the image and wait for a keypress
		cv2.imshow("image", image)
		key = cv2.waitKey(1) & 0xFF

		# if the 'r' key is pressed, reset the cropping region
		if key == ord("r"):
			image = clone.copy()

		# if the 'c' key is pressed, break from the loop
		elif key == ord("c"):
			break

	# if there are two reference points, then crop the region of interest
	# from teh image and display it
	if len(refPt) == 2:
		roi = clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
		cv2.imshow("ROI", roi)
		cv2.waitKey(0)

		# create a Python object containing the cropping points:
		roi_points = {
		"Corner1": [refPt[0][1],refPt[0][0]],
		"Corner2": [refPt[1][1],refPt[1][0]]
		}

		# save Json into a file:
		utils.saveJson(roi_points,args["conf"])

	# close all open windows
	cv2.destroyAllWindows()


