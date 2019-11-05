#!/usr/bin/env python

import vision_utilities as utilities_vision
import cv2
import time


if __name__ == "__main__":
    while True:
        state,image = utilities_vision.loadImage("temp.jpg")
        cv2.imshow("Result",image)
        cv2.waitKey(0)
