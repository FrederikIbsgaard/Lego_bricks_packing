import cv2
import numpy as np
import os
import utilities as utilities



bool, img = utilities.loadImage("lego")
#cv2.namedWindow('l', cv2.WINDOW_NORMAL)
cv2.imshow("lego",img)
cv2.waitKey(0)
