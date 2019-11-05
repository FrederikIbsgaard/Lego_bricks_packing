# General Configuration script for selectinh thresholds for color segmentation ad cropping points for the image
import os
import subprocess
interpreter = "python" #python3

import vision_utilities as utilities

image = "conf"
img = utilities.getImageFromPicam([640,480])
utilities.saveImage(img,"conf")

# Select cropping limits:
os.system(interpreter + " configuration_image_crop.py -i " + image + " -c Cropping_values_red")
os.system(interpreter + " configuration_image_crop.py -i " + image + " -c Cropping_values_blue")
os.system(interpreter + " configuration_image_crop.py -i " + image + " -c Cropping_values_yellow")

# Select RGB threshold values:
#os.system(interpreter + " configuration_rgb_thresholds.py -i " + image + " -o red -c RGB_thresholds")
#os.system(interpreter + " configuration_rgb_thresholds.py -i " + image + " -o yellow -c RGB_thresholds")
#os.system(interpreter + " configuration_rgb_thresholds.py -i " + image + " -o blue -c RGB_thresholds")

# Select HSV threshold values:
os.system(interpreter + " configuration_hsv_thresholds.py -i " + image + " -o red -c HSV_thresholds")
os.system(interpreter + " configuration_hsv_thresholds.py -i " + image + " -o yellow -c HSV_thresholds")
os.system(interpreter + " configuration_hsv_thresholds.py -i " + image + " -o blue -c HSV_thresholds")
