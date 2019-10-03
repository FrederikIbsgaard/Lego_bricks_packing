# General Configuration script for selectinh thresholds for color segmentation ad cropping points for the image
import os

interpreter = "python" #python3
image = "lego"


# Select cropping limits:
os.system(interpreter + " configuration_image_crop.py -i " + image + " -c Cropping_values")

# Select RGB threshold values:
os.system(interpreter + " configuration_rgb_thresholds.py -i " + image + " -o red -c RGB_thresholds")
os.system(interpreter + " configuration_rgb_thresholds.py -i " + image + " -o yellow -c RGB_thresholds")
os.system(interpreter + " configuration_rgb_thresholds.py -i " + image + " -o blue -c RGB_thresholds")

# Select HSV threshold values:
os.system(interpreter + " configuration_hsv_thresholds.py -i " + image + " -o red -c HSV_thresholds")
os.system(interpreter + " configuration_hsv_thresholds.py -i " + image + " -o yellow -c HSV_thresholds")
os.system(interpreter + " configuration_hsv_thresholds.py -i " + image + " -o blue -c HSV_thresholds")
