# General Configuration script for selectinh thresholds for color segmentation ad cropping points for the image
import os

image = "lego"

# Select cropping limits:
os.system("python3 configuration_image_crop.py -i " + image + " -c Cropping_values")

# Select RGB threshold values:
os.system("python3 configuration_rgb_thresholds.py -i " + image + " -o red -c RGB_thresholds")
os.system("python3 configuration_rgb_thresholds.py -i " + image + " -o yellow -c RGB_thresholds")
os.system("python3 configuration_rgb_thresholds.py -i " + image + " -o blue -c RGB_thresholds")

# Select HSV threshold values:
os.system("python3 configuration_hsv_thresholds.py -i " + image + " -o red -c HSV_thresholds")
os.system("python3 configuration_hsv_thresholds.py -i " + image + " -o yellow -c HSV_thresholds")
os.system("python3 configuration_hsv_thresholds.py -i " + image + " -o blue -c HSV_thresholds")
