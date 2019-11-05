# General Configuration script for selectinh thresholds for color segmentation ad cropping points for the image
import os
import subprocess
from time import sleep

interpreter = "python" #python3

image = "test.jpg"

cmd = "raspistill -w 640 -h 480  -o test.jpg"
subprocess.call(cmd, shell=True)

sleep(1)





# Select cropping limits:
os.system(interpreter + " conf_files/configuration_image_crop.py -i " + image + " -c conf_files/Cropping_values")
'''
# Select RGB threshold values:
os.system(interpreter + " conf_files/configuration_rgb_thresholds.py -i " + image + " -o red -c conf_files/RGB_thresholds")
os.system(interpreter + " conf_files/configuration_rgb_thresholds.py -i " + image + " -o yellow -c conf_files/RGB_thresholds")
os.system(interpreter + " conf_files/configuration_rgb_thresholds.py -i " + image + " -o blue -c conf_files/RGB_thresholds")

# Select HSV threshold values:
os.system(interpreter + " conf_files/configuration_hsv_thresholds.py -i " + image + " -o red -c conf_files/HSV_thresholds")
os.system(interpreter + " conf_files/configuration_hsv_thresholds.py -i " + image + " -o yellow -c conf_files/HSV_thresholds")
os.system(interpreter + " conf_files/configuration_hsv_thresholds.py -i " + image + " -o blue -c conf_files/HSV_thresholds")
'''
