# General Configuration script for selectinh thresholds for color segmentation ad cropping points for the image
import os

interpreter = "python" #python3

image = "/home/pi/Desktop/image.jpg"

from picamera import PiCamera
from time import sleep

camera = PiCamera()

camera.start_preview()
#sleep(5)
camera.capture(image) 
camera.stop_preview()



# Select cropping limits:
os.system(interpreter + " conf_files/configuration_image_crop.py -i " + image + " -c conf_files/Cropping_values")

# Select RGB threshold values:
os.system(interpreter + " conf_files/configuration_rgb_thresholds.py -i " + image + " -o red -c conf_files/RGB_thresholds")
os.system(interpreter + " conf_files/configuration_rgb_thresholds.py -i " + image + " -o yellow -c conf_files/RGB_thresholds")
os.system(interpreter + " conf_files/configuration_rgb_thresholds.py -i " + image + " -o blue -c conf_files/RGB_thresholds")

# Select HSV threshold values:
os.system(interpreter + " conf_files/configuration_hsv_thresholds.py -i " + image + " -o red -c conf_files/HSV_thresholds")
os.system(interpreter + " conf_files/configuration_hsv_thresholds.py -i " + image + " -o yellow -c conf_files/HSV_thresholds")
os.system(interpreter + " conf_files/configuration_hsv_thresholds.py -i " + image + " -o blue -c conf_files/HSV_thresholds")
