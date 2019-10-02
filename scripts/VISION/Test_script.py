import os

image = "lego"
# Select cropping limits:
os.system("python3 filter_crop.py -i " + image + " -f result -c Cropping_values")

image = "result"
os.system("python3 filter_color_rgb.py -i " + image + " -f result  -o red -c RGB_thresholds")