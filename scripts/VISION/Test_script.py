import os

image = "lego"
# Select cropping limits:
os.system("python3 filter_crop.py -i " + image + " -f result -c Cropping_values")

image = "result"
# With RGB color segmentation:
os.system("python3 filter_color_rgb.py -i " + image + " -f result  -o red -c RGB_thresholds")
os.system("python3 filter_color_rgb.py -i " + image + " -f result  -o yellow -c RGB_thresholds")
os.system("python3 filter_color_rgb.py -i " + image + " -f result  -o blue -c RGB_thresholds")

# With HSV color segmentation:
os.system("python3 filter_color_hsv.py -i " + image + " -f result  -o red -c HSV_thresholds")
os.system("python3 filter_color_hsv.py -i " + image + " -f result  -o yellow -c HSV_thresholds")
os.system("python3 filter_color_hsv.py -i " + image + " -f result  -o blue -c HSV_thresholds")