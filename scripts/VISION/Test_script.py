import os

interpreter = "python" #python3
image = "lego"

# Select cropping limits:
os.system(interpreter + "  filter_crop.py -i " + image + " -f result_cropped -c Cropping_values")

image = "result_cropped"
# With RGB color segmentation:
os.system(interpreter + " filter_color_rgb.py -i " + image + " -f result_red_rgb  -o red -c RGB_thresholds")
os.system(interpreter + " filter_color_rgb.py -i " + image + " -f result_yellow_rgb  -o yellow -c RGB_thresholds")
os.system(interpreter + " filter_color_rgb.py -i " + image + " -f result_blue_rgb  -o blue -c RGB_thresholds")

# With HSV color segmentation:
os.system(interpreter + " filter_color_hsv.py -i " + image + " -f result_red_hsv  -o red -c HSV_thresholds")
os.system(interpreter + " filter_color_hsv.py -i " + image + " -f result_yellow_hsv  -o yellow -c HSV_thresholds")
os.system(interpreter + " filter_color_hsv.py -i " + image + " -f result_blue_hsv  -o blue -c HSV_thresholds")

# Show results:
os.system(interpreter + " show_results.py -i ")