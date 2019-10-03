import os

interpreter = "python" #python3
image = "images/lego"

# Select cropping limits:
os.system(interpreter + "  filter_crop.py -i " + image + " -f images/result_cropped -c conf_files/Cropping_values")

image = "images/result_cropped"
# With RGB color segmentation:
os.system(interpreter + " src/filter_color_rgb.py -i " + image + " -f images/result_red_rgb  -o red -c conf_files/RGB_thresholds")
os.system(interpreter + " src/filter_color_rgb.py -i " + image + " -f images/result_yellow_rgb  -o yellow -c conf_files/RGB_thresholds")
os.system(interpreter + " src/filter_color_rgb.py -i " + image + " -f images/result_blue_rgb  -o blue -c conf_files/RGB_thresholds")

# With HSV color segmentation:
os.system(interpreter + " src/filter_color_hsv.py -i " + image + " -f images/result_red_hsv  -o red -c conf_files/HSV_thresholds")
os.system(interpreter + " src/filter_color_hsv.py -i " + image + " -f images/result_yellow_hsv  -o yellow -c conf_files/HSV_thresholds")
os.system(interpreter + " src/filter_color_hsv.py -i " + image + " -f images/result_blue_hsv  -o blue -c conf_files/HSV_thresholds")

# Show results:
os.system(interpreter + " src/show_results.py")