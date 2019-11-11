# import the necessary packages
import argparse
import cv2
import json
import numpy as np
import os
import time

NULL_STR = ""
NULL_JSON = "{}"
VALID_COLORS = ["red","yellow","blue"]
UNKNOWN_EXTENSION = "ERROR: The extension given to the file is not valid. "
UNKNOWN_FILE = "ERROR: Unknown file, the requested file can't be found in the intoduced path. "
INVALID_COLOR_OPTION = "ERROR: the introduced option is not valid, please introduce a valid option: red,yellow,blue. "
INVALID_COLORSPACE_OPTION = "ERROR: Please introduce a valid option, either RGB or HSV. "
EMPTY_FILE = "The file is empty"
SUCCESS = "operation completed"
RGB = "RGB"
HSV = "HSV"

# Path to configuration files
PATH_TO_CONF_CROPPING_JSON_RED = os.path.expanduser(
    "../conf/Cropping_values_red.json")
PATH_TO_CONF_CROPPING_JSON_BLUE = os.path.expanduser(
    "../conf/Cropping_values_blue.json")
PATH_TO_CONF_CROPPING_JSON_YELLOW = os.path.expanduser(
    "../conf/Cropping_values_yellow.json")
PATH_TO_CONF_HSV_JSON = os.path.expanduser(
    "../conf/HSV_thresholds.json")
PATH_TO_CONF_RGB_JSON = os.path.expanduser(
    "../conf/RGB_thresholds.json")


def setPathsToConfigurationFiles(path_to_cropping_conf_red,path_to_cropping_conf_blue,path_to_cropping_conf_yellow, path_to_hsv_conf, path_to_rgb_conf):
    # Path to configuration files
    PATH_TO_CONF_CROPPING_JSON_RED = os.path.expanduser(path_to_cropping_conf_red)
    PATH_TO_CONF_CROPPING_JSON_BLUE = os.path.expanduser(path_to_cropping_conf_blue)
    PATH_TO_CONF_CROPPING_JSON_YELLOW = os.path.expanduser(path_to_cropping_conf_yellow)
    PATH_TO_CONF_HSV_JSON = os.path.expanduser(path_to_hsv_conf)
    PATH_TO_CONF_RGB_JSON = os.path.expanduser(path_to_rgb_conf)


def addPaddingToMatchSize(roi, image_to_match):

    height_image, width_image, channels_image = image_to_match.shape
    height_roi, width_roi, channels_roi = roi.shape

    if((height_image-height_roi) % 2) == 0:
        top = int((height_image-height_roi)/2)
        bottom = int((height_image-height_roi)/2)
    else:
        top = int((height_image-height_roi-1)/2)
        bottom = int((height_image-height_roi+1)/2)

    if((width_image-width_roi) % 2) == 0:
        left = 10 #int((width_image-width_roi)/2)
        right = 10 #int((width_image-width_roi)/2)
    else:
        left = 10#int((width_image-width_roi-1)/2)
        right = 10#int((width_image-width_roi+1)/2)

    output = cv2.copyMakeBorder(
        roi, top, bottom, left, right, cv2.BORDER_CONSTANT, value=[255, 255, 255])

    return output

# This function checks that a filename has the expected extension.
# In case it doesn't have an extension, it would be added.
# In case the filename has an extension but is not valid, the returned value will be NULL


def checkFilename(name, extension):
    filename, file_extension = os.path.splitext(name)
    if file_extension == extension:
        return name
    elif file_extension == "":
        return name + extension
    else:
        return UNKNOWN_EXTENSION


# This function loads a json file into a json object and checks that the filename is correct and the file exists


def loadJson(filename):
    filename = checkFilename(filename, ".json")

    if filename == UNKNOWN_EXTENSION:
        print(
            "ERROR: The extension given to the file is not valid [ " + filename + " ], expected .json")
        return UNKNOWN_EXTENSION, NULL_JSONe
    else:
        # Check that the file exists
        if os.path.isfile(filename) == True:
            # open file
            with open(filename, 'r') as myfile:
                # load data
                data = myfile.read()

                if data == "":
                    print("ERROR: Json file [ " + filename + " ] is empty")
                    return EMPTY_FILE, NULL_JSON
                else:
                    # as a string:
                    obj = json.loads(data)

                    #print("INFO: Json file  [ " + filename + " ] loaded correctly")
                    return SUCCESS, obj
        else:
            print(
                "INFO: Couldn't find the Json file [ " + filename + " ], creating it now.")
            # create a Python object containing the cropping points:
            obj_json = {
                "red": [0, 255, 0, 255, 0, 255],
                "yellow": [0, 255, 0, 255, 0, 255],
                "blue": [0, 255, 0, 255, 0, 255]

            }
            # save Json:
            saveJson(obj_json, filename)

            print(UNKNOWN_FILE)
            return UNKNOWN_FILE, obj_json


def generateFile(data, filename):
    with open(filename, 'w') as outfile:
        outfile.write(json.dumps(data, indent=4, sort_keys=True))


def saveJson(json_obj, filename):
    filename = checkFilename(filename, ".json")

    if filename == UNKNOWN_EXTENSION:
        print(
            "ERROR: The extension given to the file is not valid [ " + filename + " ], expected .json")
        return UNKNOWN_EXTENSION
    else:
        generateFile(json_obj, filename)
        print("INFO: Json file saved correctly")
        return SUCCESS

# This functions loads an image and checks that the filename is correct and the file exists


def loadImage(name):

    filename = checkFilename(name, ".jpg")

    if filename == UNKNOWN_EXTENSION:
        filename = checkFilename(name, ".png")

    if filename == UNKNOWN_EXTENSION:
        print(
            "ERROR: The extension given to the file is not valid [ " + filename + " ], expected .jpg or .png")
        return UNKNOWN_EXTENSION, NULL_STR
    else:
        # Check that the file exists
        if os.path.isfile(filename) == True:
            print("INFO: Image file [ " + filename + " ] loaded correctly")
            return SUCCESS, cv2.imread(filename)
        else:
            print(UNKNOWN_FILE + " [ " + filename + " ]")
            return UNKNOWN_FILE, NULL_STR

# This function saves the content of a cv2 mat into an image.


def saveImage(image, name):

    filename = checkFilename(name, ".jpg")

    if filename == UNKNOWN_EXTENSION:
        filename = checkFilename(name, ".png")

    if filename == UNKNOWN_EXTENSION:
        print(UNKNOWN_EXTENSION  + " [ " + filename + " ], expected .jpg or .png")
        return UNKNOWN_EXTENSION

    else:
                   # save image
        cv2.imwrite(filename, image)
        print("INFO: Image file [ " + filename + " ] saved correctly")
        return SUCCESS


def updateColorConfigurationJson(key, values, option):

    if key == "red" or key == "yellow" or key == "blue":

        print("INFO: Updating the Json file")
        
        # load json
        state = []
        configuration  = []

        if option == RGB:
            state, obj_json = loadJson(PATH_TO_CONF_RGB_JSON)
        elif option == HSV:
            state, obj_json = loadJson(PATH_TO_CONF_HSV_JSON)
        else:
            print(INVALID_COLORSPACE_OPTION)
        

        if state == SUCCESS:

            # update values
            obj_json[key][0] = int(values[0])
            obj_json[key][1] = int(values[1])
            obj_json[key][2] = int(values[2])
            obj_json[key][3] = int(values[3])
            obj_json[key][4] = int(values[4])
            obj_json[key][5] = int(values[5])

            # save Json:
            saveJson(obj_json, filename)

    else:
        print(INVALID_COLOR_OPTION)


def getConfigurationValues(key, option):

    #print("INFO: Loading the configuration values for: " + key)
    if key == "red" or key == "yellow" or key == "blue":

        # load json
        state = []
        configuration  = []

        if option == RGB:
            state, obj_json = loadJson(PATH_TO_CONF_RGB_JSON)
        elif option == HSV:
            state, obj_json = loadJson(PATH_TO_CONF_HSV_JSON)
        else:
            print(INVALID_COLORSPACE_OPTION) 
            return INVALID_COLORSPACE_OPTION, []

        if state == SUCCESS:
            configuration = np.array([obj_json[key][0], obj_json[key][1], obj_json[key]
                                      [2], obj_json[key][3], obj_json[key][4], obj_json[key][5]])
            return SUCCESS, configuration
        else:
            return state, []
    else:
        return INVALID_COLOR_OPTION, []


def cropImage(image,color):

    # load Json file:
    if VALID_COLORS[0] == color:
    	state, obj_json = loadJson(PATH_TO_CONF_CROPPING_JSON_RED)
        #print("INFO: Cropping image, selected color is: " + color + ' / ' + VALID_COLORS[0])
    elif VALID_COLORS[1] == color:
    	state, obj_json = loadJson(PATH_TO_CONF_CROPPING_JSON_YELLOW)
        #print("INFO: Cropping image, selected color is: " + color + ' / ' + VALID_COLORS[1])
    elif VALID_COLORS[2] == color:
    	state, obj_json = loadJson(PATH_TO_CONF_CROPPING_JSON_BLUE)
        #print("INFO: Cropping image, selected color is: " + color + ' / ' + VALID_COLORS[2])

    if state == SUCCESS:

        # crop image using the Json information:
        output = image[obj_json["Corner1"][0]:obj_json["Corner2"]
                       [0], obj_json["Corner1"][1]:obj_json["Corner2"][1]]

        return output


def filterHSV(image, color):

    # Get previous configuration values:
    state, configuration = getConfigurationValues(
        color, HSV)

    # converting to HSV
    rgb = image.copy()
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Normal masking algorithm
    lower_bound = np.array(
        [configuration[0], configuration[2], configuration[4]])
    upper_bound = np.array(
        [configuration[1], configuration[3], configuration[5]])

    # Construct mask
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    output = cv2.bitwise_and(image, image, mask=mask)

    return output


def filterRGB(image, color):

    # Get previous configuration values:
    state, configuration = getConfigurationValues(
        color, RGB)

    # copying the image
    rgb = image.copy()

    # Normal masking algorithm
    lower_bound = np.array(
        [configuration[0], configuration[2], configuration[4]])
    upper_bound = np.array(
        [configuration[1], configuration[3], configuration[5]])

    # Construct mask
    mask = cv2.inRange(rgb, lower_bound, upper_bound)
    output = cv2.bitwise_and(rgb, image, mask=mask)

    return output


def getColor(image, option,expected_color):
    
    for color in VALID_COLORS:
        if(color == expected_color):
            #print("INFO: Getting color, expected color is: " + expected_color)
            state = [] 
            configuration  = []
            filtered_image = []

            if option == RGB:
                state, configuration = getConfigurationValues(color,RGB)
                filtered_image  = filterRGB(image,color)
            elif option == HSV:
                state, configuration = getConfigurationValues(color,HSV)
                filtered_image  = filterHSV(image,color)
            else:
                return INVALID_COLORSPACE_OPTION
            lower_bound = np.array(
		        [configuration[0], configuration[2], configuration[4]])
            upper_bound = np.array(
		        [configuration[1], configuration[3], configuration[5]])


            avg = getAverageColor(filtered_image, option,color)
        
            height, width = filtered_image.shape[:2]
            if(avg > (height*width)*0.5 ):		       
                return color

    print("INFO: predominant color  doesn't match any color")
    return NULL_STR


def getAverageColor(image, option,color):

    if option == HSV:
        # Get previous configuration values:
        state, configuration = getConfigurationValues(color, HSV)

        # converting to HSV
        rgb = image.copy()
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        # Normal masking algorithm
        lower_bound = np.array([configuration[0], configuration[2], configuration[4]])
        upper_bound = np.array([configuration[1], configuration[3], configuration[5]])
        # Construct mask
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        output = cv2.bitwise_and(image, image, mask=mask)
        saveImage(output, "images/step_2")
        avg = np.sum(mask==255)



        return avg

    elif option == RGB:

        valid_points = np.array([0, 0, 0])
        for points in image:
            # print(points)
            if points[1][0] != 0 or points[1][1] != 0 or points[1][2] != 0:
                valid_points = np.vstack(
                    [valid_points, [points[1][0], points[1][1], points[1][2]]])

        # Remove the first entry sincs its not real
        valid_points = valid_points[1:]

        # Compute the average rgb value:
        avg = np.average(valid_points, axis=0)

        # In case there aren't any valid points the returned avg value is a float instead of an array, therefore it needs to be redeclared in order not to mess with the code
        if isinstance(avg, np.float64):
            avg = np.array([0, 0, 0])

        return avg

    else:
        print(INVALID_COLORSPACE_OPTION)






