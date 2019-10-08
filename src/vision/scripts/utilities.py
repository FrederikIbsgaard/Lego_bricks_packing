# import the necessary packages
import argparse
import cv2
import json
import numpy as np
import os

NULL_STR = ""
NULL_JSON = "{}"
UNKNOWN_EXTENSION = "Unknown extension"
UNKNOWN_FILE = "Unknown file"
INVALID_OPTION = "The option introduced is invalid"
EMPTY_FILE = "The file is empty"
SUCCESS = "operation completed"


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
        left = int((width_image-width_roi)/2)
        right = int((width_image-width_roi)/2)
    else:
        left = int((width_image-width_roi-1)/2)
        right = int((width_image-width_roi+1)/2)

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
        return UNKNOWN_EXTENSION, NULL_JSON
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

                    print("INFO: Json file  [ " +
                          filename + " ] loaded correctly")
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
        generateFile(json_obj, ("./"+filename))
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
            print("ERROR: Couldn't find the image [ " + filename + " ]")
            return UNKNOWN_FILE, NULL_STR

# This function saves the content of a cv2 mat into an image.


def saveImage(image, name):

    filename = checkFilename(name, ".jpg")

    if filename == UNKNOWN_EXTENSION:
        filename = checkFilename(name, ".png")

    if filename == UNKNOWN_EXTENSION:
        print(
            "ERROR: The extension given to the file is not valid [ " + filename + " ], expected .jpg or .png")
        return UNKNOWN_EXTENSION

    else:
                   # save image
        cv2.imwrite(filename, image)
        print("INFO: Image file [ " + filename + " ] saved correctly")
        return SUCCESS


def updateColorConfigurationJson(key, values, filename):

    if key == "red" or key == "yellow" or key == "blue":

        print("INFO: Updating the Json file")
        # load json
        state, obj_json = loadJson(filename)

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
        print("ERROR: the introduced option is not valid, please introduce a valid option: r,g,b")


def getConfigurationValues(key, filename):

    if key == "red" or key == "yellow" or key == "blue":
        # load json
        state, obj_json = loadJson(filename)

        if state == SUCCESS:
            configuration = np.array([obj_json[key][0], obj_json[key][1], obj_json[key]
                                      [2], obj_json[key][3], obj_json[key][4], obj_json[key][5]])
            return SUCCESS, configuration
        else:
            return state, []

    else:
        print("ERROR: the introduced option is not valid, please introduce a valid option: r,g,b")
        return INVALID_OPTION, []


def cropImage(image, path_to_conf_file):

    # load Json file:
    state, obj_json = loadJson(path_to_conf_file)

    if state == SUCCESS:

        # crop image using the Json information:
        output = image[obj_json["Corner1"][0]:obj_json["Corner2"][0], obj_json["Corner1"][1]:obj_json["Corner2"][1]]
        return output
        