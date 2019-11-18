#!/usr/bin/env python

from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer
from twisted.internet.task import LoopingCall
import modbus_utilities as utilities_modbus
import vision_utilities as utilities_vision
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
import subprocess
logging.basicConfig()
log = logging.getLogger()


def updating_server(a):

	# Configure the registers to be used
    context = a[0]
    register = 3
    slave_id = 0x00
    address = utilities_modbus.ADDRESS

    # Read the register values
    values = context[slave_id].getValues(register, address, count=2)
    context[slave_id].setValues(register, address, values)

    if(values[0]== utilities_modbus.GET_BRICK_COLOR_BLUE):
        if checkColor(utilities_modbus.BLUE_BRICK):
            values[1]=utilities_modbus.RESULT_COLOR_MATCH
        else:
            values[1]=utilities_modbus.RESULT_COLOR_MISMATCH

    elif(values[0]== utilities_modbus.GET_BRICK_COLOR_RED):
        if checkColor(utilities_modbus.RED_BRICK):
            values[1]=utilities_modbus.RESULT_COLOR_MATCH
        else:
            values[1]=utilities_modbus.RESULT_COLOR_MISMATCH 

    elif(values[0]== utilities_modbus.GET_BRICK_COLOR_YELLOW):
        if checkColor(utilities_modbus.YELLOW_BRICK):
            values[1]=utilities_modbus.RESULT_COLOR_MATCH
        else:
            values[1]=utilities_modbus.RESULT_COLOR_MISMATCH
    
    elif(values[0]== utilities_modbus.GET_BRICK_COLOR_WAIT):
        print('INFO: Waiting for order')
        values[1]=utilities_modbus.RESULT_WAIT

    else:
        print('ERROR: Invalid value in the register')
        values[1]=utilities_modbus.RESULT_WAIT


    # Reset register value:
    values[0]=utilities_modbus.GET_BRICK_COLOR_WAIT
    context[slave_id].setValues(register, address, values)


camera = PiCamera()


def getImageFromPicam(resolution):
	camera.resolution = (resolution[0], resolution[1])
	rawCapture = PiRGBArray(camera)
	time.sleep(0.1)
	camera.capture(rawCapture, format="bgr")
	image = rawCapture.array

	return image


def checkColor(expected_color):
    print('INFO: Checking for color: ' + expected_color)
    image = getImageFromPicam([640,480])
    utilities_vision.saveImage(image, "images/step_0")
  
    output = utilities_vision.cropImage(image, expected_color)
    utilities_vision.saveImage(output, "images/step_1")
 
    color = utilities_vision.getColor(output,utilities_vision.HSV,expected_color)

    if(expected_color == color):
        print("INFO: The color matches: " + expected_color)
        return True
    else:
        print("INFO: The color doesn't match: " + expected_color)
        return False

def run_PiCam_server():
    # ----------------------------------------------------------------------- # 
    # initialize your data store
    # ----------------------------------------------------------------------- # 
    
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [1]*100),
        co=ModbusSequentialDataBlock(0, [17]*100),
        hr=ModbusSequentialDataBlock(0, [0]*100),
        ir=ModbusSequentialDataBlock(0, [17]*100))
    context = ModbusServerContext(slaves=store, single=True)
    
    # ----------------------------------------------------------------------- # 
    # initialize the server information
    # ----------------------------------------------------------------------- # 
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Robot system design'
    identity.ProductCode = 'PI'
    
    # ----------------------------------------------------------------------- # 
    # run the server you want
    # ----------------------------------------------------------------------- # 
    time = utilities_modbus.REGISTER_REFRESH_FREQUENCY
    loop = LoopingCall(f=updating_server, a=(context,))
    loop.start(time, now=False) # initially delay by time
    StartTcpServer(context, identity=identity, address=(utilities_modbus.IP, utilities_modbus.PORT))


if __name__ == "__main__":
    run_PiCam_server()
