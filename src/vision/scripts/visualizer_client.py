#!/usr/bin/env python

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import modbus_utilities as utilities_modbus
import vision_utilities as utilities_vision
import cv2
import time
import numpy as np


# Create a client and connect to the camera server
client = ModbusClient(utilities_modbus.IP, port=utilities_modbus.PORT)
client.connect()

if __name__ == "__main__":
    
    while True:
        # Read the result register
        rr = client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT)

	    # While the result register is on wait (no result published), read the result register 
        while rr.registers[1] == utilities_modbus.RESULT_WAIT: 
            rr = client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT)
            time.sleep(utilities_modbus.VISUALIZER_REQUEST_RATE )

        # Load the images showing the different steps in the image processing (input/cropping/hsv_filter)
        state,image_0 = utilities_vision.loadImage("images/step_0.jpg")
        state,image_1 = utilities_vision.loadImage("images/step_1.jpg")
        state,image_2 = utilities_vision.loadImage("images/step_2.jpg")
        padded_1 = utilities_vision.addPaddingToMatchSize(image_1,image_0)
        padded_2 = utilities_vision.addPaddingToMatchSize(image_2,image_0)
	
		# Show results:
        cv2.imshow("Result",np.hstack([image_0, padded_1, padded_2]))
        cv2.waitKey(100)
