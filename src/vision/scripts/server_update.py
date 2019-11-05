#!/usr/bin/env python

from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer
from twisted.internet.task import LoopingCall
import modbus_utilities as utilities_modbus
import vision_utilities as utilities_vision
# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
import subprocess
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)


def updating_writer(a):
    log.debug("updating the context")
    context = a[0]
    register = 3
    slave_id = 0x00
    address = 0x10
    values = context[slave_id].getValues(register, address, count=1)
    #values = [v + 1 for v in values]
    log.debug("new values: " + str(values))
    context[slave_id].setValues(register, address, values)

    if(values[0]== utilities_modbus.GET_BRICK_COLOR_BLUE):
        checkColor('blue')
    elif(values[0]== utilities_modbus.GET_BRICK_COLOR_RED):
        checkColor('red')
    elif(values[0]== utilities_modbus.GET_BRICK_COLOR_YELLOW):
        checkColor('yellow')
    else:
        print('ERROR: Invalid value in the register')


def checkColor(expected_color):
    print('Am i seeing a ' + expected_color + ' brick?')
    utilities_vision.getImageFromPicam()
    cmd = "raspistill -w 640 -h 480 -o lego_conf.jpg"
    subprocess.call(cmd, shell=True)
    state, image = utilities_vision.loadImage("lego_conf")
    output = utilities_vision.cropImage(image, expected_color)  
    color = utilities_vision.getColor(output,utilities_vision.RGB)
    if(expected_color == color):
        print("INFO: The color matches: " + color)
    else:
        print("INFO: The color doesn't match: " + color)

def run_updating_server():
    # ----------------------------------------------------------------------- # 
    # initialize your data store
    # ----------------------------------------------------------------------- # 
    
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [1]*100),
        co=ModbusSequentialDataBlock(0, [17]*100),
        hr=ModbusSequentialDataBlock(0, [1]*100),
        ir=ModbusSequentialDataBlock(0, [17]*100))
    context = ModbusServerContext(slaves=store, single=True)
    
    # ----------------------------------------------------------------------- # 
    # initialize the server information
    # ----------------------------------------------------------------------- # 
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'pymodbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
    identity.ProductName = 'pymodbus Server'
    identity.ModelName = 'pymodbus Server'
    identity.MajorMinorRevision = '2.3.0'
    
    # ----------------------------------------------------------------------- # 
    # run the server you want
    # ----------------------------------------------------------------------- # 
    time = utilities_modbus.REGISTER_REFRESH_FREQUENCY
    loop = LoopingCall(f=updating_writer, a=(context,))
    loop.start(time, now=False) # initially delay by time
    StartTcpServer(context, identity=identity, address=(utilities_modbus.IP, utilities_modbus.PORT))


if __name__ == "__main__":
    run_updating_server()
