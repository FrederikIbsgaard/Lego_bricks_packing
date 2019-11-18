#!/usr/bin/env python

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import modbus_utilities as utilities_modbus
# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
import time
log = logging.getLogger()
log.setLevel(logging.DEBUG)

# Create a client and connect to the camera server
client = ModbusClient(utilities_modbus.IP, port=utilities_modbus.PORT)
client.connect()

def run_sync_client(modbus_client,request):
    # Read the registers
    rr = modbus_client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT) 
    if(rr.registers[0] == utilities_modbus.GET_BRICK_COLOR_WAIT):
	print("INFO: Server is ready (waiting for order)")
        if(rr.registers[1] == utilities_modbus.RESULT_WAIT):
            print("INFO: Server is ready (result slot available)")
            if(request == utilities_modbus.GET_BRICK_COLOR_BLUE):
                if sendRequest(modbus_client,request):
                    checkResult(modbus_client,request)  
                    
            elif(request == utilities_modbus.GET_BRICK_COLOR_RED):
                if sendRequest(modbus_client,request):
                    checkResult(modbus_client,request)    

            elif(request == utilities_modbus.GET_BRICK_COLOR_YELLOW):
                if sendRequest(modbus_client,request):
                    checkResult(modbus_client,request)  
            else:
                print("ERROR: Invalid color request")  
        else:
            print("INFO: Server is busy (result slot available)")
    else:
        print("INFO: Server is busy (computing order)")

def sendRequest(modbus_client,request):
    # Send request
    rq = modbus_client.write_register(address=utilities_modbus.ADDRESS, value=request, unit=utilities_modbus.UNIT)
    # Check that the request arrives correctly
    rr = modbus_client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT)
    if(rr.registers[0] == request):
       print("INFO: Request was succesful")
       return True
    else:
       print("INFO: Request was not succesful")
       return False

# Function to know which is the equivalence between the register values and the colors:
def whichColor(request):
    if request == utilities_modbus.GET_BRICK_COLOR_RED:
        return utilities_modbus.RED_BRICK
    elif request == utilities_modbus.GET_BRICK_COLOR_BLUE:
        return utilities_modbus.BLUE_BRICK
    elif request == utilities_modbus.GET_BRICK_COLOR_YELLOW:
        return utilities_modbus.YELLOW_BRICK
    else:
        return "NULL"

def checkResult(modbus_client,request):
    # Read the registers
    rr = modbus_client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT)
    # Since it may take some time to get the results, loop until the result register is not set in RESULT_WAIT anymore
    while rr.registers[1] == utilities_modbus.RESULT_WAIT: 
        rr = modbus_client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT)
        if(rr.registers[1] == utilities_modbus.RESULT_COLOR_MATCH):
            print("INFO: Color matches: " + whichColor(request))
            return True
        elif(rr.registers[1] == utilities_modbus.RESULT_COLOR_MISMATCH):
            print("INFO: Color doesn't match: " + whichColor(request))
            return False

if __name__ == "__main__":

    request = utilities_modbus.GET_BRICK_COLOR_RED
    run_sync_client(client,request)
    time.sleep(5)
    request = utilities_modbus.GET_BRICK_COLOR_BLUE
    run_sync_client(client,request)
    time.sleep(5)
    request = utilities_modbus.GET_BRICK_COLOR_YELLOW
    run_sync_client(client,request)
    time.sleep(5)
    client.close()


