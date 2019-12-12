#!/usr/bin/env python
from vision.srv import check_brick, check_brickResponse
from local_log.msg import system_log_msg
from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import modbus_utilities as utilities_modbus
# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
import time
import rospy
from std_msgs.msg import String

log = logging.getLogger()
log.setLevel(logging.DEBUG)

# Create a client and connect to the camera server
client = ModbusClient(utilities_modbus.IP, port=utilities_modbus.PORT)
client.connect()

# Logging information messages:
pub = rospy.Publisher('system_log', system_log_msg,queue_size=10)
info_msg = system_log_msg()
info_msg.type = "INFO"   
info_msg.node_id = "Vision node"
error_msg = system_log_msg()
error_msg.type = "ERROR"   
error_msg.node_id = "Vision node"

def run_sync_client(modbus_client,request):
    for i in range(0,5):
        try:
            # Read the registers
            rr = modbus_client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT) 
            if(rr.registers[0] == utilities_modbus.GET_BRICK_COLOR_WAIT):

                info_msg.desc="Server is ready (waiting for order)"
                pub.publish(info_msg)
                if(rr.registers[1] == utilities_modbus.RESULT_WAIT):
                    
                    info_msg.desc="Server is ready (result slot available)"
                    pub.publish(info_msg)
                    if(request == utilities_modbus.GET_BRICK_COLOR_BLUE):
                        if sendRequest(modbus_client,request):
                            return checkResult(modbus_client,request)  
                            
                    elif(request == utilities_modbus.GET_BRICK_COLOR_RED):
                        if sendRequest(modbus_client,request):
                            return checkResult(modbus_client,request)    

                    elif(request == utilities_modbus.GET_BRICK_COLOR_YELLOW):
                        if sendRequest(modbus_client,request):
                            return checkResult(modbus_client,request)  
                    else:
                        error_msg.desc="Invalid color request."
                        pub.publish(error_msg)
                else:
                    info_msg.desc="Server is busy (result slot available)"
                    pub.publish(info_msg)
            else:
                info_msg.desc="Server is busy (computing order)"
                pub.publish(info_msg)        
        
        except:
            error_msg.desc="Modbus server not available, rebooting the raspberry pi. Sleeping for 10 sec"
            pub.publish(error_msg)
            time.sleep(10)
    return -1
    

def handle_check_brick(req):

    info_msg.desc="Request recieved, checkin for color: " + req.color
    pub.publish(info_msg)

    if req.color == utilities_modbus.RED_BRICK:
        request = utilities_modbus.GET_BRICK_COLOR_RED 
    elif req.color == utilities_modbus.BLUE_BRICK:
        request = utilities_modbus.GET_BRICK_COLOR_BLUE
    elif req.color == utilities_modbus.YELLOW_BRICK:
        request = utilities_modbus.GET_BRICK_COLOR_YELLOW
    else:
        request = utilities_modbus.GET_BRICK_COLOR_WAIT

    result = run_sync_client(client,request)
    return check_brickResponse(result)

def sendRequest(modbus_client,request):
    # Send request
    rq = modbus_client.write_register(address=utilities_modbus.ADDRESS, value=request, unit=utilities_modbus.UNIT)
    # Check that the request arrives correctly
    rr = modbus_client.read_holding_registers(address=utilities_modbus.ADDRESS, count=3, unit=utilities_modbus.UNIT)
    if(rr.registers[0] == request):

       info_msg.desc="Request was succesful"
       pub.publish(info_msg)
       return True
    else:
       error_msg.desc="Request was not succesful"
       pub.publish(error_msg)
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
            info_msg.desc="Color matches: " + whichColor(request)
            pub.publish(info_msg)
            return True
        elif(rr.registers[1] == utilities_modbus.RESULT_COLOR_MISMATCH):
            info_msg.desc="Color doesn't match: " + whichColor(request)
            pub.publish(info_msg)
            return False

def CheckBrickColor(color):
    
    rospy.init_node('CheckBrickColor')

    s = rospy.Service('check_brick', check_brick, handle_check_brick)
    info_msg.desc="ROS SERVICE: Ready to CheckBrickColor."
    pub.publish(info_msg)
    rospy.spin()

if __name__ == "__main__":

    CheckBrickColor("null")
    client.close()